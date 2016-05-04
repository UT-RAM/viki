#!/usr/bin/python

"""Start the Aeroworks robot-framework core.

Call various other functions to scan for available modules in a file tree to finally build a launch file for a specific configuration.

:param configfilename: the (relative) filename of a configuration file (can be given as first argument via command line) (default configuration.xml)
:param config_id_to_use: the id of a specific configuration within *configfilename* to be used (can be given as second argument via command line) (default None)
"""
# import sys
from backend import scan
from backend import config_interpreter
from backend import config_matcher
from backend import writeLaunch
from backend import helpers

# imports, directly from:  http://www.aclevername.com/articles/python-webgui/
import signal
import os
import time
import urllib
import subprocess

# from simplejson import dumps as to_json
from simplejson import loads as from_json

from gui.webgui import start_gtk_thread
from gui.webgui import launch_browser
from gui.webgui import synchronous_gtk_message
# from gui.webgui import asynchronous_gtk_message
from gui.webgui import kill_gtk_thread
# end of imports from internet

from cli.viki_config import VikiConfig

import gtk

ros_master_hostname = "localhost"

# from: http://www.aclevername.com/articles/python-webgui/
class Global(object):
    quit = False

    @classmethod
    def set_quit(cls, *args, **kwargs):
        cls.quit = True

def main():
    viki_config = VikiConfig()
    available_mods = scan.getAvailableModules(viki_config)

    start_gtk_thread()
    corePID = 0

    # Create a proper file:// URL pointing to demo.xhtml:
    file = os.path.abspath('core/gui/VIKI_main.html')
    uri = 'file://' + urllib.pathname2url(file)
    browser, web_recv, web_send = \
        synchronous_gtk_message(launch_browser)(uri,
                                                quit_function=Global.set_quit)

    # Predefine functions that react on buttons in GUI
    def vikiConnCheck():
        web_send('updateStatus("Communication okay")')

    def vikiRefreshModules():
        web_send('updateModules(%s)' %
                 helpers.toJSON(available_mods))

    def vikiStartRosCore():
        # sp = subprocess.Popen('/opt/ros/indigo/bin/roscore')
        sp = subprocess.Popen(['gnome-terminal', '-x', "{}/bin/roscore".format(viki_config.get_option('ros_dir'))])
        main.corePID = sp.pid # This PID is not the right one!
        web_send('enableStopCore()')
        web_send('updateStatus("ROS core started (PID: '+str(main.corePID)+')")')

    def vikiStopRosCore():
        # if main.corePID > 0: 
        #     os.kill(main.corePID, signal.SIGKILL)
        web_send('enableStartCore()')

    def vikiConfigXML(configXML):
        filename = 'configuration.xml'
        f = open(filename, 'w')
        f.write('<configurations>')
        f.write(configXML)
        f.write('</configurations>')
        f.close()

    def vikiConfigLaunch():
        configfromfile = config_interpreter.getConfig(config_id_to_use="VIKI-imported-config")     
        config_matcher.matchConfig(configfromfile, available_mods)
        writeLaunch.write(configfromfile)
        web_send('updateStatus("Written launch file")');

    def vikiRun():
        try:
            # run in new gnome terminal
            # don't know if this works with intellijIDEA, find out yourself if you use it.
            global ros_master_hostname
            env = "http://"+ros_master_hostname+":11311"
            pid = subprocess.Popen(args=["gnome-terminal", "-e", "./viki_launch.sh roslaunch aeroworks.launch"], env=dict(os.environ, **{"ROS_MASTER_URI":env})).pid
        except OSError:
            web_send('updateStatus("OSError")')
        web_send('updateStatus("Requested launch of AeroWorks.launch")')

    def vikiSetMasterUri(master_hostname):
        global ros_master_hostname
        ros_master_hostname= master_hostname
        web_send('updateStatus("Updated ROS_MASTER_URI to '+ros_master_hostname+'")')

    def vikiMakeAndRun(configXML):
        vikiConfigXML(configXML)
        vikiConfigLaunch()
        vikiRun()

    def vikiMakeNoRun(configXML):
        vikiConfigXML(configXML)
        vikiConfigLaunch()

    def vikiShowLaunch():
        web_send('updateStatus("Opening generated launch file...")')
        subprocess.Popen(args=["xdg-open", "aeroworks.launch"])

    def vikiShowConfig():
        web_send('updateStatus("Opening generated config file...")')
        subprocess.Popen(args=["xdg-open", "configuration.xml"])

    def vikiOpenRqtGraph():
        subprocess.Popen(["gnome-terminal", '-e', "./viki_launch.sh rqt_graph"])

    def add_filters(dialog):
        filter_text = gtk.FileFilter()
        filter_text.set_name("Text files")
        filter_text.add_mime_type("text/plain")
        dialog.add_filter(filter_text)

        filter_any = gtk.FileFilter()
        filter_any.set_name("Any files")
        filter_any.add_pattern("*")
        dialog.add_filter(filter_any)

    def vikiSave(project):
        dialog = gtk.FileChooserDialog("Please enter a file name",None,
                                       gtk.FILE_CHOOSER_ACTION_SAVE,
                                       (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                        gtk.STOCK_SAVE, gtk.RESPONSE_OK))

        #add_filters(dialog)

        gtk.gdk.threads_enter()
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            f = open(dialog.get_filename(), mode='w+')
            f.write(project)
            web_send('updateStatus("Save finished")')
        elif response == gtk.RESPONSE_CANCEL:
            web_send('updateStatus("Save cancelled")')
        dialog.destroy()
        gtk.gdk.threads_leave()

    def vikiOpen():
        dialog = gtk.FileChooserDialog("Please choose a file",None,
                                       gtk.FILE_CHOOSER_ACTION_OPEN,
                                       (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                        gtk.STOCK_OPEN, gtk.RESPONSE_OK))

        #add_filters(dialog)

        gtk.gdk.threads_enter()
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            f = open(dialog.get_filename(), mode='r')
            web_send('openFromJSON('+f.read()+')')
            web_send('updateStatus("Open finished")')
        elif response == gtk.RESPONSE_CANCEL:
            web_send('updateStatus("Open cancelled")')
        dialog.destroy()
        gtk.gdk.threads_leave()

    # Finally, here is our personalized main loop, 100% friendly
    # with "select" (although I am not using select here)!:
    clicks = -1
    while not Global.quit:

        again = False
        msg = web_recv()
        if msg:
            try:
                msg = from_json(msg)
            except ValueError:
                print "No valid JSON has been send.."
                continue

            again = True

            # Check if the message starts with lowercase viki. This indicates that there is a matching function with the same name in Python that should be executed.
            if msg['name'].startswith("viki"):
                try:
                    if msg['value'] == False:
                        locals()[msg['name']]()
                    else:
                        locals()[msg['name']](msg['value'])
                except KeyError:
                    web_send('updateStatus("Function '+msg['name']+' not found")')

        if again:
            pass
        else:
            time.sleep(0.1)

def my_quit_wrapper(fun):
    signal.signal(signal.SIGINT, Global.set_quit)

    def fun2(*args, **kwargs):
        try:
            x = fun(*args, **kwargs)  # equivalent to "apply"
        finally:
            kill_gtk_thread()
            Global.set_quit()
        return x
    return fun2

def run():
    my_quit_wrapper(main)()
