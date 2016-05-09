"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""

"""VIKI_VERSION_INFO
VIKI: more than a GUI for ROS 
version number: 0.1
version name: Alice

Copyright (c) 2016 Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, https://github.com/UT-RAM/viki

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Connection between the HTML GUI and Python backend was established using work by David Baird following his tutorial on http://www.aclevername.com/articles/python-webgui/

Thank you for letting us use your great work, David.

This work has been funded by the European Commission's H2020 project AEROWORKS under grant no. 644128
END_VERSION_INFO"""
import time
import Queue
import thread
import urllib

import gtk
import os
import gobject

try:
    import webkit
    have_webkit = True
except:
    have_webkit = False

try:
    import gtkmozembed
    have_gtkmozembed = True
except:
    have_gtkmozembed = False


class UseWebKit:
    pass


class UseGtkMozEmbed:
    pass

if False:
    pass
elif have_webkit:
    use = UseWebKit
elif have_gtkmozembed:
    use = UseGtkMozEmbed
else:
    raise Exception('Failed to load any of webkit and gtkmozembed modules')

# use = UseGtkMozEmbed # <- choose your desired implementation here


class WebKitMethods(object):

    @staticmethod
    def create_browser():
        return webkit.WebView()


    @staticmethod
    def inject_javascript(browser, script):
        browser.execute_script(script)

    @staticmethod
    def connect_title_changed(browser, callback):
        def callback_wrapper(widget, frame, title): callback(title)
        browser.connect('title-changed', callback_wrapper)

    @staticmethod
    def open_uri(browser, uri):
        browser.open(uri)


class GtkMozEmbedMethods(object):

    @staticmethod
    def create_browser():
        return gtkmozembed.MozEmbed()

    @staticmethod
    def inject_javascript(browser, script):
        uri = 'javascript:%s' % urllib.quote(script + '\n;void(0);')
        browser.load_url(uri)

    @staticmethod
    def connect_title_changed(browser, callback):
        # XXX: probably you should cross your fingers and hope browser
        #      isn't sending title messages too quickly...?
        def callback_wrapper(*args): callback(browser.get_title())
        browser.connect('title', callback_wrapper)

    @staticmethod
    def open_uri(browser, uri):
        browser.load_url(uri)


if use is UseWebKit:
    implementation = WebKitMethods

if use is UseGtkMozEmbed:
    implementation = GtkMozEmbedMethods


def asynchronous_gtk_message(fun):

    def worker((function, args, kwargs)):
        apply(function, args, kwargs)

    def fun2(*args, **kwargs):
        gobject.idle_add(worker, (fun, args, kwargs))

    return fun2


def synchronous_gtk_message(fun):

    class NoResult:
        pass

    def worker((R, function, args, kwargs)):
        R.result = apply(function, args, kwargs)

    def fun2(*args, **kwargs):
        class R:
            result = NoResult
        gobject.idle_add(worker, (R, fun, args, kwargs))
        while R.result is NoResult:
            time.sleep(0.01)
        return R.result

    return fun2

def activate_inspector(inspector, target_view):
    inspectorwindow = gtk.Window()
    inspectorwindow.set_default_size(800, 600)
    inspectorbox = gtk.VBox(homogeneous=False, spacing=0)
    inspectorwindow.add(inspectorbox)
    inspectorview = webkit.WebView()
    inspectorbox.pack_end(inspectorview)
    inspectorwindow.show_all()
    return inspectorview


def launch_browser(uri, quit_function=None, echo=True):

    window = gtk.Window()

    file = os.path.abspath('core/gui/img/viki_logo.png')
    window.set_icon_from_file(file)
    browser = implementation.create_browser()
    browser.get_settings().set_property("enable-developer-extras", True)

    sw = gtk.ScrolledWindow()
    sw.set_policy(gtk.POLICY_NEVER, gtk.POLICY_NEVER)
    sw.add(browser)

    # box = gtk.VBox(homogeneous=False, spacing=0)
    window.add(sw)

    inspector = browser.get_web_inspector()
    inspector.connect("inspect-web-view", activate_inspector)

    if quit_function is not None:
            # TODO: Fix this for the scrollable window, since the Box function isn't being used anymore
            # Obligatory "File: Quit" menu
            # {
            file_menu = gtk.Menu()
            quit_item = gtk.MenuItem('Quit')
            accel_group = gtk.AccelGroup()
            quit_item.add_accelerator('activate',
                                      accel_group,
                                      ord('Q'),
                                      gtk.gdk.CONTROL_MASK,
                                      gtk.ACCEL_VISIBLE)
            window.add_accel_group(accel_group)
            file_menu.append(quit_item)
            quit_item.connect('activate', quit_function)
            quit_item.show()
            #
            menu_bar = gtk.MenuBar()
            menu_bar.show()
            file_item = gtk.MenuItem('File')
            file_item.show()
            file_item.set_submenu(file_menu)
            menu_bar.append(file_item)
            # }
            # box.pack_start(menu_bar, expand=False, fill=True, padding=0)

    if quit_function is not None:
        window.connect('destroy', quit_function)

    # box.pack_start(browser, expand=False, fill=False, padding=0)

    window.set_default_size(800, 600)
    window.show_all()
    window.maximize()

    message_queue = Queue.Queue()

    def title_changed(title):
        if title != 'null':
            message_queue.put(title)

    implementation.connect_title_changed(browser, title_changed)

    implementation.open_uri(browser, uri)

    def web_recv():
        if message_queue.empty():
            return None
        else:
            msg = message_queue.get()
            if echo:
                print '>>>', msg
            return msg

    def web_send(msg):
        if echo:
            print '<<<', msg
        asynchronous_gtk_message(implementation.inject_javascript)(browser,
                                                                   msg
                                                                   )

    return browser, web_recv, web_send


def start_gtk_thread():
    # Start GTK in its own thread:
    gtk.gdk.threads_init()
    thread.start_new_thread(gtk.main, ())


def kill_gtk_thread():
    asynchronous_gtk_message(gtk.main_quit)()
