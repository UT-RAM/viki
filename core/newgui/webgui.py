import htmlPy
import os
import sys
import subprocess
import signal

# needed to import aero modules
# sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(os.path.abspath('../'))
from aero import scan
from aero import helpers
from aero import config_interpreter
from aero import config_matcher
from aero import writeLaunch

app = htmlPy.AppGUI(title=u"VIKI GUI",
                    maximized=True,
                    developer_mode=True,
                    plugins=True)
app.template_path = os.path.abspath("./dist/")
app.static_path = os.path.abspath("./dist/")
app.template = ("index.html", {})


class BackendConfig(htmlPy.Object):

    BASE_DIR = os.path.abspath(os.path.dirname(__file__))

    @htmlPy.Slot(result=str)
    def get_base_dir(self):
        return "file:///" + self.BASE_DIR + "/dist/"


class VikiBackend(htmlPy.Object):
    def __init__(self):
        super(VikiBackend, self).__init__()
        self.corePID = 0  # placeholder for process id of roscore
        self.available_mods = []  # placeholder for available mods
        self.ros_master_hostname = 'localhost'  # ros master hostname

    @htmlPy.Slot()
    def connCheck(self):
        self.vikiLog('Communication ok!')
        return

    @htmlPy.Slot(result=str)
    def getModules(self):
        self.available_mods = scan.getAvailableModules()
        print('Got all (%i) modules, returning as JSON' % len(self.available_mods))
        return helpers.toJSON(self.available_mods)

    @htmlPy.Slot(result=str)
    def startRosCore(self):
        sp = subprocess.Popen(['gnome-terminal', '-x', '/opt/ros/indigo/bin/roscore'])
        self.corePID = sp.pid  # This PID is not the right one!
        app.evaluate_javascript("enableStopCore();")
        self.vikiLog('ROS core started, PID: %s' % str(sp.pid))

        return str(sp.pid)

    @htmlPy.Slot()
    def stopRosCore(self):
        # if self.corePID > 0: 
        #     self.vikiLog('Killing process %s' % self.corePID)
        #     os.kill(self.corePID, signal.SIGKILL)

        # TODO user feedback
        app.evaluate_javascript("enableStartCore();")

    @htmlPy.Slot(str)
    def writeConfigXML(self, xml):
        filename = 'configuration.xml'
        with open(filename, 'w') as f:
            f.write('<configurations>')
            f.write(xml)
            f.write('</configurations>')
            self.vikiLog('Configuration written to configuration.xml')
            return

        self.vikiLog('Unable to write to configuration.xml')
        return

    @htmlPy.Slot()
    def configLaunch(self):
        configfromfile = config_interpreter.getCofig(config_id_to_use="VIKI-imported-config")
        matchCfg = config_matcher.matchConfig(configfromfile, self.available_mods)
        writeLaunch.write(configfromfile)
        # mabe this should be:
        # writeLaunch.write(matchCfg)
        self.vikiLog('Written launch file')
        return

    @htmlPy.Slot()
    def run(self):
        try:
            env = "http://"+self.ros_master_hostname+":11311"
            pid = subprocess.Popen(args=["gnome-terminal", "-e", "./viki_launch.sh roslaunch aeroworks.launch"], env=dict(os.environ, **{"ROS_MASTER_URI":env})).pid
            self.vikiLog('Requested launch of aeroworks.launch')
        except OSError:
            self.vikiLog('Unable to start process (OSError)')

    @htmlPy.Slot()
    def setMasterUri(self, uri):
        self.ros_master_hostname = uri
        self.vikiLog('Updated ROS_MASTER_URI to %s' % uri)
        return

    @htmlPy.Slot()
    def makeAndRun(self, configXML):
        self.configXML(configXML)
        self.configLaunch()
        self.run()
        return

    @htmlPy.Slot()
    def makeNoRun(self, configXML):
        self.configXML(configXML)
        self.configLaunch()
        return

    @htmlPy.Slot()
    def showLaunch(self):
        self.vikiLog('Opening generated launch file...')
        subprocess.Popen(args=["xdg-open", "aeroworks.launch"])

    @htmlPy.Slot()
    def showConfig(self):
        self.vikiLog('Opening generated config file...')
        subprocess.Popen(args=["xdg-open", "aeroworks.launch"])
        return

    def vikiLog(self, msg):
        # TODO prepend time here
        print(msg)
        app.evaluate_javascript("console.log('"+msg+"');")
        return


app.bind(BackendConfig())
app.bind(VikiBackend())

def main():
    app.start()

# for easy running in debug, remove later because main()
# will be called from runnables.py
if __name__ == '__main__':
    main()