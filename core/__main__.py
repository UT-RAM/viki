import time
from aero.objects import *
from aero import scan
from aero import config_interpreter

available_mods = scan.getAvailableModules()
print "Got all the modules"

configuration = config_interpreter.getConfig()


# WAIT BEFORE EXIT
print "Finished, use a breakpoint on this line to avoid exit."
