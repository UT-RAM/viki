"""Start the Aeroworks robot-framework core.

Call various other functions to scan for available modules in a file tree to finally build a launch file for a specific configuration.

:param configfilename: the (relative) filename of a configuration file (can be given as first argument via command line) (default configuration.xml)
:param config_id_to_use: the id of a specific configuration within *configfilename* to be used (can be given as second argument via command line) (default None)
"""
import sys
from aero import scan
from aero import config_interpreter
from aero import config_matcher
from aero import writeLaunch
from aero import helpers

available_mods = scan.getAvailableModules()
print "Got all the modules"

configfilename = None
config_id = None
if len(sys.argv) > 1:
    configfilename = sys.argv[1]
if len(sys.argv) > 2:
    config_id = sys.argv[2]
configuration = config_interpreter.getConfig(configfilename=configfilename, config_id_to_use=config_id)
config_matcher.matchConfig(configuration, available_mods)
writeLaunch.write(configuration)


# WAIT BEFORE EXIT
print "Finished, use a breakpoint on this line to avoid exit."
