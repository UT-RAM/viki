from aero import scan
from aero import config_interpreter
from aero import config_matcher
from aero import writeLaunch
from aero import helpers

available_mods = scan.getAvailableModules()
print "Got all the modules"

configuration = config_interpreter.getConfig(config_id_to_use='simple')
config_matcher.matchConfig(configuration, available_mods)
writeLaunch.write(configuration)


# WAIT BEFORE EXIT
print "Finished, use a breakpoint on this line to avoid exit."
