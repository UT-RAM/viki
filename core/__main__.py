from aero import scan
from aero import config_interpreter
from aero import config_matcher
from aero import helpers

available_mods = scan.getAvailableModules()
print "Got all the modules"

configuration = config_interpreter.getConfig()
config_matcher.matchConfig(configuration, available_mods)

# WAIT BEFORE EXIT
print "Finished, use a breakpoint on this line to avoid exit."
