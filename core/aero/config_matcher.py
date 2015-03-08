from objects import *
from helpers import *


def matchConfig(configuration, available_mods):
    recursiveMatch(configuration, available_mods)
    return configuration


def recursiveMatch(parent, available_mods):
    for module_to_add in parent.modules_to_add:
        impl = helpers.findModuleById(available_mods, module_to_add.type)
        if impl is None:
            print 'I was looking for a module with id: ' + module_to_add.type
            raise Exception("Could not find the requested module")
        else:
            module_to_add.implementation = impl

    try:
        for namespace in parent.namespaces:
            recursiveMatch(namespace, available_mods)
    except AttributeError:
        pass
