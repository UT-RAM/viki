from objects import *
from helpers import *


def matchConfig(configuration, available_mods):
    for namespace in configuration.namespaces:
        for module_to_add in namespace.modules_to_add:
            impl = helpers.findModuleById(available_mods, module_to_add.type)
            if impl is None:
                print 'I was looking for a module with id: ' + module_to_add.type
                raise Exception("Could not find the requested module")
            else:
                module_to_add.implementation = impl

    for module_to_add in configuration.modules_to_add:
        impl = helpers.findModuleById(available_mods, module_to_add.type)
        if impl is None:
            print 'I was looking for a module with id: ' + module_to_add.type
            raise Exception("Could not find the requested module")
        else:
            module_to_add.implementation = impl

    return configuration