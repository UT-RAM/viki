"""Provide functions to match a configuration (abstraction) to module abstractions"""
from objects import *
from helpers import *


def matchConfig(configuration, available_mods):
    """Start matching configuration to available modules_to_add

    :param configuration: the abstraction of a configuration
    :param available_mods: list of all available modules (as delivered by *scan.py*)
    """
    recursiveMatch(configuration, available_mods)
    return configuration


def recursiveMatch(parent, available_mods):
    """Recursively find all modules of *parent*.

    :param parent: the to-be-implemented object
    :param available_mods: list of all available modules
    """
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
