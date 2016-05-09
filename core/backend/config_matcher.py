"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""

"""Provide functions to match a configuration (abstraction) to module abstractions"""

from objects import *
from helpers import *


def matchConfig(configuration, available_mods):
    """Start matching modules and settings specified in *configuration* to available modules in *available_mods*.

    :param configuration: the abstraction of a configuration
    :param available_mods: list of all available modules (can be delivered by :func:`core.aero.scan.getAvailableModules` )
    """
    recursiveMatch(configuration, available_mods)
    return configuration


def recursiveMatch(parent, available_mods):
    """Find a the abstraction of an object mentioned in *parent* in *available_mods* and return.

    Also step into every level of *parent* recusively and finally return is an abstraction of *parent* with all children abstracted.

    :param parent: the to-be-implemented object
    :param available_mods: list of all available modules (can be delivered by :func:`core.aero.scan.getAvailableModules`
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
