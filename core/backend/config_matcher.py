"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""

"""Provide functions to match a configuration (abstraction) to module abstractions"""
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
