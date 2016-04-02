"""A collection of functions that help during lookup of modules and interpretation."""
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

import json

def lookupMessageType(message_type):
    """Look up a message type from a list given its acronym *message_type* and returns the full message type.

    If the *message_type* is not an acronym from the list, return *message_type*.

    :param message_type: the acronym to look up, or a non acronym message type
    """
    return message_type


def getElements(node):
    """Return a list of al dom elements in the element *node*.

    :param node: the parent element
    """
    elems = []
    for child in node.childNodes:
        if child.nodeType == child.ELEMENT_NODE:
            elems.append(child)
    return elems


def getElementsOnFirstLevel(parent, element):
    """Return a list of elements below *parent*, that have tagname *element*.

    :param parent: the parent element
    :param element: tagname of elements to return in the list
    """
    elements = []
    occurences = parent.getElementsByTagName(element)
    for e in occurences:
        if e.parentNode == parent:
            elements.append(e)
    return elements


def getOptionalAttribute(element, attribute):
    """Return the value of an *attribute* from an *element*,
    but do not crash/throw if *attribute* does not exist.

    Return None if *attribute* is not in the parent *element*.

    :param element: the parent element that (might) contain(s) the attribute
    :param attribute: the optional attribute to return the value of
    """
    if element.hasAttribute(attribute):
        return element.attributes[attribute].value
    else:
        return None


def findModuleById(available_mods, module_id):
    """Find a module from list of modules (*available_mods*) by its *module_id* and return the module.

    Return None if the module is there is not a module with id *module_id* in *available_mods*.

    :param available_mods: list of all available modules
    :param module_id: id of the module to find
    """
    module_found = None
    for available_mod in available_mods:
        if available_mod.id == module_id:
            module_found = available_mod
            break
    return module_found


def getElementsOnFirstLevelExceptTag(parent, element):
    """Return all elements below *parent* except for the ones tagged *element*.

    :param parent: the parent dom object
    :param elemnt: the tag-name of elements **not** to return
    """
    elements = []
    children = getElements(parent)
    for c in children:
        if c.parentNode == parent and c.tagName.lower() != element.lower():
            elements.append(c)
    return elements


def toJSON(py_object):
    """Convert an object to JSON string using its __dict__

    :param py_object: object to create a JSON string of
    """

    return json.dumps(py_object, default=lambda o: o.__dict__)
