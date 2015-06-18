"""A collection of functions that help during lookup of modules and interpretation."""

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