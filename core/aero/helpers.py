def lookupMessageType(message_type):
    """Look up a message type from a list given its acronym and returns the full message type.

    If the <message_type> is not an acronym from the list, return message_type.

    Keyword arguments:
    message_type -- the acronym to look up, or a non acronym message type
    """
    return message_type


def getElements(node):
    """Return a list of al dom elements in another element.

    Keyword arguments:
    node -- the parent element
    """
    elems = []
    for child in node.childNodes:
        if child.nodeType == child.ELEMENT_NODE:
            elems.append(child)
    return elems


def getElementsOnFirstLevel(parent, element):
    """Return a list of elements below parent, that have tagname element.

    Keyword arguments:
    parent -- the parent element
    element -- tagname of elements to return in the list
    """
    elements = []
    occurences = parent.getElementsByTagName(element)
    for e in occurences:
        if e.parentNode == parent:
            elements.append(e)
    return elements


def getOptionalAttribute(element, attribute):
    """Return the value of an attribute from an element,
    but do not crash if it does not exist.

    Return None if the attribute is not in the parent element.

    Keyword arguments:
    element -- the parent element that contains the attribute
    attribute -- the optional attribute to return the value of
    """
    if element.hasAttribute(attribute):
        return element.attributes[attribute].value
    else:
        return None


def findModuleById(available_mods, module_id):
    # Finds a module from a list of modules by its id and returns the module
    # Returns None if the module is not in the list
    module_found = None
    for available_mod in available_mods:
        if available_mod.id == module_id:
            module_found = available_mod
            break
    return module_found


def getElementsOnFirstLevelExceptTag(parent, element):
    elements = []
    children = getElements(parent)
    for c in children:
        if c.parentNode == parent and c.tagName.lower() != element.lower():
            elements.append(c)
    return elements