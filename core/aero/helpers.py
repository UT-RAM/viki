def lookupMessageType(message_type):
    return message_type


def getElements(node):
    # Empty
    elems = []
    for child in node.childNodes:
        if child.nodeType == child.ELEMENT_NODE:
            elems.append(child)
    return elems


def getElementsOnFirstLevel(parent, element):
    elements = []
    occurences = parent.getElementsByTagName(element)
    for e in occurences:
        if e.parentNode == parent:
            elements.append(e)
    return elements


def getOptionalAttribute(element, attribute):
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