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
