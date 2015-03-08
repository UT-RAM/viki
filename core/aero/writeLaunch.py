import xml.etree.cElementTree as ET
import xml.dom.minidom
import os
from objects import *
from helpers import *


def write(configuration, filename="aeroworks.launch"):
    # Create root element <launch>
    root = ET.Element("launch")
    recursiveWrite(configuration, root)
    f = open(filename, 'w')
    f.write(prettify(root))


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = xml.dom.minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")


def recursiveWrite(configPart, rootElem):
    # Loop through modules at current level
    for mod in configPart.modules_to_add:
        # Loop through executables for specific module
        for executable in mod.implementation.executables:
            node = ET.SubElement(rootElem, "node", pkg=executable.pkg, name=executable.id, type=executable.executable)
            # Check if one of the parameters that are to be set are present in this executable
            # Todo/problem: params are not defined at executable level, but at module level.
            for paramSearch in mod.parameters_to_add[:]:
                for paramList in executable.params[:]:
                    if paramSearch.name == paramList.name:
                        param = ET.SubElement(node, "param", name=paramSearch.name, value=paramSearch.value)
                        # Remove found param from both lists
                        mod.parameters_to_add.remove(paramSearch)
                        executable.params.remove(paramList)
            # At this point, we also have the default parameters we should fill. loop again.
            for paramList in executable.params:
                param = ET.SubElement(node, "param", name=paramList.name, value=paramList.default)
        # At this point, we end up with some parameters that are not "connected". Echo those.
        for paramSearch in mod.parameters_to_add:
            print 'Parameter "' + paramSearch.name + '", valued "' + paramSearch.value + '", could not be connected.'

    try:
        for ns in configPart.namespaces:
            ns_elem = ET.SubElement(rootElem, "namespace", id=ns.id)
            recursiveWrite(ns, ns_elem)
    except AttributeError:
        print 'No namespaces in configPart'