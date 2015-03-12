import xml.etree.cElementTree as ET
import xml.dom.minidom
import os
from objects import *
from helpers import *


def write(configuration, filename="aeroworks.launch"):
    # Create root element <launch>
    root = ET.Element("launch")
    recursiveWrite(configuration, root, root)
    f = open(filename, 'w')
    f.write(prettify(root))


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = xml.dom.minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")


def recursiveWrite(configPart, configElem, rootElem, path=''):
    # Loop through modules at current level
    for mod in configPart.modules_to_add:
        # Loop through executables for specific module
        for executable in mod.implementation.executables:
            nodens = ET.SubElement(configElem, "group", ns=mod.id+'_'+executable.id)
            node = ET.SubElement(nodens, "node", pkg=executable.pkg, name=mod.id+'_'+executable.id, type=executable.executable)
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

    for con in configPart.connections_to_add:
        from_attr = lookup(configPart, con.listener, path)
        to_attr = lookup(configPart, con.publisher, path)
        remap = ET.SubElement(rootElem, "remap", to=to_attr)
        remap.set('from', from_attr)

    try:
        for ns in configPart.namespaces:
            ns_elem = ET.SubElement(rootElem, "group", ns=ns.id)
            recursiveWrite(ns, ns_elem, rootElem, path+'/'+ns.id)
    except AttributeError:
        print 'No namespaces in configPart'


def lookup(configPart, string, path):
    print 'Lookup started'
    # Split string in parts (basically the address)
    parts = string.split('/')
    # If the number of parts is smaller than two, something is wrong.
    if len(parts) < 2:
        raise Exception("Incorrect connect-statement")
    connectionString = ''
    while len(parts) > 2:
        connectionString += parts[0] + '/'
        for ns in configPart.namespaces:
            if ns.id == parts[0]:
                configPart = ns
        parts.pop(0)
    # Correct namespace is found, now find node name
    for mod in configPart.modules_to_add:
        if mod.id == parts[0]:
            interfaces = mod.implementation.outputs+mod.implementation.inputs
            for con in interfaces:
                if con.name == parts[1]:
                    linkparts = con.link.split('/')
                    linkparts[0] = mod.id + '_' + linkparts[0]
                    connectionString += linkparts[0] + '/' + linkparts[1]
                    break
            break
    return path + '/' + connectionString