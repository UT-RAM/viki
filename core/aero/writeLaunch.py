import xml.etree.cElementTree as ET
import xml.dom.minidom
import os
import time
import random
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
    for con in configPart.connections_to_add:
        from_attr = lookup(configPart, con.listener, path)
        to_attr = lookup(configPart, con.publisher, path)

        # NEW REMAP
        if from_attr is not to_attr:
            relayElement = ET.SubElement(rootElem, 'node')
            relayElement.set('name', '$(anon remap_'
                             + str(random.random()) + ')')
            relayElement.set('pkg', 'topic_tools')
            relayElement.set('type', 'relay')
            relayElement.set('args', to_attr + ' ' + from_attr)
            relayElement.set('ns', 'remaps')

        # OLD REMAP
        # remap = ET.SubElement(rootElem, "remap")
        # remap.set('to', to_attr)
        # remap.set('from', from_attr)

    for mod in configPart.modules_to_add:
        # Loop through executables for specific module
        for ic in mod.implementation.config:
            from_attr = path + '/' + lookupInternal(ic.listener, mod)
            to_attr = path + '/' + lookupInternal(ic.publisher, mod)

            # THIS IS THE NEW REMAP
            # relayElement = ET.SubElement(rootElem, 'node')
            # relayElement.set('name', '$(anon remap_' +  str(random.random()) + ')')
            # relayElement.set('pkg', 'topic_tools')
            # relayElement.set('type', 'relay')
            # relayElement.set('args', to_attr + ' ' + from_attr)
            # relayElement.set('ns', 'remaps')

            # THIS IS THE OLD REMAP
            remap = ET.SubElement(rootElem, "remap")
            remap.set('to', to_attr)
            remap.set('from', from_attr)

        for executable in mod.implementation.executables:
            nodens = ET.SubElement(configElem, "group", ns=mod.id+'_'+executable.id)
            node = ET.SubElement(nodens, "node", pkg=executable.pkg, name=mod.id+'_'+executable.id, type=executable.executable)
            # Check if one of the parameters that are to be set are present in this executable
            # Todo/problem: params are not defined at executable level, but at module level.
            for paramSearch in mod.parameters_to_add[:]:
                print len(executable.params)
                for paramList in executable.params[:]:
                    print "mathcing parameters:"
                    print paramSearch.name
                    print paramList.name
                    if paramSearch.name == paramList.name:
                        print "found a mathcing parameter!!"
                        param = ET.SubElement(node, "param", name=paramSearch.name, value=paramSearch.value)
                        # Remove found param from both lists
                        mod.parameters_to_add.remove(paramSearch)
                        executable.params.remove(paramList)
                    else:
                        print "params did not match"

            # At this point, we also have the default parameters we should fill. loop again.
            for paramList in executable.params:
                param = ET.SubElement(node, "param", name=paramList.name, value=paramList.default)

            # find any command line arguments that belong to this executable
            for argSearch in mod.args:
                if argSearch.execid == executable.id:
                    node.attrib['args'] = argSearch.argument

            # find any prefixes for this executable
            for prefixSearch in mod.prefixes:
                if prefixSearch.execid == executable.id:
                    node.attrib['launch-prefix'] = prefixSearch.prefix

        # At this point, we end up with some parameters that are not "connected". Echo those.
        for paramSearch in mod.parameters_to_add:
            print 'Parameter "' + paramSearch.name + '", valued "' + paramSearch.value + '", could not be connected.'

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
    # If the number of parts is smaller than two, something is wrong. More is possible (but not recommended)
    if len(parts) < 2:
        raise Exception("Incorrect connect-statement")
    connectionString = ''
    # Keep checking if the first part of the connection string matches a namespace. If so: add it to the final connection string and pop it off.
    at_end = False
    while at_end is False:
        matchFound = False
        for ns in configPart.namespaces:
            if ns.id == parts[0]:
                matchFound = True
                configPart = ns
                connectionString += parts[0] + '/'
                parts.pop(0)
        if matchFound is False:
            at_end = True
    # Correct namespace is found, now find node name
    for mod in configPart.modules_to_add:
        if mod.id == parts[0]:
            interfaces = mod.implementation.outputs+mod.implementation.inputs
            for con in interfaces:
                if con.name == parts[1]:
                    linkparts = con.link.split('/')
                    linkparts[0] = mod.id + '_' + linkparts[0]
                    connectionString += linkparts[0] + '/' + "/".join(linkparts[1:])
                    break
            break
    return path + '/' + connectionString

def lookupInternal(string, mod):
    parts = string.split('/')
    if len(parts) < 2:
        raise Exception("Incorrect connect-statement")
    parts[-len(parts)] = mod.id + "_" + parts[-len(parts)]
    return "/".join(parts)