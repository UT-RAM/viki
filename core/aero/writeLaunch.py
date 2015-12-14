import xml.etree.cElementTree as ET
import xml.dom.minidom
import os
import time
import random
from objects import *
from helpers import *
import pprint


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


def getName(obj):
    return obj.name

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

    for machine in configPart.machines_to_add:
        machineElement = ET.SubElement(rootElem, 'machine')
        machineElement.set('name', machine.name)
        machineElement.set('address', machine.hostname)
        machineElement.set('user', machine.username)
        machineElement.set('password', machine.password)
        machineElement.set('env-loader', '~/.viki_env')
        # machineElement.set('ros-root', '/opt/ros/')
        # machineElement.set('ros-package-path', '~/catkin_ws/')
        # machineElement.set('default', 'false')

    for mod in configPart.modules_to_add:
        # Loop through executables for specific module
        for ic in mod.implementation.config:
            from_attr = path + '/' + lookupInternal(ic.listener, mod)
            to_attr = path + '/' + lookupInternal(ic.publisher, mod)

            # THIS IS THE NEW REMAP
            relayElement = ET.SubElement(rootElem, 'node')
            relayElement.set('name', '$(anon remap_{})'.format(str(random.random())))
            relayElement.set('pkg', 'topic_tools')
            relayElement.set('type', 'relay')
            relayElement.set('args', to_attr + ' ' + from_attr)
            relayElement.set('ns', 'remaps')

            # THIS IS THE OLD REMAP
            # remap = ET.SubElement(rootElem, "remap")
            # remap.set('to', to_attr)
            # remap.set('from', from_attr)

        added_params = []
        for executable in mod.implementation.executables:
            nodens = ET.SubElement(configElem, "group", ns=mod.id+'_'+executable.id)
            node = ET.SubElement(nodens, "node", pkg=executable.pkg, name=mod.id+'_'+executable.id, type=executable.executable)
            # Check if one of the parameters that are to be set are present in this executable
            # Todo/problem: params are not defined at executable level, but at module level.
            for paramSearch in mod.parameters_to_add[:]:
                paramFound = False
                for paramList in executable.params[:]:
                    if paramSearch.name == paramList.name:
                        param = ET.SubElement(node, "param", name=paramSearch.name, value=paramSearch.value)
                        # Remove found param from both lists
                        mod.parameters_to_add.remove(paramSearch)
                        added_params.append(paramList.name)
                        paramFound = True
                        break
                if not paramFound:
                    print "Error during parameter config: could not find {}".format(paramSearch.name)

            print "Added parameters: {}".format(added_params)

            # At this point, we also have the default parameters we should fill. loop again.
            for paramList in executable.params:
                if not paramList.name in added_params:
                    added_params.append(paramList.name)
                    param = ET.SubElement(node, "param", name=paramList.name, value=paramList.default)

            # find any command line arguments that belong to this executable
            for argSearch in mod.args:
                if argSearch.execid == executable.id:
                    node.attrib['args'] = argSearch.argument

            # find any prefixes for this executable
            for prefixSearch in mod.prefixes:
                if prefixSearch.execid == executable.id:
                    node.attrib['launch-prefix'] = prefixSearch.prefix

            # find any machine selections
            for selectionSearch in mod.machine_selections:
                print selectionSearch.execid
                if selectionSearch.execid == executable.id:
                    node.attrib['machine'] = selectionSearch.machine_name

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
    pprint.pprint(configPart.namespaces)
    pprint.pprint(configPart.modules_to_add)
    pprint.pprint(string)
    pprint.pprint(path)

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
                    exec_id = linkparts[0]

                    linkparts[0] = ''
                    #TODO: invert this, use root as the default topic root, and provide possibility to define namespace in the module.xml
                    if exec_id not in ['usb_cam', 'cmd_vel_merge', 'cmd_vel_lin_invert', 'turtle_teleop_node', 'turtlenode']:
                        connectionString += mod.id + '_' + exec_id + '/'
                    connectionString += mod.id + '_' + exec_id + '/' + "/".join(linkparts[1:])
                    break
            break
    return path + '/' + connectionString

def lookupInternal(string, mod):
    parts = string.split('/')
    if len(parts) < 2:
        raise Exception("Incorrect connect-statement")

    exec_id = parts[0]
    parts[0] = ''
    if exec_id not in ['joystick_node']: #TODO: This is an ugly hack, somehow we should make it possible to use 'root' namespaces
        parts[0] = '{}_{}/'.format(mod.id, exec_id)
    parts[0] += mod.id + "_" + exec_id
    return "/".join(parts)