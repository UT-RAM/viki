import xml.etree.cElementTree as ET
from objects import *
from helpers import *

def write(configuration, filename="aeroworks.launch"):
    # Create root element <launch>
    root = ET.Element("launch")
    for mod in configuration.modules_to_add:
        for executable in mod.implementation.executables:
            node = ET.SubElement(root, "node", pkg=executable.pkg, name=executable.id, type=executable.executable)

    # Make this recursive?
    for ns in configuration.namespaces:
        ns_elem = ET.SubElement(root,"namespace", id=ns.id)
        for mod in ns.modules_to_add:
            for executable in mod.implementation.executables:
                node = ET.SubElement(ns_elem, "node", pkg=executable.pkg, name=executable.id, type=executable.executable)
    tree = ET.ElementTree(root)
    tree.write(filename)
