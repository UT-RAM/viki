""" Provide tools to interpret configuration files and return abstractions of it."""

import xml.dom.minidom
from objects import *
from helpers import *


def getConfig(configfilename='configuration.xml', config_id_to_use=None):
    """Read in the configuration file.

    If there are multiple configurations inside the file, one specific
    configuration is used, depending on the variables

    Keyword arguments:
    configfilename -- the xml-file containing configuration(s) (default configuration.xml)
    config_id_to_use -- use this specific configuration from within the configuration file (default None)
    """

    if not configfilename:
        configfilename = 'configuration.xml'
    print 'Parsing ' + configfilename + ', looking for configurations...'
    dom = xml.dom.minidom.parse(configfilename)
    configurations = dom.getElementsByTagName('configurations')[0]

    domconfig = None
    if not configurations:
        print "No configurations found"
    else:
        configelements = configurations.getElementsByTagName('configuration')
        for configuration in configelements:
            this_id = getOptionalAttribute(configuration, 'id')
            if this_id == config_id_to_use:
                domconfig = configuration
                print 'Found the desired configuration in "' + configfilename +'"'
                print 'Now interpreting configuration...'
                break
            else:
                pass

        if not domconfig:
            raise Exception("Could not found the desired configuration")

        config = Configuration(config_id_to_use)
        recursiveGet(domconfig, config)
        print 'Configuration interpreted succesfully.'
        return config


def recursiveGet(domparent, parent):
    """Get information from a dom element recursively and place in an abstraction.

    Uses objects from Objects.py to create an abstraction.

    Keyword arguments:
    domparent -- the (current) dom parent
    parent -- the parent to which abstractions of the domparent are to be added
    """
    elementsInParent = getElementsOnFirstLevelExceptTag(domparent, 'namespace')
    if elementsInParent:
        for dommod in elementsInParent:
            tagtype = dommod.tagName.lower()
            if tagtype == 'connect':
                connection = Connection_to_add(
                    dommod.attributes['publisher'].value,
                    dommod.attributes['listener'].value
                )
                parent.connections_to_add.append(connection)
            else:
                mod = Module_to_add(
                    dommod.tagName.lower(),
                    dommod.attributes['type'].value,
                    dommod.attributes['id'].value,
                    getOptionalAttribute(dommod, 'supress_warning')
                )

                params = getElementsOnFirstLevel(dommod, 'param')
                if params:
                    for domparam in params:
                        param = Parameter_to_add(
                            domparam.attributes['name'].value,
                            domparam.attributes['value'].value
                        )
                        mod.parameters_to_add.append(param)

                parent.modules_to_add.append(mod)
    namespacesInParent = getElementsOnFirstLevel(domparent, 'namespace')
    if namespacesInParent:
        for domNamespace in namespacesInParent:
            ns = Namespace(domNamespace.attributes['id'].value)
            recursiveGet(domNamespace, ns)
            parent.namespaces.append(ns)
