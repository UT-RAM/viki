import xml.dom.minidom
from objects import *
from helpers import *


def getConfig(configfilename='configuration.xml', config_id_to_use=None):
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
                print 'Now intrepreting configuration...'
                break
            else:
                pass

        if not domconfig:
            raise Exception("Could not found the desired configuration")

        config = Configuration(config_id_to_use)
        domNamespaces = getElementsOnFirstLevel(domconfig, 'namespace')
        if domNamespaces:
            for domNamespace in domNamespaces:
                elementsInNamespace = getElements(domNamespace)
                if elementsInNamespace:
                    ns = Namespace(domNamespace.attributes['id'].value)
                    for dommod in elementsInNamespace:
                        tagtype = dommod.tagName.lower()
                        if tagtype == 'connect':
                            interface = Interface_to_add(
                                dommod.attributes['publisher'].value,
                                dommod.attributes['listener'].value
                                )
                            ns.interfaces_to_add.append(interface)
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
                                        domparam.attributes['name'],
                                        domparam.attributes['value']
                                        )
                                    mod.parameters_to_add.append(param)

                            ns.modules_to_add.append(mod)
                    config.namespaces.append(ns)

        elements = getElements(domconfig)
        for element in elements:
            tagname = element.tagName
            if tagname == 'namespace':
                pass
            elif tagname == 'connect':
                interface = Interface_to_add(
                    element.attributes['publisher'].value,
                    element.attributes['listener'].value
                    )
                config.interfaces_to_add.append(interface)
            else:
                mod = Module_to_add(
                    tagname,
                    element.attributes['type'].value,
                    element.attributes['id'].value,
                    getOptionalAttribute(element, 'supress_warning')
                    )

                params = getElementsOnFirstLevel(element, 'param')
                if params:
                    for domparam in params:
                        param = Parameter_to_add(
                            domparam.attributes['name'],
                            domparam.attributes['value']
                            )
                        mod.parameters_to_add.append(param)

                config.modules_to_add.append(mod)

    print 'Configuration interpreted succesfully.'
    return config
