"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""

"""Provide tools to interpret configuration files and return abstractions of it."""
"""VIKI_VERSION_INFO
VIKI: more than a GUI for ROS 
version number: 0.1
version name: Alice

Copyright (c) 2016 Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, https://github.com/UT-RAM/viki

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Connection between the HTML GUI and Python backend was established using work by David Baird following his tutorial on http://www.aclevername.com/articles/python-webgui/

Thank you for letting us use your great work, David.

This work has been funded by the European Commission's H2020 project AEROWORKS under grant no. 644128
END_VERSION_INFO"""
import xml.dom.minidom
from objects import *
from helpers import *


def getConfig(configfilename='configuration.xml', config_id_to_use=None):
    """Read in the configuration file with path *configfilename*.

    If *config_id_to_use* is set, use a configuration with this specific id.

    :param configfilename: the xml-file containing configuration(s) (default configuration.xml)
    :param config_id_to_use: use this specific configuration from within the configuration file (default None)
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
    """Get information from dom element *domparent* and place in an abstraction called *parent*.

    Recursively increase depth, stepping untill the deepest level of *domparent*. use objects from :mod:`core.aero.objects` to create an abstraction.

    :param domparent: the (current) dom parent
    :param parent: the parent to which abstractions of the domparent are to be added
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
            elif tagtype == 'machine':
                machine = Machine(
                    dommod.attributes['name'].value,
                    dommod.attributes['hostname'].value,
                    dommod.attributes['username'].value,
                    dommod.attributes['password'].value
                )
                parent.machines_to_add.append(machine)
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

                args = getElementsOnFirstLevel(dommod, 'arg')
                if args:
                    for domarg in args:
                        arg = Cmdline_argument(
                            domarg.attributes['exec_id'].value,
                            domarg.attributes['argument'].value
                            )
                        print("found a cmd line argument and added it to module to add")
                        mod.add_cmdline_arg(arg)

                pfs = getElementsOnFirstLevel(dommod, 'launch-prefix')
                if pfs:
                    for dompf in pfs:
                        pf = Launch_prefix(
                            dompf.attributes['exec_id'].value,
                            dompf.attributes['prefix'].value
                            )
                        print("Adding prefix")
                        mod.add_prefix(pf)

                selections = getElementsOnFirstLevel(dommod, 'selected-machine')
                if selections:
                    for domselection in selections:
                        selection = Selected_machine(
                            domselection.attributes['exec_id'].value,
                            domselection.attributes['machine_name'].value
                        )
                        print("Adding machine selection")
                        mod.add_machine_selection(selection)

                parent.modules_to_add.append(mod)
    namespacesInParent = getElementsOnFirstLevel(domparent, 'namespace')
    if namespacesInParent:
        for domNamespace in namespacesInParent:
            ns = Namespace(domNamespace.attributes['id'].value)
            recursiveGet(domNamespace, ns)
            parent.namespaces.append(ns)
