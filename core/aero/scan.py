"""Provides :func:`getAvailableModules`.
"""
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
import os
import re
import traceback
from objects import *
from helpers import *


def getAvailableModules():
    """Return a list of modules available in the framework at this moment.

    #. Run through the entire project tree searching for files called 'module.xml'.
    #. Create an abstraction from each .xml file as objects from :mod:`core.aero.objects`.
    #. Put abstraction in a list
    #. Return list when done.
    """
    available_mods = []

    # START FILE LOOP
    # TODO: Being able to configure this directory, making it able to run viki everywhere, not just in its own directory
    rootDir = '../'
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fName in fileList:
            if fName == 'module.xml':
                try:
                    fPath = dirName + '/' + fName
                    f = open(fPath)
                    fLine = f.readlines()[0]
                    if re.search('AEROWORKS', fLine) is not None:
                        # Get DOM
                        dom = xml.dom.minidom.parse(fPath)
                        moddom = dom.getElementsByTagName('module')[0]

                        # Skip if the module already exists
                        modname = moddom.attributes['id'].value
                        existingMod = findModuleById(available_mods, modname)
                        if existingMod is not None:
                            print "Module ", modname, " (in ", fPath, ") already exists (in ", mod.path, ") and is therefore skipped."
                            continue

                        mod = Module(moddom.attributes['type'].value, modname)

                        # META DATA
                        meta = dom.getElementsByTagName('meta')
                        if not meta:
                            print "No meta data present"
                        else:
                            # We can have multiple META sections
                            for metaelem in meta:
                                # Check if there are childnodes
                                if len(getElements(metaelem)) > 0:
                                    for metachild in getElements(metaelem):
                                        mod.addMeta(metachild.tagName.lower(), metachild.firstChild.nodeValue)
                                else:
                                    print "Empty meta data section in document"

                        # DEPENDENCIES
                        dependencies = dom.getElementsByTagName('dependencies')
                        if len(dependencies) == 1:
                            for depchild in getElements(dependencies[0]):
                                if depchild.tagName == "depends":
                                    src = ""
                                    type = "apt-get"
                                    if depchild.hasAttribute('src'):
                                        src = depchild.attributes['src'].value
                                    if depchild.hasAttribute('type'):
                                        type = depchild.attributes['type'].value
                                    mod.addPackageDependency(depchild.firstChild.nodeValue, type, src)

                        # MODULE PATH
                        mod.setPath(fPath)

                        # MODULE INPUTS
                        gInputElement = getElementsOnFirstLevel(moddom, 'inputs')
                        if gInputElement:
                            gInputs = getElements(gInputElement[0])
                            for gInput in gInputs:
                                oType = gInput.attributes['type'].value
                                oName = gInput.attributes['name'].value
                                oLink = gInput.attributes['link'].value
                                oMessageType = gInput.attributes['message_type'].value
                                oRequired = gInput.attributes['required'].value
                                interface = Interface(oType, oName, oMessageType, oRequired, oLink)
                                mod.addInput(interface)

                        # MODULE OUTPUTS
                        gOutputElement = getElementsOnFirstLevel(moddom, 'outputs')
                        if gOutputElement:
                            gOutputs = getElements(gOutputElement[0])
                            for gOutput in gOutputs:
                                oType = gOutput.attributes['type'].value
                                oName = gOutput.attributes['name'].value
                                oLink = gOutput.attributes['link'].value
                                oMessageType = gOutput.attributes['message_type'].value
                                oRequired = gOutput.attributes['required'].value
                                interface = Interface(oType, oName, oMessageType, oRequired, oLink)
                                mod.addOutput(interface)

                        # Instead of looping over userinputs, controllers, etc. separately, go find the executables to add flexibility in the classes
                        executables = dom.getElementsByTagName('executable')
                        for executable in executables:
                            executableId = executable.attributes['id'].value
                            executablePkg = executable.attributes['pkg'].value
                            executableExec = executable.attributes['exec'].value
                            executableObject = Executable(executableId, executablePkg, executableExec)

                            # EXECUTABLE INPUTS
                            gInputElement = getElementsOnFirstLevel(executable, 'inputs')
                            if gInputElement:
                                gInputs = getElements(gInputElement[0])
                                for gInput in gInputs:
                                    oType = gInput.attributes['type'].value
                                    oName = gInput.attributes['name'].value
                                    oMessageType = gInput.attributes['message_type'].value
                                    oRequired = getOptionalAttribute(gInput, 'required')
                                    oNamespace = "base"
                                    if gInput.hasAttribute('namespace'):
                                        oNamespace = gInput.attributes['namespace'].value
                                    interface = Interface(oType, oName, oMessageType, oRequired, namespace=oNamespace)
                                    executableObject.addInput(interface)

                            # EXECUTABLE OUTPUTS
                            gOutputElement = getElementsOnFirstLevel(executable, 'outputs')
                            if gOutputElement:
                                gOutputs = getElements(gOutputElement[0])
                                for gOutput in gOutputs:
                                    oType = gOutput.attributes['type'].value
                                    oName = gOutput.attributes['name'].value
                                    oMessageType = gOutput.attributes['message_type'].value
                                    oRequired = getOptionalAttribute(gOutput, 'required')
                                    oNamespace = "base"
                                    if gOutput.hasAttribute('namespace'):
                                        oNamespace = gOutput.attributes['namespace'].value
                                    interface = Interface(oType, oName, oMessageType, oRequired, namespace=oNamespace)
                                    executableObject.addOutput(interface)

                            # PARAMS
                            ParameterElement = getElementsOnFirstLevel(executable, 'params')
                            if ParameterElement:
                                Parameters = getElements(ParameterElement[0])
                                for aParameter in Parameters:
                                    aName = aParameter.attributes['name'].value
                                    aType = getOptionalAttribute(aParameter, 'type')
                                    if aType not in ['str', 'int', 'double', 'bool']:
                                        print "[WARNING] - Type of parameter {} in {} has no valid type".format(aName, executableId)
                                    aDefault = getOptionalAttribute(aParameter, 'default')
                                    parameter = Parameter(aName, aType, default=aDefault)
                                    executableObject.addParameter(parameter)

                            mod.addExecutable(executableObject)

                        # Internal connections
                        ConfigElements = getElementsOnFirstLevel(moddom, 'configuration')
                        if ConfigElements:
                            for ConfigElement in ConfigElements:
                                connections_to_add = getElementsOnFirstLevel(ConfigElement, 'connect')
                                if connections_to_add:
                                    for connection_to_add in connections_to_add:
                                        internal_interface = Internal_Interface(connection_to_add.attributes['publisher'].value,
                                                                        connection_to_add.attributes['listener'].value)
                                        mod.addIntConnect(internal_interface)

                        available_mods.append(mod)

                except Exception as e:
                    print "Skipped adding '" + fPath + "' because it is a broken file. Error thrown was:"
                    print traceback.format_exc()

    # END FILE LOOP
    return available_mods
