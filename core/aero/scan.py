import xml.dom.minidom
import os
import re
from objects import *
from helpers import *


def getAvailableModules():
    print "Scanning for modules in file tree..."
    available_mods = []

    # START FILE LOOP
    rootDir = '../'
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fName in fileList:
            if fName == 'module.xml':
                fPath = dirName + '/' + fName
                f = open(fPath)
                fLine = f.readlines()[0]
                if re.search('AEROWORKS', fLine) is not None:
                    # Get DOM
                    dom = xml.dom.minidom.parse(fPath)
                    moddom = dom.getElementsByTagName('module')[0]
                    mod = Module(moddom.attributes['type'].value, moddom.attributes['id'].value)

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
                                interface = Interface(oType, oName, oMessageType, oRequired)
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
                                interface = Interface(oType, oName, oMessageType, oRequired)
                                executableObject.addOutput(interface)

                        # PARAMS
                        ParameterElement = getElementsOnFirstLevel(executable, 'params')
                        if ParameterElement:
                            Parameters = getElements(ParameterElement[0])
                            for aParameter in Parameters:
                                aName = aParameter.attributes['name'].value
                                aType = aParameter.attributes['type'].value
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
                    print mod.id, ' added!'

    # END FILE LOOP
    print "Got all available modules."
    return available_mods
