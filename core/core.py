import xml.dom.minidom


def lookupMessageType(message_type):
    return message_type


def getElements(node):
    # Empty
    elems = []
    for child in node.childNodes:
        if child.nodeType == child.ELEMENT_NODE:
            elems.append(child)
    return elems


def getElementsOnFirstLevel(parent, element):
    elements = []
    occurences = parent.getElementsByTagName(element)
    for e in occurences:
        if e.parentNode == parent:
            elements.append(e)
    return elements


class Interface:
    def __init__(self, interface_type, name, message_type, required):
        # TODO: Add error handling: what if attribute does not exist
        self.name = name
        self.type = interface_type
        self.message_type = lookupMessageType(message_type)
        self.required = required

    def setLink(self, link):
        self.link = link


class Module:
    inputs = []
    outputs = []
    roles = []

    def __init__(self, id):
        self.id = id

    def setMeta(self, metaDict):
        self.meta = metaDict

    def addInput(self, interface):
        self.inputs.append(interface)

    def addOutput(self, interface):
        self.outputs.append(interface)


class Role:
    executables = []

    def __init__(self, id, type_id):
        self.id = id
        self.type_id = type_id

    def addExecutable(self, executable):
        self.executables.append(executable)


class Executable:
    inputs = []
    outputs = []

    def __init__(self, id, pkg, executable):
        self.id = id
        self.pkg = pkg
        self.executable = executable

available_mods = []

# Todo: Loop through files
# START FILE LOOP
# Get DOM
dom = xml.dom.minidom.parse('../modules/userinput/joystick/module.xml')
moddom = dom.getElementsByTagName('module')[0]
mod = Module(moddom.attributes['id'])
print "Mod id is: ", mod.id.value

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
                print metachild.tagName.title(), " has value: ", metachild.firstChild.nodeValue
                # TODO: put meta in dict and add to object
        else:
            print "Empty meta data section in document"

# INPUTS
gInputElement = getElementsOnFirstLevel(moddom, 'inputs')
if gInputElement:
    gInputs = getElements(gInputElement[0])
    for gInput in gInputs:
        oType = gInput.attributes['type'].value
        oName = gInput.attributes['name'].value
        oLink = gInput.attributes['link'].value
        oMessageType = gInput.attributes['message_type'].value
        oRequired = gInput.attributes['required'].value
        interface = Interface(oType, oName, oMessageType, oRequired)
        interface.setLink(oLink)
        mod.addInput(interface)

# OUTPUTS
gOutputElement = getElementsOnFirstLevel(moddom, 'outputs')
if gOutputElement:
    gOutputs = getElements(gOutputElement[0])
    for gOutput in gOutputs:
        oType = gOutput.attributes['type'].value
        oName = gOutput.attributes['name'].value
        oLink = gOutput.attributes['link'].value
        oMessageType = gOutput.attributes['message_type'].value
        oRequired = gOutput.attributes['required'].value
        interface = Interface(oType, oName, oMessageType, oRequired)
        interface.setLink(oLink)
        mod.addOutput(interface)

# Instead of looping over userinputs, controllers, etc. seperately, go find the executables to add flexibility in the classes
executables = dom.getElementsByTagName('executable')
for executable in executables:
    # Check the parent and save to module
    print executable.parentNode.tagName


available_mods.append(mod)
print available_mods[0].outputs[0].type

# END FILE LOOP
