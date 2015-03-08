from aero import helpers

class Interface:
    def __init__(self, interface_type,
                 name, message_type, required, link=None):
        # TODO: Add error handling: what if attribute does not exist
        self.name = name
        self.type = interface_type
        self.message_type = helpers.lookupMessageType(message_type)
        self.required = required
        self.link = link


class Module:
    def __init__(self, type, id):
        self.inputs = []
        self.outputs = []
        self.executables = []
        self.id = id
        self.type = type
        self.meta = {}

    def addMeta(self, key, value):
        self.meta[key] = value

    def addInput(self, interface):
        self.inputs.append(interface)

    def addOutput(self, interface):
        self.outputs.append(interface)

    def addExecutable(self, executable):
        self.executables.append(executable)


class Executable:
    def __init__(self, id, pkg, executable):
        self.inputs = []
        self.outputs = []
        self.params = []
        self.id = id
        self.pkg = pkg
        self.executable = executable

    def addInput(self, interface):
        self.inputs.append(interface)

    def addOutput(self, interface):
        self.outputs.append(interface)

    def addParameter(self, parameter):
        self.params.append(parameter)


class Parameter:
    def __init__(self, name, type, default=None, value=None):
        self.name = name
        self.default = default
        self.type = type
        self.value = value


class Configuration:
    def __init__(self, id):
        self.id = id
        self.modules_to_add = []
        self.connections_to_add = []
        self.namespaces = []


class Namespace:
    def __init__(self, id):
        self.id = id
        self.modules_to_add = []
        self.connections_to_add = []
        self.namespaces = []


class Module_to_add:
    def __init__(self, role, type, id, supress_warning=False):
        self.role = role
        self.id = id
        self.type = type
        self.supress_warning = supress_warning
        self.parameters_to_add = []

        # here we put Module classes used for implementation
        self.implementation = None


class Connection_to_add:
    def __init__(self, publisher, listener):
        self.publisher = publisher
        self.listener = listener


class Parameter_to_add:
    def __init__(self, name, value):
        self.name = name
        self.value = value
