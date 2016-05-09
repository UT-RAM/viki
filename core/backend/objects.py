"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""

"""Describes the different objects used as abstraction of the AeroWorks configuration and modules."""
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
import helpers


class Interface:
    """An *interface* represents an in- or output of a module. This in- and outputs can be connected to other module's in- and outputs."""
    def __init__(self, interface_type,
                 name, message_type, required, link=None, namespace="base"):
        #: Represents the name of the interface. E.g.: "cmd_vel"
        self.name = name
        #: Represents the type of the interface. E.g.: "ros_topic"
        self.type = interface_type
        #: Namespace used in the ros package for this topic, "base", "global" or "private"; according to wiki.ros.org/Names
        self.namespace = namespace
        #: Represents the message type the interface can handle. E.g.: "twist" or "standard_msgs/Empty"
        self.message_type = helpers.lookupMessageType(message_type)
        #: Defines if the interface is required to be connected. Execution will halt if this requirement is not met.
        self.required = required
        #: Contains the reference to the executable within the module that actually provides the in- or output. E.g.: "test_node/test_topic"
        self.link = link


class Internal_Interface:
    """An *internal_interface* represents a connection between an in- and output **within** a module. At this point, implicitly only implemented for *ros_topic* type interfaces.

    .. note:: In the future, this might be a specific case of :class:`Interface`.
    """
    def __init__(self, publisher, listener):
        #: Refers to the topic on which messages are be published. E.g. "node_name/command_giver"
        self.publisher = publisher
        #: Refers to the topic where the node is listening for messages. This topic is remapped to listen to the publisher. E.g. "node2_name/i_listen_here"
        self.listener = listener


class Module:
    """A *module* represents a separately distributable piece of software, that is able to work within the AeroWorks infrastructure.
    """
    def __init__(self, type, id):
        #: A list of :class:`Interface` describing all inputs of the module.
        self.inputs = []
        #: A list of :class:`Interface` describing all outputs of the module.
        self.outputs = []
        #: A list of :class:`Executable` describing all executables (ROS nodes) in the module.
        self.executables = []
        #: Represents the ID of the module, as defined in the ``viki.xml``. This is used for identifying and selecting the module throughout the process, so this value should be unique.
        self.id = id
        #: Represents the type of the module, as defined in the ``viki.xml``. This categorization is added for the comfort of the user. Examples are "userinput", "sensor", "controller" and "vehicle".
        self.type = type
        #: Contains a dictionary with all module metadata, as defined in ``viki.xml``.
        self.meta = {}
        #: A list of :class:`Internal_Interface` describing all internal connections.
        self.config = []
        #: Contains the path to the module config
        self.path = ''
        #: List with names of dependent ROS packages
        self.package_dependencies = []
        #: List with missing packages, initially empty
        self.missing_packages = []

    def addMeta(self, key, value):
        """Adds metadata to :attr:`meta`.

        :param key: The key of the metadata. E.g. "name"
        :type key: string
        :param value: The value of the metadata. E.g. "AeroWorks"
        :type value: string
        """
        self.meta[key] = value

    def addPackageDependency(self, package_name, type="apt-get", src=""):
        self.package_dependencies.append({
            'name': package_name,
            'type': type,
            'src': src
        })

    def addMissingPackage(self, package_name):
        self.missing_packages.add(package_name)

    def addInput(self, interface):
        """Adds an input to the list of module inputs (:attr:`inputs`).

        :param interface: The interface to add
        :type interface: :class:`Interface`
        """
        self.inputs.append(interface)

    def addOutput(self, interface):
        """Adds an output to the list of module outputs (:attr:`outputs`).

        :param interface: The interface to add
        :type interface: :class:`Interface`
        """
        self.outputs.append(interface)

    def addExecutable(self, executable):
        """Adds an executable to the list of module executables (:attr:`executables`). Those executables are ROS nodes.

        :param executable: The executable to add.
        :type executable: :class:`Executable`
        """
        self.executables.append(executable)

    def addIntConnect(self, internal_interface):
        """Adds an internal connection to the list of module internal connections (:attr:`config`).

        :param internal_interface: The connection to add
        :type internal_interface: :class:`Internal_Interface`
        """
        self.config.append(internal_interface)

    def setPath(self, path):
        """Sets the path for the current module.

        :param path: The path where to find the module
        :type path: string"""
        self.path = path

    def getExecutable(self, exec_id):
        """ Returns the executable with the id, within this module
        :param exec_id: the string id of the executable, as defined in the xml file
        :return: Executable
        """
        for executable in self.executables:
            if executable.id == exec_id:
                return executable
        return None

class Executable:
    """An *executable* represents a ROS node that can be launched through a ROS launch file, generated by the AeroWorks infrastructure."""
    def __init__(self, id, pkg, executable):
        #: A list of :class:`Interface` describing all inputs of the executable.
        self.inputs = []
        #: A list of :class:`Interface` describing all outputs of the executable.
        self.outputs = []
        #: A list of :class:`Parameter` describing parameters to be set for the executable.
        self.params = []
        #: A string with default arguments
        self.args = ""
        #: The ID of the executable, as given in the ``viki.xml``.
        self.id = id
        #: The ROS package that contains the executable. E.g.: "joy"
        self.pkg = pkg
        #: The ROS node executable to launch. E.g. "joy_node".
        self.executable = executable

    def addInput(self, interface):
        """Adds an input to the list of executable inputs (:attr:`inputs`).

        :param interface: The interface to add
        :type interface: :class:`Interface`
        """
        self.inputs.append(interface)

    def addOutput(self, interface):
        """Adds an output to the list of executable outputs (:attr:`outputs`).

        :param interface: The interface to add
        :type interface: :class:`Interface`
        """
        self.outputs.append(interface)

    def addParameter(self, parameter):
        """Adds a parameter to the list of executable parameters (:attr:`params`).

        :param parameter: The parameter to add
        :type parameter: :class:`Parameter`
        """
        self.params.append(parameter)

    def setArguments(self, argument):
        self.args = argument

    def getInterface(self, name):
        for interface in (self.inputs + self.outputs):
            if (interface.name == name):
                return interface
        return None


class Parameter:
    """A *Parameter* represents a param in the rosparam server

    :param name: The name of the parameter
    :param default: A default value, when no specific value is given
    :param type: Type of parameter
    :param value: Value of the parameter
    """
    def __init__(self, name, type, default=None, value=None):
        self.name = name
        self.default = default
        self.type = type
        self.value = value


class Configuration:
    """A *Configuration* is a list of settings,
    usually from a configuration file, used to setup an experiment.

    it contains:
    :param id: unique id for this configuration
    :param modules_to_add: list of modules needed in this *configuration*
    :param connections_to_add: list of connections needed
    :param namespaces: list of :class:Namespace in this *configuration*
    """
    def __init__(self, id):
        self.id = id
        self.modules_to_add = []
        self.connections_to_add = []
        self.machines_to_add = []
        self.namespaces = []


class Namespace:
    """A namespace is in itself basically a :class:Configuration ,
    which can in turn contain multiple namespaces.
    
    :param id: unique id for this configuration
    :param modules_to_add: list of modules needed in this *configuration*
    :param connections_to_add: list of connections needed
    :param namespaces: list of *namespace*s
    """
    def __init__(self, id):
        self.id = id
        self.modules_to_add = []
        self.connections_to_add = []
        self.namespaces = []


class Module_to_add:
    """An object representing a module that is desired to be in a configuration.
    It is a temporary container for all information to get the actual :class:Module classes,
    and contains an *implementation* where the :class:Module can be placed.

    :param role: role of this module eg. userinput, vehicle, etc.
    :param id: unique id of the module
    :param type: type of module
    :param supress_warning: suppress warnings given in/by this module (not yet implemented)
    :param parameters_to_add: list of parameters that are to be added to class:Executable in this module
    :param args: list of command line arguments that are used to start this modules nodes
    """
    def __init__(self, role, type, id, supress_warning=False):
        self.role = role
        self.id = id
        self.type = type
        self.supress_warning = supress_warning
        self.parameters_to_add = []
        self.args = []
        self.prefixes = []
        self.machine_selections = []

        # here we put Module classes used for implementation
        self.implementation = None

    def add_cmdline_arg(self, arg):
        """Add a command line argument object *arg* to the list of arguments needed to run this module

        :param arg: The command line argument object to add
        """
        self.args.append(arg)

    def add_prefix(self, pf):
        """Add a launch-prefix object *pf* to the list of prefixes desired to run

        :param pf: prefix object
        """
        self.prefixes.append(pf)

    def add_machine_selection(self, selection):
        """Add a machine selection object *selection* to the list of machine selections desired to run

        :param selection: machine selection object
        """
        self.machine_selections.append(selection)


class Connection_to_add:
    """Representation of a connection desired to add to a configuration

    :param publisher: topic of the publisher to be connected
    :param subscriber: topic of the subscriber to be connected
    """
    def __init__(self, publisher, listener):
        self.publisher = publisher
        self.listener = listener


class Parameter_to_add:
    """A parameter to add to a module (and finally to an executable)

    :param name: name of the parameter
    :param value: value of the parameter
    """
    def __init__(self, name, value):
        self.name = name
        self.value = value


class Cmdline_argument:
    """An argument to use when starting a specific rosnode.
    Used for instance when starting rviz with a configuration file.

    :param executable_id: id of the executable this argument is used for
    :param argument: string of all arguments to use for this executable
    """
    def __init__(self, executable_id, argument):
        self.execid = executable_id
        self.argument = argument


class Launch_prefix:
    """A launch prefix that will be run before the actual ROSnode.
    Can be very useful to for instance start a node in debug mode.

    :param executable_id: id of the executable this prefix is used for
    :param prefix: string of the prefix
    """
    def __init__(self, executable_id, prefix):
        self.execid = executable_id
        self.prefix = prefix

class Selected_machine:
    """A machine selection defines at what machine the ros_node will run.

    :param executable_id: id of the executable this selection is used for
    :param machine_name: name of the machine
    """
    def __init__(self, executable_id, machine_name):
        self.execid = executable_id
        self.machine_name = machine_name

class Machine:

    def __init__(self, name, hostname, username, password):
        self.name = name
        self.hostname = hostname
        self.username = username
        self.password = password