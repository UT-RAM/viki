.. _`modtaglist`:

description of module attributes and tags
-----------------------------------------

AEROWORKS (comment, not a tag)
	comment at the beginning of module.xml file telling VIKI that this is a file she can work with
MODULE
	Top level tag for the module
	attributes:
	- type: the kind of module this is, for instance userinput or sensor
	- id: unique identifier for this module (no modules can have the same name)


META
	Container for metadata
NAME
	Short name for the module
AUTHOR
	Module author/creator
DESCRIPTION
	Explains what the module does
ICON
	Icon used in VIKI's gui when displaying this module


INPUTS
	Container for module-wide inputs (when not inside **executable**-tag)
INPUT
	Description of module-wide input
	attributes:
	- type: (at this stage always ros_topic)
	- name: name for this input
	- link: connect to *thisexecutable/withthisinput/
	- message_type: ros message type
	- required: tag to identify importance, for now unused

OUTPUTS
	Container for module-wide outputs (when not inside **executable**-tag)
OUTPUT
	Description of module-wide output
	attributes:
	- type: (at this stage always ros_topic)
	- name: name for this output
	- link: connect to *thisexecutable/withthisoutput/
	- message_type: ros message type
	- required: tag to identify importance, for now unused


EXECUTABLE
	rosnode to be added
	attributes:
	- id: unique identifier inside this module
	- pkg: package this node can be found in
	- exec: rosnode to be run
INPUTS
	container for topics this executable can subscribe to (when inside **executable** tag)
INPUT
	topic this executable can subscribe to
	attributes:
	- type: (at this stage always ros_topic)
	- name: name for this input
	- message_type: ros message type
	- required: tag to identify importance, for now unused
OUTPUTS
	container for topics this executable can publish to (when inside **executable** tag)
OUTPUT
	topic this executable can publish to
	attributes:
	- type: (at this stage always ros_topic)
	- name: name for this output
	- message_type: ros message type
	- required: tag to identify importance, for now unused


PARAMS
	container for parameters
PARAM
	Parameter for this executable
	attributes:
	- name: name for this parameter (must be unique inside this module)
	- type: datatype ofthis parameter (unused)
	- default: default value of this parameter