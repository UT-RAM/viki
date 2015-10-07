xml-tag description
-------------------

### module
 

| Tag  			| description  										| inside |
|---------------|---------------------------------------------------|--------|
| aeromodule    | a top level tag in which everything on this module fits. The xml file will have nothing outside these tags, and it will have only one of these tags. there are nog add  |
|  id           | unique id/name for this module  |
| meta  		| 2nd level tag that encapsulates informative data, that is not used by the core, but rather when coding stuff and as a referral. it contains owner info and licnese etc.  |
| description   | A description of what the module does |
| source 		| where the original code for this module came from |
| author        | The author of the original code |
| aeromaintainer| The person who added this module to the framework |
| email 		| email adres of maintainer | aeromaintainer |
| license 		| license belonging to original code |
| executable    | executable file belonging to this module |
| type 			| type of executable: exe, rosnode, python, etc. utlitmately tells the core how to execute this executable | executable |
| src 			| location on disk where the executable can be found | executable |
| id 			| id of this executable, unique within the module | executable |
| inputs 		| collection tag for all inputs |
| input 		| a unidirectional interface into the executable |
| type 			| the type of input i.e. file, rostopic, usb etc. | input/output |
| msgtype 		| concerning a rostopic, the type of message | input/output |
| required 		| either true or false, sets if this modules requires this specific input to work, default is false | input/output |
| name 			| the name of the input | input/output |
| description 	| a description if needed, does nothing but explain | input/output |
| outputs 		| a colletion tag for all outputs |

### core-config

| Tag  			| description  										| inside |
|---------------|---------------------------------------------------|--------|
| aeroconfig 	| top level tag containing the complete config |
| meta 			| second level tag containing info, core does not use this | 
| description  	| description of this specific set up | meta |
| author 		| creator of this specific config | meta |
| params 		| collection tag for parameters needed for this config, in general not specific for a module |
| param 		| a parameter |
| name 			| name of the parameter in the param server | param |
| value 		| value of the parameter | param |
| modules 		| collection tag for modules to start | 
| robot 		| collection tag for modules belonging to a singel robot, might be appended with a namespace so all modules start in that namespace |
| namespace 	| the name of the robot, in whose namespace all modules will start | robot|
| module 		| Find and run this module | robot |
| id 			| unique module id | module |
| required 		| this configuration needs this module to run, exit if it doesnt run | module |
| param 		| paramaeter specific for this module | module |
| remap 		| remap topics inside this module | module |
| depend 		| make the module depend on its specific executables | module |