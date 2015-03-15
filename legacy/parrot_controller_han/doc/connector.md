Connector
========
The connector application is an application developed to connect to multiple AR.Drones simultaneously in an automated fashion. Through the interface, [controllers](controller.md) can be started for the connected drones. This Python application (interface.py) does not contain any ROS nodes, but starts and manages executables that do contain nodes.
![Interface of the connector][1]

> The connector only works under Ubuntu, since the built-in NetworkManager is used over the DBus to connect to the drones.

Configuration of the connector
---------
To use the connector, configure it by opening the *interface.py* file and modifying the variables directly after the imports.

SSID
:   The SSID of the dedicated, unprotected WiFi network that the drones have to connect to. Since it is an unprotected network, it is recommended to disable the SSID broadcast.

SSID_SLUG
:   A string the WiFi SSID should contain to be identified as drone network. The value 'ardrone' will work if the SSID of the quadcopters is not modified.

wlan_interface
:   The interface to be used when searching and connecting to the drones. This string (for example 'wlan0') can be found easily by running 'ifconfig'. 

Connector functionality
-------------
### Scan
Scan for drones by pressing the "Scan" button. The application will look in the list of currently detected WiFi networks provided by the NetworkManager and look for SSID's containing the SSID_SLUG to identify drones. When drones are detected that are not in the list yet, a prefix (string not containing spaces or special characters) and the trackable ID from the OptiTrack software have to be given.
> Detecting drones can take very long. This seems to be a bug in the NetworkManager. Disabling and enabling the WiFi (sometimes) helps.

### Import and Export
Since the prefix and trackable ID are usually constant over multiple experiments it is possible to export the current list of drones. Just the SSID, prefix and Trackable ID are exported. Pressing the "Import" button before starting a scan will import this information, so drones are recognized when performing the scan.

### Assign
Pressing "Assign" will make the application loop through the list of Ad-Hoc available drones (that is: drones found while the last scan was performed) and perform the steps described in [this document][2] for all of them, resulting in a unique IP address per drone.

> When you have to enter the IP address by hand (for example, when restarting the app while the drone is still powered on), right-click that drone in the list to define its IP.

### Clear
Pressing "Clear" will clear the list of drones.

### Start a controller
By double clicking a certain drone in the list, a [controller](controller.md) will be started.

### Automatically performed tasks
The application will automatically update the configuration of the mocap_optitrack package and restart the mocap_optitrack node on every change of the list. Besides that, a *joy* node will be launched to listen for joystick input.

Definition of mass connectors
----------------------
In this specific project a semi-automated pick-up of a mass was performed. For that reason, the connectors of the connectors to the mass were to be tracked as well. By changing the number of connectors in the box, the mocap_optitrack package will be reconfigured to also listen for the trackable IDs of these connectors. 

The IDs are assumed to be directly following the highest ID in the quadcopter list. In the GUI shown, the first trackable ID corresponding to a mass connector would be 3, then 4, etc.

Bulk setpoint edit
---------------------
Clicking the "Bulk setpoint edit" button will open a separate Python application that listens for setpoints for all drones, stores them and publishes changes to them in a relative fashion. This tool can be used to send new setpoints to all drones, e.g. 1 meter above their current setpoints.
> Be careful using this tool. As mentioned in its GUI, the tool should first have received the current setpoints of all drones. If this is not the case, the undefined drones will get the same setpoint and crash into each other.


  [1]: http://s29.postimg.org/vc0an7niv/connector_GUI.png
  [2]: https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones