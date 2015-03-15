Connecting to multiple drones
==========================
This is a step-by-step guide to connecting to multiple drones through the [connector GUI](connector.md) supplied by this package. 
> - Make sure you have the correct version of ardrone_autonomy installed, as described in the [package list](root.md).
> - Make sure roscore is running and all steps from the [environment configuration](environment.md) are performed.
> - Make sure the [connector](connector.md) is configured correctly. 

1. Run the interface by running `rosrun ram interface.py`
2. If you have ever used the interface before, click "Load" to import your drones. 
3. Click "Scan" to scan for drones. For unknown drones, you have to enter a trackable ID and a Prefix (string not containing spaces, special characters etc.). After identifying the drone, it will show up in the list, with an indication that it is available "ad-hoc" (so: using the drone's access point).
4. Click "Assign" to make the application connect to the drones and change their configurations to use a static IP with a dedicated WiFi network.
5. If applicable: set the number of mass-connectors.
6. Double click a drone in the list to start its [controller](controller.md).

> In general, it is a good idea to keep the terminal window that started the interface open and within view. This clarifies why some steps take more time than others.