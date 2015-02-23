ram_ba_package
=======================

This is the documentation for the ROS package *ram*, consisting of both technical documentation of the available executables and some tutorials to get started.

> - All code within this project is intended for reuse and therefore documented. If things are unclear, please contact me (ceesietopc).
- It is assumed in all directions given in this documentation that the latest version of the source code is placed within a Catkin workspace and built using catkin_make. "source devel/setup.bash" should be executed after catkin_make by hand or automatically by placing it in .bashrc to make sure the executables can be found.
- roscore should be running. 

Package goal and functionality
-------------------------
This package was developed during a bachelor assignment at the University of Twente. The goal of the assignment was to use a set of Parrot AR.Drone 2.0s for the pick-up (using a hook and ring) and movement of a mass. To perform this task in an indoor scenario, this package implements a position controller, based on state feedback from a [NaturalPoint OptiTrack][1] sytem. Besides position control, tele-operation through the keyboard and a joystick were also implemented.

The implemented features are listed below.

 1. Automated connection to multiple AR.Drones through a dedicated Wifi network (including GUI). See [Connector](connector.md) page.
 2. Position control of multiple quadcopters using PD control (including GUI). See [Controller](controller.md) page.
 3. Joystick teleoperation. See [Joystick](joystick.md) page.
 4. Keyboard teleoperation. See [Keyboard](keyboard.md) page.

Since documentation on the features is quite specific, it is recommended to take a look at the tutorials to get started.

Folder structure
-----------------
The package consists of multiple folders. The root folder only contains files necessary for the correct detection and compiling of this package through Catkin.

doc
:   Folder containing all documentation of this package.

launch
:   roslaunch files to be used with this package. Some of these files can be used stand-alone, some are intented for automated use with the developed Python applications. This is always mentioned in the header of the launch file. A nice starting point is `single.launch`

matlab
:   Matlab script files for setting setpoints automatically through the [Matlab ROS I/O package][2]

msg
:   Custom ROS messages used by the package.

py
:   Folder containing the Python nodes and applications used within this project. Glade files in this folder are for the definition of the GUI. `drones.json` stores the drones for import and export.

src
:   Folder containing the C++ source of the nodes used within this project.

Tutorials and guides
--------------
[Environment setup](environment.md)
This tutorial guides you through the setup and configuration of the OptiTrack system, the dedicated Wifi network and the AR.Drones.

[Connecting](connecting.md)
This tutorial describes how to use the GUI of the connector to connect to multiple drones.

Matlab
--------------
To use the controller with Matlab, connect to the drones and use the controllers to make sure they are hovering somewhere in the air. Then, run your Matlab script sending Pose-type messages on the setpoints topics you want to address (for a drone with prefix John, that will be /John/setpoint).

Other quadcopters
--------------------
This package is tested with the ardrone_autonomy driver to gain control over a Parrot AR.Drone. The output of the controller however is a normalized Twist message on the /cmd_vel topic. All robots using a similar interface should be able to use this package. Of course, this could require retuning the controller. There are two things however, one should be careful with:

1. The controller interface starts sending commands as soon as you start the interface. In the case of the Parrot AR.Drone, these commands will just be neglected. If this is not the case for your platform, the code should be changed.
2. The references in the Twist messages are attitude references for linear x and y, and a speed reference for linear z (as described in the ardrone_autonomy documentation). So, for other quadcopters that need a certain throttle to stay levelled, the controller in Z-direction will not work.

> To change the default controller tuning, make sure to change both the launch file *controller_module.launch* and the Glade file containing the default position of the sliders.

Other state feedback systems
--------------------
There are alternatives for OptiTrack that can be used as well. Just make sure the current position of the quadcopter is published in a Pose type message in the corresponding /unfiltered_pose topic. In theory it is possible to get this input from a GPS receiver, **but** this requires modifications to the state_estimation executable: GPS signal has to be properly filtered before it is accurate enough to be used with this setup.

Dependencies
---------------
This package uses multiple other non-standard ROS packages. They are shortly mentioned in the following list. Some of these packages are not explicitly updated for ROS Hydro and therefore cannot be installed using apt-get. Clone these repositories and build them manually.

 - [ardrone_autonomy][3] ROS driver for the Parrot AR.Drone. Since multiple quadcopter are to be used, [a version modified by kbogert][4] is used, following [this discussion][5].
 - [mocap_optitrack][6] package to listen for and interpret messages from the OptiTrack system.
 - [yocs_cmd_vel_mux][7] package to switch automatically between multiple *cmd_vel* topics.

  [1]: http://www.naturalpoint.com/optitrack/
  [2]: http://www.mathworks.nl/hardware-support/robot-operating-system.html
  [3]: https://github.com/AutonomyLab/ardrone_autonomy
  [4]: https://github.com/AutonomyLab/ardrone_autonomy/pull/98
  [5]: https://github.com/AutonomyLab/ardrone_autonomy/issues/56
  [6]: http://wiki.ros.org/mocap_optitrack
  [7]: http://wiki.ros.org/yocs_cmd_vel_mux