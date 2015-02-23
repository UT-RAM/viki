Teleoperation through joystick
=============================
A generic Linux joystick can be used as input device through the ROS *joy* package. Since multiple types of joysticks are supported (tests were performed with a Logitech joystick and a PS3 controller), always start this executable through a launch file to configure it with the correct parameters for the axes and buttons to be used.
```
roslaunch ram single_from_joystick
```
This launch file starts an instance of the *ardrone_driver*, a *joy* node, the *fly_from_joystick* executable and a nodelet that can listen to multiple *Twist* type topics and decide which signal is most important and should be sent to the drone. In this context the nodelet does nothing more than copying the messages from */cmd_vel_joy* to */cmd_vel*

The parameters that can be given to the *fly_from_joystick* package are usually self-explanatory (like the used buttons and axes). Some can use some extra information.

\*_velocity_max
:   The maximum relative velocity command that will be sent to the drone. 1 for full-range (since velocity commands are normalized), 0 for no command.

slow_button
:   When this button is pushed, all commands are multiplied by 0.2 to get slower motion.

toggle_control_button
:   When this button is pushed, the input from the axes are neglected. Some joysticks give zero command quite randomly, so sometimes it is a good idea to temporarily fully disable the output from the axes.

enable_stick_control_init
:   Describes if the aforementioned control through axes is enabled or disabled when the scripts is started.

