Controller
========
The controller is the main part of this package. It is written in C++, but an interface was added using Python and Glade to ease setting new setpoints (controller.py). In the process this functionality is expanded to also be able to configure the controller parameters. Controllers are to be started from [the connector](connector.md). 
![Interface of the controller][1]
> When referring to axes or directions, the inertial frame is used, **not** the body fixed frame.

Functionality
------------

### Basic quadcopter functionality
The three buttons at the top of the application can be used to send the Take-off, Land and Reset commands to the drone.

### Switch to joystick control
Through the interface it is possible to switch to joystick control. The a *fly_from_joystick* starts with every instance of the application, publishing *Twist* messages to */cmd_vel_joystick*. When this setbox is ticked, those messages are republished onto the */cmd_vel* for this specific drone topic.
> It is possible to enable joystick control from the joystick as well, by pushing button 3 on the input device. Joystick control will only be enabled for the quadcopter whose interface is currently in focus. Be careful though: take-off, land and reset commands are sent to **all** drones.

### Publish hover
When this checkbox is ticked, an all-zero Twist message will be sent to the drone, enabling the on-board hovermode that utilises the camera pointing down.

### Publish "do nothing"
When this checkbox is ticked, an zero-velocity Twist message will be sent to the drone. The on-board hovermode will not be enabled.
> Even while this is not the hovermode, when this command is sent the drone will still counteract velocities by external forces. Probably using the data from the accelerometers.

### Publish setpoint
Only when this checkbox is enabled, the setpoint configured by the upper four sliders will be published to be used by the controller. The actual controller executable is written in C++ and automatically started by this Python application.

### Mass connector
If a number of mass connectors is configured from the connector interface, they can be selected in the dropdown box. When a connecter is selected, the "Prepare pick up" button will set the setpoint to a location with a .3 meter larger X setpoint, equal Y setpoint and .27 meter larger Z setpoint.

The "Pick up" button will increase the current setpoint with 0.05 meter in Z direction, and decrease the X setpoint with .45 meter, resulting in a pick up of the mass.

> Make sure the Publish Setpoint is enabled when using this functionality. Otherwise, only the sliders will move, not the quadcopter.

### Nonlinearity
For experiments performed in this research some non-linear elements were built-in. When the "Set nonlinearity" is pushed, these settings are sent to the controller. I sincerely doubt if these features will ever be helpful in other research.

Since commands given to the drone are reference attitudes, **Attitude offset** is self-explanatory. **Nonlinearity** implements the following behaviour: the value given in the slider, plus one, will be multiplied by the proporitional gain **if** the error is positive. If the error is negative, the value will be subtracted from one, and than multiplied by the proportional gain. Example:

    Proportional gain set to be 0.1
    Slider set to 0.5
    In case of a positive error, the control action is given by: error * 0.1 * (1+0.5).
    In case of a negative error, the control action is given by: error * 0.1 * (1-0.5).
    
### Gains
Through pushing the "Set gains" function, the gains as set using the sliders will be sent to the controller. Most of the labels and values are self-explanatory, two options need extra explanation. The state of the checkboxes for these last two options are only considered after pushing the "Set Gains" button.
> Both of these actions are not necessary for using position control. They were implemented for experiments specific for this research.

I-action
:   The controller implemented is a PD controller. When desired however, an I action can be enabled as well. The only anti-wind up that this action currently has implemented, is a reset on receiving a new setpoint. That means that landing while the I action is powered on is a **bad idea**, this will most likely result in a crash.

Velocity damping
:   To reduce overshoot in general, absolute velocity damping was added. This is implemented as an opposing control effort in the world frame. To get the speed information a C++ node was implemented, named *state_estimation*.

  [1]: http://s17.postimg.org/i87s4u4gf/controller_GUI.png