Teleoperation through keyboard
==============================

The executable 'fly_from_keyboard' listens to keyboard input in the terminal and translates key presses to commands for the drone. Velocity commands are sent to the */cmd_vel* topic using a *Twist* message. For the take-off, land and reset commands the corresponding topics within */ardrone/* are used.

> - Make sure an instance of the *ardrone_driver* from the *ardrone_autonomy* package is running and connected to the drone.

Run the executable by running
```
rosrun ram fly_from_keyboard
```

The following shortcuts can be used now:

| Action | Key |
| ----- | ----- |
| Go left | a |
| Go right | d |
| Go forward | w |
| Go backward | s |
| Go up | k |
| Go down | m |
| Turn right | e |
| Turn left | q |
| Take off | o |
| Land | l |
| Reset | r |
| Exit keyboard control (use after land!) | X (capital x)

Limitations of this executable:

 - Since the terminal can only read one character at a time, direct commands for smooth diagonal motion are not possible.
 - Because of the Auto Key Repeat Delay built-in in the BIOS it takes a while before repeating commands can be read out. The script incorporates a delay to make sure holding in a key results in smooth motion. This delay however can lead to small overshoots.
