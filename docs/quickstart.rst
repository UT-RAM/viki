.. _quickstart:

Quick start guide
=================

This guide assumes you have installed VIKI succesfully. If you have not, check out :ref:`installation`. Using this guide you can run your first setup: controlling a turtle-simulator with your keyboard.

1. Starting VIKI
----------------
First off, you will have to start VIKI. If you installed via the provided installscript you can hit left alt, type VIKI and hit the VIKI-icon. 

If you installed manually you can open a terminal, cd to your installation directory and run

.. code-block:: bash

    python core

You should see a screen open up, with the VIKI logo on the top left of the screen.

2. Finding and using your modules
---------------------------------
We need two modules for this setup: the turtle simulator and a module that interprets input from your keyboard and sends it to the simulator.

Below the VIKI-logo you can search for modules by clicking in the textbox and entering *turtle*. You will see two boxes called:
- turtle_teleop_key
- turtlesim node

You need both of these. Use your mouse to drag these boxes to the centerpart of the screen (we call this the canvas). You use the canvas as a schematic of your setup as you build it.

3. Making a connection
----------------------
You now need to connect the *turtle_teleop_key* to the *turtlesim node*. You can do this by dragging the teleop's output node to the turtle's input node. An arrow will appear that indicates the direction of information. Notice that you can not start dragging at the turtle's input node: you need to follow the direction of information.

4. Build/run
------------
Your setup is now ready. Hit the green *run* button on top of the screen. A new terminal should open that gives some text feedback. This terminal is used to open your complete setup. More importantly, you should also see a window with a turtle in it. If you focus your input on the text window (click somewhere on it) then you can use your arrow keys to control the turtle in the other window.