.. _virtualbox:

Difficulties with virtualbox
============================

Setting up ros through your virtualbox might cause a troubles. The Aeroworks framework does not aim to help you solve all of these, but here you might find the solution to some common problems.

Joystick related
----------------
If you are on a Windows host chances are that you have got to specifically *claim* the joystick for your virtual machine. You can do that by going to the virtual machine, click Devices in the menubar, click USB devices, and enable the one port with your joystick. That should be it.

If the menubar is not visible you might need to hit your Host key, which is your right ctrl key by default.

Connecting to a wireless network
--------------------------------
There are two approaches:

Via an extra adapter
^^^^^^^^^^^^^^^^^^^^
Get an external wireless adapter, plug it into your usb port and *claim* it following the procedure above. That should do the trick.

Via your host pc's wireless card
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Close your virtual machine and open vbox. Go to the network settings for the virtual machien you are planning to use. Add a new network adapter of type 'bridged'. Make sure it is enabled using the button right there.

Now connect to the drone with your Host, it will not work via the virtual machine. The virtual machine however is indirectly connected to the drone via the *bridge* you have just created.