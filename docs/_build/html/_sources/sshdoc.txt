.. _sshdoc:

Distributed systems (SSH documentation)
=======================================

VIKI allows you to run nodes on several computers, it does not even be installed on the other systems. There are however some network requirements that can be tedious to setup. Below there is one way to set up your distributed system, there are however more possibilities (if you are an advanced user).

Setting up the network
----------------------
Ros works by running a ros master, or roscore, on one system. All other systems connect to it via it's location that is saved in the *ROS_MASTER_URI* environment variable. Other computers on the network are found by using there name. Furthermore all ports need to be open as ROS opens a new connection for every topic on a random port.

First we make sure we can communicate bidirectionally between systems. As an example take a computer named Ash and a computer named Pikachu. Ash will be the main computer where we run VIKI. Pikachu will be the remote computer (with ros, but without VIKI). There can be more computers like Pikachu (with different names!) but in this guide we will describe just one.

Network connection
******************
Make sure your computers are connected to the same network. This can be done by connecting to the same wireless or wired network. In virtualbox this can be achieved by adding an extra 'internal' network adapter.

Finding computer names
**********************
Find the name of your computers. In this case we know that they are Ash and Pikachu. If you are unsure of your computer name you can find it by typing

.. code-block:: bash

      hostname

in a terminal. It will return the name of your computer.

Finding IP adresses
*******************
Now we need to find the IP adresses of both computers. Open a terminal and run

.. code-block:: bash
	
	ifconfig

You will get some feedback on the network connections that you have open. Find the adapter that is connected to the network you are going to use and look for *inet addr:*. Behind it will be a number that looks like *192.168.1.10*. Remember this for both computers.

You might also want to make both systems have a static IP adress so that you do not have to reconfigere after a computer loses network connection (because most of the time IP adresses are redistributed by a DHCP server, and so it might change over time). A google search on *set static IP-adress* will help you with this.

Setting hostnames
*****************
Open a new terminal and run

.. code-block:: bash

	sudo gedit /etc/hosts

The top line of this file says something like

.. code-block:: bash

	127.0.0.1      localhost

For every computer in the distributed setup you are going to use add a line that has the IP-adress of the computer, some space separation, and it's name. In our case the first three lines of the file will look like this

.. code-block:: bash

	127.0.0.1 		localhost
	192.168.1.10    Ash
	192.168.1.20    Pikachu

Testing
*******
You can test if you found the right IP adress and if communication is possible by running

.. code-block:: bash

	ping 192.168.1.20

with *192.168.1.20* replaced by the IP adress of the computer you want to reach. You can test if your changes to */etc/hosts* has worked by running

.. code-block:: bash
	
	ping Pikachu

from Ash, or the other way around. Both should give you information about succesfully sent packages. If not, recheck your network and make sure you have the right IP's.

Setting up SSH
--------------
ROS uses the SSH protocol to run things on other systems. For it we need a username and password on the machine on which we want to run things. These are the username and password you would normally use to log in on the computer.

If it is not yet previously installed you need to install *openssh-client* on the main PC (Ash) and *openssh-server* on the remote. You can do this by running one of the following two lines and accepting the questions

.. code-block:: bash

	sudo apt-get install openssh-client
	sudo apt-get install openssh-server

Testing SSH
***********
Now test if you can make an SSH connection to the other machine. Open a terminal on Ash and run

.. code-block:: bash

	ssh username@Pikachu

where you replace *username* by the username you would normally use to log in on Pikachu. The terminal asks you if you want to connect, reply yes and fill in your password when asked. You should now see username@Pickachu in front of you terminal entry-area in stead of anotherusername@Ash (which is the normal situation). This means you are now 'in' Pikachu. You can exit by typing

.. code-block:: bash

	exit

and you will return to the normal terminal (on Ash).

Adding to known_hosts
*********************
Your computer keeps a list of computers you can connect to called *known_hosts*. To make ROS able to connect to another computer you need to add it to the list. Open a terminal and run

.. code-block:: bash

	ssh-keygen -R [hostname]
	ssh-keygen -R [ip_address]
	ssh-keygen -R [hostname],[ip_address]
	ssh-keyscan -H [hostname],[ip_address] >> ~/.ssh/known_hosts
	ssh-keyscan -H [ip_address] >> ~/.ssh/known_hosts
	ssh-keyscan -H [hostname] >> ~/.ssh/known_hosts

This will add both the username and the ip adress and the combination ot the known_hosts file, so that you can acces it either way.

At this point you should be able to run anything you want on the remote computer by using VIKI. However by default SSH session are not 'visible' on the remote PC. They just sort of run on the background. To for instance open a screen on the remote machine you need to run things in it's X server. VIKI uses an environment loader for this.

Creating the environment file
*****************************
On Pikachu create a file called *.viki_env* in the home folder for instance via

.. code-block:: bash

	gedit ~/.viki_env

and add to it the following:

.. code-block:: bash

	#!/bin/bash
	. /opt/ros/jade/setup.sh
	DISPLAY=:0; export DISPLAY
	exec "$@"

replace *jade* by the ros distribution on Pikachu, then save and exit.

Testing the setup (tutorial)
----------------------------
Everything should now be set up. We can test the setup with a simple configuration of VIKI. For instance:

1. Start VIKI
2. Add a turtle sim node
3. Add a turtle teleop node
4. Connect them
5. Click the 'machin list' icon (top of screen in VIKI)
6. change the localhost to the name of the computer running VIKI (Ash in above example)
7. add a machine, give it a name and fill in the adresses/username and password. In our case: name Pikachu, adress Pikachu
8. Save and close
9. Click the turtlesim node, and in the properties screen (right side) use select screen to make it run on Pikachu rather then default (local)
10. Select the teleop node and add a prefix *xterm -e* to make it open in a new terminal
11. Click run

This setup will open a roscore on the current machine, a turtlesim node on Pikachu, and a teleop node on Ash. Run RQT to see everything works (remember to refresh after the you have sent a few messages) or just run around with your turtle trying to draw a Pikachu.