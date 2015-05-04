.. _userguide:

User guide
==========
This userguide aims to get you started using the Aeroworks framework as quick as possible. If you are here for the first time, you should check if you meet the requirements in :ref:`system setup`. If you are not try using the :ref:`start guide` for a quick setup or create your own setup with `How to write config`_

.. _`system-setup`:

System setup
-------------

For using the framework
^^^^^^^^^^^^^^^^^^^^^^^

To use the Aeroworks framework you are going to need a few things. Let's start at the very basis.

Ubuntu operating system
"""""""""""""""""""""""
Either use a computer that has Ubuntu installed at the RAM lab or install Ubuntu on your own computer from `the Ubuntu download page`_.
Strictly speaking, Ubuntu is not necessary for running the framework: any platform supported by ROS and Python will do. Using Ubuntu is recommended since it is widely used within the RAM and ROS communities. By working on Ubuntu you enable yourself to use the many different kinds of software that have been developed for the platform, some of which we'll use.

.. _`the ubuntu download page`: http://www.ubuntu.com/download/desktop

If you have in no way acces to a computer with Ubuntu, or cannot install it on a computer, you can use a virtual machine to run Ubuntu on your own computer. You could use `Oracle VM Virtualbox`_ to set it up.

.. _`Oracle VM Virtualbox`: https://www.virtualbox.org/

ROS
"""
Ros.org offers `an excellent guide for installing ros`_ on your Ubuntu system, follow it to install ROS.

.. _`an excellent guide for installing ros`: http://wiki.ros.org/indigo/Installation/Ubuntu

Python
""""""

You need Python, get it at the `Python webpage`_. If you need a little more help on this, check out `a Python wiki for beginners`_.

.. _`Python webpage`: https://www.python.org/downloads/source/
.. _`a Python wiki for beginners`: https://wiki.python.org/moin/BeginnersGuide/

The code
""""""""

The framework is in an Mercurial repository hosted at RAM. To get it you need Mercurial which is often installed by default in Ubuntu. Open a console and type

.. code-block:: bash

    hg version

to see if you have mercurial. If you do not there is `an Ubuntu help page on getting mercurial`_.

The easiest way to set up the AeroWorks framework is by putting it in your catkin workspace. If you haven't already, set up your catkin workspace using `this tutorial`_. Open a console and use the 'cd' command to get to your catkin workspace, and then into the 'src' folder. If you have created the catkin workspace in you home folder, you can use the following code to clone the repository.

.. code-block:: bash

    cd ~/catkin_ws/src/ && hg clone https://hg.ce.utwente.nl/aeroworks/

You will need to enter your RAM username and password, after which you will have the latest version of the AeroWorks framework available. Go to the `Quick start guide`_ to see what you can do with it!

.. _`an ubuntu help page on getting mercurial`: https://help.ubuntu.com/community/Mercurial
.. _`this tutorial`: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Some more useful links
""""""""""""""""""""""

* `A simple guide to mercurial and version control in general`_

.. _`A simple guide to mercurial and version control in general`: http://hginit.com/

Help
""""

If you need help, contact :doc:`../developers`.

For development
^^^^^^^^^^^^^^^

If you want to contribute to the AeroWorks project, be sure to check out `How to contribute`_.

You can get Sphinx through apt-get, look for info on `the Sphinx documentation website`_.

.. _`the Sphinx documentation website`: http://sphinx-doc.org/latest/install.html


Quick start guide
-----------------
.. It might be a good idea to create a turtlesim thingy here as well. because the AR.drone setup requires quite some work.

Assuming you have set everything up (for instance through :ref:`System set up`) you can now run your first experiment using the AeroWorks framework.

Quickstarting a turtle
^^^^^^^^^^^^^^^^^^^^^^
ROS includes a turtle simulator package specifically to get you up to speed quickly. Here we will set it up.

Preparing ROS
""""""""""""""
If you haven't already build your catkin workspace using *catkin_make* in your catkin workspace. Do not forget to source setup.bash [#]_ afterwards. If you have installed your catkin workspace in the default directory you can do this by running

.. code-block:: bash

    cd ~/catkin_ws/ && catkin_make && source devel/setup.bash

Run the Aeroworks core
""""""""""""""""""""""

Open up a console and go to your AeroWorks root directory. If you installed it at the default you can do this by running

.. code-block:: bash

    cd ~/catkin_ws/src/aeroworks

Now start the AeroWorks core, use *configuration.xml* and *quickstart_turtle* as parameters. You can do that by running

.. code-block:: bash

    python core configuration.xml quickstart_turtle

You should get some feedback from the core, telling you it has finished. Now run the following to start your turtle

.. code-block:: bash

    roslaunch aeroworks.launch

You should see a small screen with a turtle in it. You can control the turtle hitting arrow keys into the console you used to launch the *aeroworks.launch* file.

What have you done?
"""""""""""""""""""
Let's go step by step through what you have done to create some understanding. We will only consider the work using the AeroWorks core. You can read more on catkin on the `ros wiki on catkin`_.

.. _`ros wiki on catkin`: http://wiki.ros.org/catkin

You have used python to start the core, and provided it two arguments. The first is *configuration.xml*. You can open it, for instance using

.. code-block:: bash

    gedit ~/catkin_ws/src/aeroworks/configuration.xml

to see what is in there. You might see a lot of difficult stuff, but somewhere you should be able to find the following lines:

.. code-block:: xml

    <configuration id="quickstart_turtle">
        <vehicle type="turtlesim" id="turtle1" />
        <userinput type="turtlekey" id="turtlekeynode" />
        <connect publisher="turtlekeynode/cmd_vel" listener="turtle1/cmd_vel" />
    </configuration>

The configuration.xml file can contain one or many configurations. A configuration is a set up for one run of for instance an experiment or simulation. The configuration we are going to use is called *quickstart_turtle*, as specified by the second argument you used when starting the core.

Configurations can grow really complicated, but this one specifically shows only three more things:

* **A vehicle** which is the turtle you see on the screen
* **A userinput** which is software to generate velocity commands from keyboard input.
* **A connect** Which specifies that the velocity commands generated by the userinput should be used as input to the vehicle.

The vehicle and userinput you have specified are both called modules. Information on the modules is stored in *module.xml* files that you can find in the *modules* folder of the AeroWorks root. Normally :doc:`../developers` will provide you with modules to use. If you are missing a module or functionality and are ready to contribute check out `How to contribute`_.

Quickstarting an AR.Drone
^^^^^^^^^^^^^^^^^^^^^^^^^

Allright, you are going to fly a Parrot AR.drone, and going to control it with a joystick. Do not worry, we'll talk you through it.

Additional requirements
""""""""""""""""""""""""
You are going to need a joystick. Find one from the lab (preferably one that nobody is using at the moment) and plug it into your pc. You need to find where it publishes to. Open a console and cd to */dev/input/* and see what's there using ls. You can do that by running the following line

.. code-block:: bash

    cd /dev/input/ && ls

You should see some items like *mouse1* and *js0*, maybe with different numbers and maybe more then just one. You need to find which one belongs to your joystick. To do that you can use the tool *jstest*. Key in jstest and use one of the *js*-items as argument (as shown below). You can cancel the test by hitting *ctrl+c*.

.. code-block:: bash

    jstest js0

If you don't have *jstest* yet you can get it by running: 

.. code-block:: bash

    sudo apt-get install jstest-gtk

You are going to have to do this for every *js*-item (with the number behind it) until you find one that clearly responds when you press buttons or move axis on your joystick, before hitting *ctrl+c*. When you have found it remember which one it was. Fur this quick start guide we will assume that it is *js0*. If you are having trouble finding your joystick through a virtual machine, check out :ref:`virtualbox`.

Go to the root of the AeroWorks framework, and open configuration.xml, for instance by running

.. code-block:: bash

    gedit ~/catkin_ws/src/aeroworks/configuration.xml

Find the part that says (or with another number behind js)

.. code-block:: xml

    <param name="dev" value="/dev/input/js0" />

and replace *js0* with whatever you found before. We were going to assume it was *js0*, so in that case leave it be. Be sure to save the file if you have changed anything.

To interpret the joystick information to command velocities that make sense we use a some special ros packages. We'll talk you through them but for now just make sure you have the joy package. It might be installed for you, to check this run:

.. code-block:: bash

    rospack find joy

If this returns a path rather than an error, you have got the package and can skip the following. If not, get it by running

.. code-block:: bash

    sudo apt-get install ros-indigo-joy

You might need to enter the password to an administrator privileged user account. Furthermore, you should replace *indigo* by the ROS distribution that you have got. You can find whichever distribution you have got by running

.. code-block:: bash

    rosversion -d

Furthermore you need the ardrone_autonomy package, it's a great ROS package that let's you control the parrot. You can check if you have it using

.. code-block:: bash

    rospack find ardrone_autonomy

If you don't have it you can install it by running (with indigo replaced by your ros distribution)

.. code-block:: bash

    apt-get install ros-indigo-ardrone-autonomy


Catkin and source
"""""""""""""""""
Because you might have added new packages, you need to rebuild your catkin workspace and source the setup file so that ROS can find the new packages. You can do this by running

.. code-block:: bash

    cd ~/catkin_ws/ && catkin_make && source devel/setup.bash

Connecting to the drone
""""""""""""""""""""""""
Go to a place where it's safe to fly, for instance the RAM flightarena. Switch on the Parrot AR.drone. Connect to the wireless network the parrot has created, it's called ardrone\_*number*. Don't mind the number. You can acces a list of wireless networks usually at the top-right of your screen, using the arrow symbols.

Once you are connected to the drone you can test the connection by opening a console and running

.. code-block:: bash

    ping 192.168.1.1

If the connection was succesful you should see several ping statistics appearing on screen. Otherwise you'll get a message saying it is unreachable. If you are having trouble connecting to the drone through a virtual machine check out :ref:`virtualbox`.

Starting the framework
""""""""""""""""""""""
You are now ready to actually start the framework. Things are going to be fast now, keep in mind that the button to land is (if the joystick is labeled) labeled 11.
w
Open a console and *cd* to the AeroWorks root directory you have cloned in the :ref:`system-setup`. Then start the core by running Python core with *configuration.xml* and *quickstart_parrot* as arguments. If you have set up the Aeroworks framework in the *src* folder of *~/catkin_ws* you can run the following line to start the core:

.. code-block:: bash

    cd  ~/catkin_ws/src/aeroworks/ && python core configuration.xml quickstart_parrot

You will get some feedback from the framework, eventually telling you it has finished. You should now have a file called aeroworks.launch in the root of the aeroworks folder (~/catkin_ws/src/aeroworks/aeroworks.launch by default). You can use this launch file to start every ROS node you are going to need for this experiment and also set up connections among them.

*cd* to the Aeroworks root and run *roslaunch aeroworks.launch*. If the framework is located in the default location you can do that by running

.. code-block:: bash

    cd ~/catkin_ws/src/aeroworks && roslaunch aeroworks.launch

Go fly
""""""

Hit the button labeled 12 to lift of, move the joystick around to play with your drone. Remember 11 is land.
If your joystick is not labelled get yourself a labeled one, or be careful.


How to write config
-------------------

How to contribute
-----------------

Will be more available later. For now please contact :doc:`../developers`.

Writing code
^^^^^^^^^^^^

Sending in modules
^^^^^^^^^^^^^^^^^^


.. rubric:: Footnotes

.. [#] Sourcing *setup.bash* is required every time ROS packages are added. It might be useful to add the *source* command to your *.bashrc* file, as described in `ROS Question 200174`_.
.. _`ROS Question 200174`: http://answers.ros.org/question/200174/how-to-aviod-running-the-command-source-develsetupbash/


* :ref:`search`

