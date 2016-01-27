.. _installation:

Installation
============
You can get VIKI either by using the installscript, or manually. For manual installation you can scroll down to `Manual Installation`_. Either way though, you need the requirements listed below. This installation guide assumes you have installed ROS already. If you have not done this, follow `this guide <http://wiki.ros.org/jade/Installation/Ubuntu>`_ to install it.

If you have already a catkin workspace, VIKI can be installed in that workspace. However, we recommend to install VIKI in its own **seperate workspace**. ROS packages that are not defined as a module can not be used anyway in VIKI and setting up a new workspace allows for a clean installation with up-to-date package versions. If you just want some functionality from the modules of VIKI, while not using the GUI, you can also just get a module repository and use ROS packages directly in your base workspace.

VIKI consists of a core and at least one module repository. Right now, there is only one main repository, but this design makes it easy to create your own repository of modules for use easy sharing. This installation will setup the core and the main module repository for you. Adding extra module respositories is quite easy and discussed in `Module repositories`_.

Requirements
------------
VIKI uses ROS for all its main functionality and therefore builds on top of those dependencies. In line with ROS, VIKI only supports Ubuntu as its platform and is tested on 12.04 and 14.04. Other Ubuntu (and even Linux) distributions might work just as well, but are not supported.

Installation
------------

Installation is done by pulling the VIKI repository inside your Catkin Workspace.

Module repositories
-------------------
.. todo:: Module repositories, explanation, how to add new ones