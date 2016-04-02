.. _installation:

Installation
============
This installation guide assumes you have installed ROS already. If you have not done this, follow `this guide <http://wiki.ros.org/jade/Installation/Ubuntu>`_ to install it.

If you have already a catkin workspace, VIKI can be installed in that workspace. However, we recommend to install VIKI in its own **seperate workspace**. ROS packages that are not defined as a module can not be used anyway in VIKI and setting up a new workspace allows for a clean installation with up-to-date package versions. If you just want some functionality from the modules of VIKI, while not using the GUI, you can also just get a module repository and use ROS packages directly in your base workspace.

VIKI consists of a core and at least one module repository. Right now, there is only one main repository, but this design makes it easy to create your own repository of modules for use easy sharing. This installation will setup the core and the main module repository for you. Adding extra module respositories is quite easy and discussed in `Module repositories`_.

Requirements
------------
VIKI uses ROS for all its main functionality and therefore builds on top of those dependencies. In line with ROS, VIKI only supports Ubuntu as its platform and is tested on 12.04 and 14.04. Other Ubuntu (and even Linux) distributions may work just as well, but are not supported.

Installation
------------
Installation is as easy as pulling the repository into your (or a new) catkin_workspace. Detailed instructions can be found in the repository itself. Go to https://github.com/UT-RAM/VIKI/ to install VIKI easily.

Configuring VIKI
----------------
VIKI can be easily configured using

.. code-block:: bash

    ./viki configure

This will install some extra dependencies. It should also put a symlink in your catkin_workspace directory that is included in your PATH, such that you can directly use VIKI, but this doesn't work yet..

Right now, there are still some issues regarding the configuration of VIKI.

Module repositories
-------------------
To be able to really use VIKI, a module repository needs to be added. VIKI works with module repositories, such that you can
get started quickly by automatically adding a set of packages. Using the command line, you can add the default repository as follows

.. code-block:: bash

    ./viki add-module-repository

This will pull the repository and automatically install all dependencies that might be needed for the modules so they can be launched. Furthermore, VIKI should be able to run catkin_make instantly, but this does not work as well,