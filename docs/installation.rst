.. _installation:

Installation
============
This installation guide assumes you have installed ROS already. If you have not done this, follow `this guide <http://wiki.ros.org/jade/Installation/Ubuntu>`_ to install it.

If you have already a catkin workspace, VIKI can be installed in that workspace. If not, you have to create one using `this guide <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_. It is also possible to create a new workspace, just for the VIKI modules or even extend your old workspace.

If you want to have the most clean configuration, we suggest creating a new workspace for VIKI. ROS packages that are not defined as a module can not be used anyway in VIKI and setting up a new workspace allows for a clean installation with up-to-date package versions. If you just want some functionality from the modules of VIKI, while not using the GUI, you can also just get a module repository and use ROS packages directly in your base workspace.

VIKI consists of a core and at least one module repository. Right now, there is only one main repository, but this design makes it easy to create your own repository of modules for use easy sharing. This installation will setup the core and the main module repository for you. Adding extra module respositories is quite easy and discussed in `Module repositories`_.

Requirements
------------
VIKI uses ROS for all its main functionality and therefore builds on top of those dependencies. In line with ROS, VIKI only supports Ubuntu as its platform and is tested on LTS versions 12.04 and 14.04. Other Ubuntu (and even Linux) distributions may work just as well, but are not supported.

Installation
------------
Installation is as easy as pulling the repository into your (or a new) catkin_workspace. The repository can be found on https://github.com/UT-RAM/viki/. VIKI is a standalone tool and therefore, by default, does not live in your catkin workspace. We recommend installing it in your home folder, but you're free to install it where you like.

.. code-block:: bash

    cd ~
    git clone https://github.com/UT-RAM/viki

This will create a new folder /home/<user>/viki, where all the VIKI magic lives.

Configuring VIKI
----------------
VIKI has a self-configuring tool, which setups the folder directories for you. This is done by launching the configure application from the terminal.

.. code-block:: bash

    cd <viki_dir>
    ./viki configure

Running this will install some extra dependencies, setup a config.json file for the right directories and setup a .desktop file such that you can launch VIKI from the Unity Dash.

Module repositories
-------------------
To be able to really use VIKI, a module repository needs to be added. VIKI works with module repositories, such that you can
get started quickly by automatically adding a set of packages. Using the command line, you can add the default repository as follows

.. code-block:: bash

    ./viki add-module-repository

This will pull the repository and automatically install all dependencies that might be needed for the modules so they can be launched. Furthermore, VIKI should be able to run catkin_make instantly, but this does not work as well,


ROS package dependencies
------------------------
If you followed the steps above, you should be all set. VIKI tries to handle all its ROS package dependencies by using the rosdep tool. However, since this is not optimized for different ROS versions, this may sometimes not work that well. To get an overview of what your missing packages are, run

.. code-block:: bash

    ./viki check-packages

based on this, you might find out you're missing the *libuvc_camera* package when using ROS jade. Since this package is not available for jade, it may not automatically install. To install this by hand, you can run

.. code-block:: bash

    sudo apt-get install ros-indigo-libuvc-camera

as the package is available for ROS indigo. It should not be that hard to figure this out for your packages and we are working to find a way to neatly resolve this. In the meantime, if you're still experiencing any problems, please do not hesitate to contact :ref:`developers`