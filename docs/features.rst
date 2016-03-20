.. _features:

Features
========

VIKI aims at making software for robotics easier to run and exchange between the community.

Currently supported
-------------------

VIKI supports a wide set of features

- Build a graphical model of your robot software with modules based on ROS
- Save/Open build canvas models
- Add extra commandline options to modules on the canvas
- Extend functionality by easily creating your own modules
- Export your build model to a launch file
- Automatically launch nodes on a different machine with SSH

Of course, VIKI supports (at least the basic) functionality of ROS features.

- Supply parameters for launch, which can be edited directly in the GUI as well
- Dependency management on the modules, based on rosdep, rospack and git repositories
- Many2Many communication between modules, also to be edited easily in the GUI

Future goals
------------

VIKI is continuing to be improved and different features get added constantly. Functionality that has not made it yet to its release includes

- Communication via services
- Communication via topics that are not created by nodes in VIKI
- ROS multimaster support

We're working on bringing these features into VIKI as well! If there's any functionality you would like to see, contact :ref:`developers` and we will (most likely) help you out quickly!