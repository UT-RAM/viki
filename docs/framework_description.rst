.. _`Framework description`:

Framework description
=====================

The AeroWorks robotics framework is software aiming to enable the quick and easy start up of experiments involving ROS. It can help a student that is new to a experimental setup to quickly start where the previous contributor stopped. For example: if you are designing a position-controlled quadrotor, based on on-board camera and IMU info, you can immediately start a position-controlled drone based on external cameras (OptiTrack), redo/validate the experiments of the researcher who created that setup, and focus on your research.

There are a few parts to the framework:

Core
----
The core of the framework is the actual part doing the work. The core provides functionality and ease but is rather stupid: it knows of no ROS nodes, executables or anything else you actually want to run. To create a useful experience the core needs the following additions.

Documentation
-------------
Documentation consists of technical information and a user guide. This page is also part of the documentation and you have probably already seen :ref:`home`.

Modules
-------
A module is a collection of ROS nodes, with some configuration added so that it provides one simple function. A module is the smallest building block of the framework. You can write your own modules and put it in the dedicated contrib folder so that you and others can use it. Some modules are provided by the maintainers already to get you started quickly.

Configuration
-------------
Configurations are basically experimental set ups. It is a description of a combination of modules and how they interact with each other and with you, the user. For an experiment you are probably going to write your very own configuration files (though simple examples are provided).