.. _`Framework description`:

Framework description
=====================



There are a few parts to the framework:

Core
----
The core of the framework is the actual part doing the work. The core provides functionality and a GUI but is rather stupid: it knows of no ROS nodes, executables or anything else you actually want to run. To create a useful experience the core needs the following additions.

Documentation
-------------
Documentation consists of technical information and user guides. This page is also part of the documentation and you have probably already seen :ref:`home`.

Updating documentation
----------------------
Once you have made changes please update the documentation by editing the *.rst* files in *VIKI/doc/* and re-build the documentation by running the following the the VIKI root directory

.. code-block:: bash

    sphinx-apidoc -f -o docs core; cd docs; make html; cd ..

Modules
-------
A module is a collection of ROS packages or nodes, with some configuration added so that it provides one simple function. A module is the smallest building block of the framework. You can write your own modules and put it in a dedicated folder so that you and others can use it. If you are a RAM member the developers provide a list of useful modules in an actively maintained repository at http://www.github.com/UT-RAM/viki-modules .

A more elaborate description (much more technical) can be found in 
.. todo:: add link to overview here