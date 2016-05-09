.. _`package_guidelines`:

Package and module writing guidelines
=====================================

If you want to extend functionality in VIKI, you can make your own module. The module basically describes the ROS package, which you have written.
When developing a ROS package for use in VIKI, it is good to follow some general guidelines to optimize the use and re-use of your package.

Modularity
""""""""""
VIKI aims at creating and maintaining a highly modular environment. This means we highly encourage you to write your modules as modular as possible. It is good to really think beforehand what you want the module to do and keep it as general as possible.

Location
""""""""
If you want to add your own module, you can place this in your own folder in the viki_modules directory. If you have installed VIKI properly, you should have a viki_modules directory inside the src directory in your workspace. In there, there is the core directory that supplies the basic modules for VIKI. You can place a directory next to that, where your own modules and packages will live. You can place the viki.xml for VIKI in the same folder as you create your ROS package with.

::

    +--- catkin_ws/src
    |------ viki/
    |------ viki_modules/
    |       |--- core/
    |       |--- <user>
    |       |    |--- <your_module>
    |       |    |   |--- src/
    |       |    |   |--- package.xml
    |       |    |   |--- viki.xml
    |       |    |--- <your_second_module>
    |       |        |--- src/
    |       |        |--- package.xml
    |       |        |--- viki.xml

Since VIKI does have good dependency management, you can also just put your ROS package in its own seperate repository and define the dependency in the module file.


Checklist
"""""""""
When writing a (ROS package for a) VIKI module you **must**:

* Use catkin to build your package
* Specify the right dependencies in the package.xml file (from ROS)
* Specify your name and contact information within the viki.xml file
* Specify your own package name in the dependency in the viki.xml file

Your module **must**:

* Publish its topics in its base namespace (so no '/' at the start of your topic name). (For more information about namespaces, see http://wiki.ros.org/Names ) If you use python, this is done automatically, but if you're using C++, make sure you have the right nodehandle.

Your module **should**

* Have a clear (and preferably short) answer to the question: 'What is the task of this module?'

Your ROS package/node **should**:

* Live in the same folder as the viki.xml file
* Have clear documentation within the code
