.. _`howtomod`:

Module writing guide
====================

This guide consists of:

#. :ref:`modintroduction`
#. :ref:`modtutorial`
#. :ref:`modtaglist`

.. _`modintroduction`:

Introduction to modules
-----------------------

VIKI comes with (a currently not so) wide range of modules. To be able to use your own ROS packages within VIKI you need to create a module description for your package. This description is a module.xml file which describes the inputs and outputs and meta data. Besides that, you can combine different ROS packages into one module to make your life easier!

A module file is standard XML and generally looks like this:

.. code-block:: xml

    <!-- AEROWORKS -->
    <module type="sensor" id="camera_source">

        <meta>
            <name>USB camera</name>
            <author>Robin Hoogervorst</author>
            <icon>icon.png</icon>
            <description>
                Wrapper for the USB cam package within ros
            </description>
        </meta>

        <!-- The in- and outputs of the module as a whole. They are linked to specific executables within the module -->
        <outputs>
            <output type="ros_topic" name="image" link="usb_cam/image_raw" message_type="sensor_msgs/Image" required="true" />
        </outputs>

        <executable id="usb_cam" pkg="libuvc_camera" exec="camera_node">
            <params>
                <param name="vendor" type="string" default="null" />
                <param name="product" type="string" default="null" />
                <param name="width" type="string" default="640" />
                <param name="height" type="string" default="480" />
                <param name="frame_rate" type="string" default="15.0" />
            </params>

            <outputs>
                <output type="ros_topic" name="image" message_type="sensor_msgs/Image" required="false" />
            </outputs>
        </executable>
    </module>


General outline
^^^^^^^^^^^^^^^

Every module file starts out with:

.. code-block:: xml

    <!--AEROWORKS-->

This line makes sure the module is found by VIKI during the scan of files. Not implementing this line makes your module invisible to VIKI. The second line starts the module description:

.. code-block:: xml

    <module type="sensor" id="camera_source">

This line specifies the type and id of the module. The type is a string on which VIKI groups its modules. You are free to choose anything you like here, but to be able to ingerate with VIKI better it is recommended to choose one of the following:

* Controller
* Middleware
* Rosdefaults
* Sensor
* Userinput
* Vehicle
* Views   

The id is a string that is unique for every module. **Make sure it is not too generic!** If you make a module called 'camera', the chance is big that such a module already exists and causes conflicts with other modules. Preferably, use a prefix for your specific project modules.

After the opening lines, the important stuff happens. There are 5 types of XML nodes you can add here:

* meta
* inputs
* outputs
* executable
* configuration

Meta
^^^^

As the name suggests, this is the place to add the meta information of your package.  This looks like

.. code-block:: xml

    <meta>
        <name>USB camera</name>
        <author>Robin Hoogervorst</author>
        <icon>icon.png</icon>
        <description>
            Wrapper for the USB cam package within ros
        </description>
    </meta>

These tags basically speak for themselves, but for completeness sake:

* *name:* This is the name of the module as VIKI shows it in the list
* *author:* This is the name of the Author of the module. If you're writing it, it would be you.
* *icon:* The icon that VIKI uses to show it in the list. For more options, see below
* *description:* The description shown in the interface of VIKI

Icon
""""
For specifying the icon, you have three options:

* Specify nothing: The default VIKI icon will be used.
* Specify a filename: VIKI will look for a image with this filename in the same folder as the module.xml file. If you add icon.png here, make sure you add a icon.png file as well.
* Specify a bootstrap icon: If you provide an icon name which starts with 'glyphicon-' (e.g. glyphicon-star), VIKI will look for a bootstrap icon. This is an easy way to quickly add fancy icons. An overview of icons can be found at `bootstrap icons`_ 

.. _`bootstrap icons`: http://getbootstrap.com/components/

Inputs and Outputs
^^^^^^^^^^^^^^^^^^

.. code-block:: xml
    
    <inputs>
        <input type="ros_topic" name="image_view_input" link="image_view/image" message_type="sensor_msgs/Image" required="true" />
    </inputs>

    <outputs>
        <output type="ros_topic" name="image" link="usb_cam/image_raw" message_type="sensor_msgs/Image" required="true" />
        <output type="ros_topic" name="image2" link="usb_cam_2/image_raw" message_type="sensor_msgs/Image" required="true" />
        <output type="ros_topic" name="<name>" link="<executable_id>/<topic_name>" message_type="<ros_type>" required="<boolean>" />
    </outputs>

The inputs and outputs come after the meta information. These specify the *module* in- and outputs, not executable specific ones. As can be seen, these blocks consist of a group XML node and (a set of) XML node(s) for each in/output. Attributes available for each specific in/output:

* *type*: This is the type of input for the module. Currently, only 'ros_topic' is supported. 
* *name*: This is the name of the output, which will be shown in VIKI.
* *link*: This specifies to which ROS executable topic this links. It is of the format '<executable_id>/<topic_name>'. 
* *message_type*: A ROS topic type: (e.g. sensor_msgs/Image, geometry_msgs/PoseStamped, std_msgs/Empty). VIKI makes sure you only can connect topics of the same type. So it's important to specify!
* *required*: Indicates whether the topic is required to be connected. Is currently not used in the interface, but will probably be implemented in the future.

Executables
^^^^^^^^^^^^
An executable in a ROS node specifies a ROS node that is to be executed. 

.. code-block:: xml

    <executable id="usb_cam" pkg="libuvc_camera" exec="camera_node">
        <params>
            <param name="vendor" type="string" default="null" />
            <param name="product" type="string" default="null" />
            <param name="width" type="string" default="640" />
            <param name="height" type="string" default="480" />
            <param name="frame_rate" type="string" default="15.0" />
        </params>

        <outputs>
            <output type="ros_topic" name="image" message_type="sensor_msgs/Image" required="false" />
        </outputs>
    </executable>

The first line has three parameters:

* *id*: This is the id used in the configuration to specify this executable. The module inputs and outputs are linked to executable inputs and outputs using this id.
* *pkg*: The package from which to run the node
* *exec*: The node that is to be run

The pkg and exec parameters correspond to running the node with

.. code-block:: bash

    rosrun <package> <executable>


Configuration
^^^^^^^^^^^^^

TODO: Cees + Alex?

.. _`modbestpractices`
Best practices for writing modules
----------------------------------

Modules can be split up into two categories:

* **VIKI modules**: These are general modules that are part of VIKI. This is the main set that probabably everyone needs. For now, these live in the VIKI modules repository at RaM. These modules should be tested well and assumed to be stable. Bugs in these modules should be reported to the developers. These modules live in the 'modules' folder within aeroworks.
* **User modules**: These modules are created by users, like you. These modules are project specific and should live only in your own repository. These modules should live in the 'contrib' folder within aeroworks. These are used for your own testing and will probably contain bugs until you finished your assignment.

Of course, the bigger the first category, the more useful VIKI will be. We highly encourage user modules to be included in VIKI. By following the guidelines described below, user modules can be submitted to be added to the VIKI modules. 

General
^^^^^^^
When writing a (ROS package for a) VIKI module you **must**:

* Use catkin for building your package
* Specify the right dependencies in the package.xml file (from ROS)
* Specify your name and contact information within the module.xml file

File structure
^^^^^^^^^^^^^^
User modules live in the 

Modularity
^^^^^^^^^^
Since VIKI is really modular, it is encouraged to use multiple ROS packages for your project. These are no hard guidelines, but to be useful they should be followed:

Your module **must**:
* Publish its topics in its own namespace (so no '/' at the start of your topic name). 


Your module **should**
* Have a clear (and preferably short) answer to the question: 'What is the task of this module?'
* Use only one package

Your ROS package/node **should**:
* Not contain lots of nodes
* Live in the same folder as the module.xml file
* Have clear documentation within the code


.. _`modtutorial`:

Tutorial on module.xml writing
------------------------------

.. _`simplemoduletutorial`:

Simple tutorial
^^^^^^^^^^^^^^^
In this tutorial we will create a module. The module in this tutorial will start the ros_rqt_graph (which is already installed in ROS), which is a nice graph of ROS nodes and topics active on your PC.

Now since you are here it means you are probably not one of the maintainers of the framework. Therefore I kindly request that you only place your module in the 'contrib' folder of the framework. Go there and create a folder for youself with a usefull name, for instance *YourNameMSC*. Let us assume that your name is Doutzen Kroes and you name your folder *DoutzenMSC*. If you have installed the framework in the default location then running this should do the job for you:

.. code-block:: bash

    cd ~/catkin_ws/src/aeroworks/contrib && mkdir DoutzenMSC

Inside there create your very first module definition file. You have no choice for name here, it must be named *module.xml*. Open it to start defining your module. This should help you:

.. code-block:: bash

    gedit module.xml

Write, on the first line of the file

.. code-block:: xml

    <!-- AEROWORKS -->

and save the file. Congratulations, you have just defined your first module! Nevertheless you did not do a good job, yet. Try running the aeroworks core (you should know how by now). You should see that there is something wrong with the file you have just created because the terminal tells you something like:

.. code-block:: bash

    Skipped adding '../aeroworks/contrib/DoutzenMSC/module.xml' because it is a broken file. Error thrown was:
    Traceback (most recent call last):
      File "core/aero/scan.py", line 33, in getAvailableModules
        dom = xml.dom.minidom.parse(fPath)
      File "/usr/lib/python2.7/xml/dom/minidom.py", line 1918, in parse
        return expatbuilder.parse(file)
      File "/usr/lib/python2.7/xml/dom/expatbuilder.py", line 924, in parse
        result = builder.parseFile(fp)
      File "/usr/lib/python2.7/xml/dom/expatbuilder.py", line 211, in parseFile
        parser.Parse("", True)
    ExpatError: no element found: line 2, column 0

As you can see that is quite a lot of text. The framework does not crash, and other modules are still available, but you can imagine that this kind of feedback is undesireable. Therefore **only commit modules that are complete**.

Now open your gedit with module.xml again, and we will create something more useful.

Add this to your module file and save:

.. code-block:: xml

    <module type="feedback_to_user" id="rqtmodule">
    
    </module>

To help you and others increase the re-useability of the module please add metadata to all your module files, between the module tags, in this format:

.. code-block:: xml

    <meta>
        <name>rqt module</name>
        <description>Created using a tutorial, this modules starts the ros_rqt_graph</description>
        <author>Doutzen Kroes</author>
    </meta>

Your module is now no longer broken (confirm by running the core if you like) but it has not got any functionality either. You will need to add a reference to an *executable* which is a rosnode for us. Add this below your meta data, between the module tags, and save:

.. code-block:: xml

    <executable id="graphnode" pkg="rqt_graph" exec="rqt_graph">
    </executable>

For your convenience, this is how your module.xml file should look now:

.. code-block:: xml

    <!-- AEROWORKS -->
    <module type="feedback_to_user" id="rqtmodule">
        <meta>
            <name>rqt module</name>
            <description>Created using a tutorial, this modules starts the ros_rqt_graph</description>
            <author>Doutzen Kroes</author>
        </meta>

        <executable id="graphnode" pkg="rqt_graph" exec="rqt_graph">
        </executable>
    </module>

Your module is now finished and runnable. Find out how and confirm as described in :ref:`simpleconfigtutorial` or read on for more on module writing.

Advanced examples
^^^^^^^^^^^^^^^^^


.. _`modtaglist`:

description of module attributes and tags
-----------------------------------------