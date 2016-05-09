.. _`modintroduction`:

More advanced modules
=====================

VIKI comes with (a currently not so) wide range of modules. As an engineer, you probably want to use your own functionality within VIKI. To do this, you will have to write a ROS package and enable it for VIKI by creating a *viki.xml* file. This file is a module description for your package which describes the inputs, outputs, executables and meta data. This guide assumes you already have a ROS package (or multiple of them) that you want to use within VIKI. For creating new ROS packages for use in VIKI, it is useful to follow some guidelines, which can be found at :ref:`package_guidelines`.

A module file is standard XML and generally looks like this:

.. code-block:: xml

    <!-- VIKI_MODULE -->
    <module type="sensor" id="camera_source">

        <meta>
            <name>USB camera</name>
            <author>Robin Hoogervorst</author>
            <icon>icon.png</icon>
            <description>
                Wrapper for the USB cam package within ros
            </description>
        </meta>

        <dependencies>
            <depends>libuvc_camera</depends>
        </dependencies>

        <!-- The in- and outputs of the module as a whole. They are linked to specific executables within the module -->
        <outputs>
            <output type="ros_topic" name="image" link="usb_cam/image_raw" message_type="sensor_msgs/Image" required="true" />
        </outputs>

        <executable id="usb_cam" pkg="libuvc_camera" exec="camera_node">
            <params>
                <param name="vendor" type="str" default="null" />
                <param name="product" type="str" default="null" />
                <param name="width" type="str" default="640" />
                <param name="height" type="str" default="480" />
                <param name="frame_rate" type="str" default="15.0" />
            </params>

            <outputs>
                <output type="ros_topic" name="image" message_type="sensor_msgs/Image" required="false" />
            </outputs>
        </executable>
    </module>


General outline
---------------

Every module file starts out with:

.. code-block:: xml

    <!-- VIKI_MODULE -->

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
----

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
* Specify a filename: VIKI will look for a image with this filename in the same folder as the viki.xml file. If you add icon.png here, make sure you add a icon.png file as well.
* Specify a bootstrap icon: If you provide an icon name which starts with 'glyphicon-' (e.g. glyphicon-star), VIKI will look for a bootstrap icon. This is an easy way to quickly add fancy icons. An overview of icons can be found at `bootstrap icons`_ 

.. _`bootstrap icons`: http://getbootstrap.com/components/

Dependencies
------------
To be able to automatically let VIKI install the ROS packages, you need to specify which you are using. This is simply done by specifying the name of the ROS package inside the dependency tags.

.. code-block:: xml

    <dependencies>
        <depends>libuvc_camera</depends>
    </dependencies>

This can now be automatically installed using the VIKI command line tool. If there is no apt-get package available for your package, you can specify the repository of the package. Set a 'type' and 'src' attribute in the depends tag like such:

.. code-block:: xml

    <dependencies>
		<depends type="git" src="https://github.com/ros-drivers/mocap_optitrack">mocap_optitrack</depends>
	</dependencies>

Inputs and Outputs
------------------

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
-----------
An executable in a ROS node specifies a ROS node that is to be executed. 

.. code-block:: xml

    <executable id="usb_cam" pkg="libuvc_camera" exec="camera_node">
        <params>
            <param name="vendor" type="str" default="null" />
            <param name="product" type="str" default="null" />
            <param name="width" type="str" default="640" />
            <param name="height" type="str" default="480" />
            <param name="frame_rate" type="str" default="15.0" />
        </params>

        <outputs>
            <output type="ros_topic" name="image" message_type="sensor_msgs/Image" required="false" />
        </outputs>
    </executable>

The first line has three attributes:

* *id*: This is the id used in the configuration to specify this executable. The module inputs and outputs are linked to executable inputs and outputs using this id.
* *pkg*: The package from which to run the node
* *exec*: The node that is to be run

The pkg and exec parameters correspond to running the node with

.. code-block:: bash

    rosrun <package> <executable>

The params block corresponds to the parameters that can be set for each executable. The type corresponds to the types of `ros parameters`_. Since this module file is basically an template for what will be runned, only a default option can be set and no definite value. These default options can be changed by the user using VIKI before launch.

.. _ros parameters: http://wiki.ros.org/roslaunch/XML/param