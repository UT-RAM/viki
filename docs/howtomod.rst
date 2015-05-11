.. _`howtomod`:

Module writing guide
====================

This guide consists of two things:

#. :ref:`modtutorial`
#. :ref:`modtaglist`

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