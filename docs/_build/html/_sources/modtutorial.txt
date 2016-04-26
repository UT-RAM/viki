.. _`modtutorial`:

Tutorial on module XML writing
==============================

.. _`simplemoduletutorial`:

In this tutorial we will create a module. The module in this tutorial will start the ros_rqt_graph (which is already installed in ROS), which is a nice graph of ROS nodes and topics active on your PC.

To keep your module nicely separated from the rest it is advisable to create your modules in a new folder. Let us assume that your name is Doutzen Kroes and you name your folder *DoutzenMSC*. If you have installed the framework in the default location then running this should do the job for you:

.. code-block:: bash

    cd ~/catkin_ws/src/viki_modules/ && mkdir DoutzenMSC

Inside there create your very first module definition file. You have no choice for name here, it must be named *viki.xml*. Open it to start defining your module. This should help you:

.. code-block:: bash

    gedit viki.xml

Write, on the first line of the file

.. code-block:: xml

    <!-- AEROWORKS -->

and save the file. Congratulations, you have just defined your first module! Nevertheless you did not do a good job, yet. Try running VIKI (you should know how by now). You should see that there is something wrong with the file you have just created because the terminal tells you something like:

.. code-block:: bash

    Skipped adding '../aeroworks/contrib/DoutzenMSC/viki.xml' because it is a broken file. Error thrown was:
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

As you can see that is quite a lot of text. The framework does not crash, and other modules are still available, but you can imagine that this is undesireable.

Now open your gedit with viki.xml again, and we will create something more useful.

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

For your convenience, this is how your viki.xml file should look now:

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

Your module is now finished and runnable. Of course, these are the real basics, look at :ref:`modtaglist`
to go more in-depth regarding writing a module file.
