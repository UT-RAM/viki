.. _`howtoconfig`:

Configuration writing guide
===========================



Writing configuration tutorials
-------------------------------

.. _`simpleconfigtutorial`:

Simple tutorial
^^^^^^^^^^^^^^^
In this simple tutorial you are going to write a configuration file and a configuration to start the rqt_node that you might have created in :ref:`simplemoduletutorial`. If you have not made it yet, please do so first by following the tutorial.

Now go to the folder (inside contrib) you have created for yourself and create a file there, for instance using gedit:

.. code-block:: bash

    cd ~catkin_ws/src/aeroworks/contrib/DoutzenMSC && gedit configuration.xml

To let the framework know you are working on an Aeroworks configuration file start with and save:

.. code-block:: xml

    <!-- AEROWORKS -->

Now add the following tags:

.. code-block:: xml

    <configurations>

    </configurations>

You can place more then one configuration in the this file, so that you may collect all sort of different setups together. To start writing one specific configuration add, between the *configurations* tag and save:

.. code-block:: xml

    <configuration id="onlyrqt">
        <feedback_to_user type="rqtmodule" id="rqtnode1" />
    </configuration>

Your configuration is now ready to be started. It should look like this:

.. code-block:: xml

    <!-- AEROWORKS -->
    <configurations>
        <configuration id="onlyrqt">
        <feedback_to_user type="rqtmodule" id="rqtnode1" />
        </configuration>
    </configurations>

Go to the Aeroworks root folder and start the core, providing your configuration file and configuration id as arguments. If you used the same file and configuration names as in the tutorials this should do it for you:

.. code-block:: bash

    cd ~catkin_ws/src/aeroworks/ && python core contrib/DoutzenMSC/configuration.xml onlyrqt

Now run

.. code-block:: bash

    roslaunch aeroworks.launch

and watch as the rqt graph shows you which ros nodes are running. You can for instance launch the turtle simulator using the :ref:`userguide` and see what topics it is using!

Advanced tutorial
^^^^^^^^^^^^^^^^^