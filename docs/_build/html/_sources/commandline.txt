Commandline Interface
=====================

VIKI comes with a command line interface. During installation, you have already used this. This command line interface is mainly used for dependency management and installing ROS packages that are needed for VIKI modules.

The following commands are available

    run
        Runs VIKI

    configure
        Sets VIKI up for the first time. This installs dependencies, places VIKI in your PATH and creates a desktop entry, so VIKI can be launched easily

    check-packages
        This will check if ROS dependencies are met. First it will look for ROS packages that are not installed, but that are needed for the module repositories that are installed. Furthermore, it will use the rosdep tool to install second level dependencies of ROS packages.

    install-packages
        This will automatically install all packages that are missing, according to check-packages. Normally, this tool will install the packages using apt-get, but if that is not available, it will pull the dependencies from source

    add-module-repository
        This will add another module repository. Right now, it will always pull the core repository, but this is te be adapted soon.

To see this information, you can also run the viki help command.

.. code-block:: bash

    ./viki --help

