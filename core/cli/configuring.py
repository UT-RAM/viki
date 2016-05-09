"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""


import os
from collections import OrderedDict

from viki_config import VikiConfig

class Configuring:
    """
    Class which will run the configuration for the user
    especially useful the first time
    """

    def __init__(self):
        self.config = VikiConfig()

        # Return functions here, which are used with lazy evaluation,
        # such that we can evaluate already set options
        # Also, initialize it in this way, because of order
        self.default_options = OrderedDict()
        self.default_options['ros_version'] = lambda: 'indigo'
        self.default_options['ros_dir'] = lambda: '/opt/ros/' + self.config.get_option('ros_version')
        self.default_options['viki_dir'] = lambda: os.getcwd()
        self.default_options['catkin_workspace'] = lambda : '~/catkin_ws'
        self.default_options['root_module_directory'] = lambda : self.config.get_option('catkin_workspace') + '/src/viki_modules'

    def run(self):
        print "Configuring settings:\nPress enter to use default, or enter your own value"
        for option in self.default_options.keys():
            default = self.config.get_option(option, True)
            if default is None:
                default = self.default_options[option]()

            value = self.ask_config_option(option, default)
            self.config.set_option(option, value)
        self.config.save_config()

    def ask_config_option(self, option, value):
        input = raw_input("{}: [{}]".format(option, value))
        if input is not "":
            return input
        return value