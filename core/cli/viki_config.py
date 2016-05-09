"""
VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki 
version: 0.2 - Alice
copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, 2016 
licensed under the MIT License
"""


import json
from collections import OrderedDict
import unicodedata

class VikiConfig:

    config_filename = 'config.json'

    def __init__(self):
        self.config = OrderedDict()
        self.load_config()

    def set_option(self, option, value):
        self.config[option] = value
        self.save_config()

    def get_option(self, option, none_valid=False):
        if option not in self.config:
            if none_valid:
                return None
            raise Exception("Option {} not available in the configuration".format(option))

        return self.config[option]

    def load_config(self):
        try:
            with open(self.config_filename, 'r') as c_file:
                loaded_config = json.load(c_file)
                for key in loaded_config.keys():
                    new_index = unicodedata.normalize('NFKD', key).encode('ascii', 'ignore')
                    self.set_option(new_index, loaded_config[key])
        except:
            raise Exception("Error during loading of configuration")

    def save_config(self):
        with open(self.config_filename, 'w') as c_file:
            c_file.write(json.dumps(self.config, indent=1))
            c_file.write('\n')

    # Extra specific getters and setters

    def get_root_module_dir(self):
        return self.get_option('root_module_directory')