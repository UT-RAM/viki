
import json
from collections import OrderedDict

class VikiConfig:

    config_filename = 'config.json'

    def __init__(self):
        self.load_config()
        self.config = OrderedDict()

    def set_option(self, option, value):
        self.config[option] = value
        self.save_config()

    def get_option(self, option, none_valid=False):
        if option not in self.config:
            if none_valid:
                return None
            raise Exception("Option {} not available in the configuration")

        return self.config[option]

    def load_config(self):
        try:
            with open(self.config_filename, 'r') as c_file:
                self.config = json.load(c_file)
        except:
            pass

    def save_config(self):
        with open(self.config_filename, 'w') as c_file:
            c_file.write(json.dumps(self.config, indent=1))
            c_file.write('\n')