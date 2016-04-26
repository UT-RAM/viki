#!/usr/bin/python
import sys
import os

from datetime import date
import fnmatch

class VersionBumper:

    version_file = 'VERSION'
    license_template = 'VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki \n' \
                       'version: {} - {}\n' \
                       'copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, {} \n' \
                       'licensed under the MIT License'

    def __init__(self):
        self.version = self.parse_version()
        self.list_dirs = ['.', 'core/backend', 'core/cli', 'core/gui', 'core/gui/js', 'core/gui/css']
        self.whitelist_files = ['*.py', 'viki', 'viki_env.sh', 'viki_launch.sh', 'viki.js', 'viki.css', 'VIKI_main.html']
        self.blacklist_files = ['bumpversion.py']

        self.license_text = self.license_template.format(self.version[0], self.version[1], date.today().year)

    def run(self):
        print "Adding version info to listed files.."
        print "VERSIONING: {} - {}\n".format(self.version[0], self.version[1])
        self.process_files()

    def process_files(self):
        for dir in self.list_dirs:
            for item in os.listdir(dir):
                item_path = os.path.join(dir, item)
                if not os.path.isfile(item_path):
                    continue
                if not self.check_file(item):
                    continue
                self.process_file(item_path)
                print "exiting early because of testing reaons..."

    def process_file(self, filename):
        print "processing file {}".format(filename)
        text_to_add = self.get_license_comment(filename)
        self.prepend_to_file(filename, text_to_add)

    def get_license_comment(self, filename):
        _, extension = os.path.splitext(filename)
        if (extension in ['.py', '']):
            return '"""\n{}\n"""'.format(self.license_text)
        if (extension in ['.js', '.css']):
            return '/*\n{}\n*/'.format(self.license_text)
        if (extension == '.html'):
            return '<!--\n{}\n-->'.format(self.license_text)
        if (extension == '.sh'):
            return '\n'.join(map(lambda x: '# '+x, self.license_text.split('\n')))

        raise Exception("Extension {} is not valid for genreting licensing..".format(extension))

    def prepend_to_file(self, filename, line):
        with open(filename, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(line.rstrip('\r\n') + '\n\n' + content)

    def check_file(self, filename):
        """
        Check if we should process a file based on white and blacklisting
        the file expression MUST be in the whitelist
        and MUST NOT be present in the blacklist
        this way, we can exclude specific cases from the whitelist
        :param filename:
        :return: bool
        """
        for expression in self.whitelist_files:
            if fnmatch.fnmatch(filename, expression):
                for blacklist_item in self.blacklist_files:
                    if fnmatch.fnmatch(filename, blacklist_item):
                       return False
                return True

        return False

    def parse_version(self):
        with open(self.version_file, 'r') as file:
            v_number = file.readline().rstrip()
            v_name = file.readline().rstrip()
        # TODO: validate number, name
        return (v_number, v_name)

if __name__ == '__main__':
    vm =  VersionBumper()
    vm.run()
