#!/usr/bin/python
import sys
import os

from datetime import date
import fnmatch

# def processFile(fname, versiontext):
#     print('Processing file: ', fname)
#     tf = open(tempfile, 'w')
#     foundFlag = False
#     try:
#         with open(fname, 'r') as f:
#             for line in f:
#                 if versionstart in line:
#                     foundFlag = True
#                     tf.write(line)
#                     tf.write(versiontext)
#                     tf.write(lincensetext)
#                     tf.write('\n')
#                     tf.write(granttext)
#                     tf.write('\n')
#                     continue
#                 if versionend in line:
#                     foundFlag = False
#                     tf.write(line)
#                     continue
#                 if not foundFlag:
#                     tf.write(line)
#                     continue
#     finally:
#         tf.close()
#         os.rename(tempfile, fname)
#     print('Done!')


class VersionBumper:

    version_file = 'VERSION'
    license_template = 'VIKI: more than a GUI for ROS, https://github.com/UT-RAM/viki \n' \
                       'version: {} - {}\n' \
                       'copyright: Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, {} \n' \
                       'licensed under the MIT License\n\n'

    def __init__(self):
        self.version = self.parse_version()
        self.list_dirs = ['.', 'core/backend', 'core/cli', 'core/gui', 'core/gui/js', 'core/gui/css']
        self.whitelist_files = ['*.py', 'viki', 'viki_env.sh', 'viki_launch.sh', 'viki.js', 'viki.css', 'VIKI_main.html']
        self.blacklist_files = ['bumpversion.py']

        self.license_text = self.license_template.format(self.version[0], self.version[1], date.today().year)

    def run(self):
        print "Adding version info to listed files.."
        print "VERSIONING: {} - {}\n".format(self.version[0], self.version[1])

    def process_files(self):
        for dir in self.list_dirs:
            for item in os.listdir(dir):
                item_path = os.path.join(dir, item)
                if not os.path.isfile(item_path):
                    continue
                if not self.check_file(item):
                    continue
                self.process_file(item_path)

    def process_file(self, filename):
        self.prepend_to_file(filename, self.license_text)

    def prepend_to_file(self, filename, line):
        with open(filename, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(line.rstrip('\r\n') + '\n' + content)

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
