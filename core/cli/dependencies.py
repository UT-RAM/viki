__author__ = 'robin'

"""
    Python file with a set of functions that handles basically all dependency management concerning ROS for viki

    A couple of kinds of dependencies:
    - First level dependencies are the dependencies of ROS packages that are needed directly for modules
    - Second level dependencies are dependencies of ROS packages, these can be
        - System level, to be installed with rosdep
        - ROS package leven, to be installed (parsed, pulled, etc.) with rospack
"""
"""VIKI_VERSION_INFO
VIKI: more than a GUI for ROS 
version number: 0.1
version name: Alice

Copyright (c) 2016 Robin Hoogervorst, Alex Kamphuis, Cees Trouwborst, https://github.com/UT-RAM/viki

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Connection between the HTML GUI and Python backend was established using work by David Baird following his tutorial on http://www.aclevername.com/articles/python-webgui/

Thank you for letting us use your great work, David.

This work has been funded by the European Commission's H2020 project AEROWORKS under grant no. 644128
END_VERSION_INFO"""

from core.backend import scan
from viki_config import VikiConfig
import os
import subprocess


def check_installed_packages():
    """
    Checks for every module that is available,
    if the ROS packages needed for it are installed on the system

    :return: Boolean
    """
    viki_config = VikiConfig()
    missing_packages = get_missing_packages(viki_config)

    if len(missing_packages) > 0:
        print "[WARNING] - There are missing packages for full VIKI support:"
        print "\n".join(map((lambda x: x['name']), missing_packages))
        return False
    else:
        print "[OK] - All ROS package dependencies are met!"
        print "Note: only second level dependencies of already installed packages have been checked"
        return True

def get_installed_packages():
    """
    Gets the currently installed ROS packages and returns the names as an array
    :return:
    """
    p = subprocess.Popen(['rospack', 'list-names'], stdout=subprocess.PIPE)
    packages = p.stdout.read().split()

    return packages

def get_missing_packages(viki_config):
    """
    Looks for missing ROS packages that are defined as dependency in a module
    :return: List with names of the missing ROS packages
    """
    installed_packages = get_installed_packages()
    modules_on_system = get_modules(viki_config)

    missing_packages = []

    # if multiple modules use that package
    for module in modules_on_system:
        for package in module.package_dependencies:
            if not package['name'] in installed_packages:
                # Only add the package if it is not yet in the list
                if not package in missing_packages:
                    missing_packages.append(package)
                module.missing_packages.append(package)

    return missing_packages

def get_second_level_dependencies():
    p = subprocess.Popen(['rosdep', 'check', '--from-paths', '../'], stdout=subprocess.PIPE)
    output_string = p.stdout.read()

    print output_string
    if "All system dependencies" in output_string:
        return True

    return False

def install_second_level_dependencies():
    p = subprocess.Popen(['rosdep', 'install', '--from-paths', '../'], stdout=subprocess.PIPE)
    print p.stdout.read()
    p.communicate()

def get_package_locations():
    """
    Uses 'rosdep db' and parsing to get the packages that we can install and the apt-get packages to install it

    :return:
    """
    p = subprocess.Popen(['rosdep', 'db'], stdout=subprocess.PIPE)
    package_lines = p.stdout.read().splitlines()
    package_map = map((lambda x: x.split(' -> ')), package_lines)
    return package_map

def get_aptget_packages(ros_package_names):
    """
        Returns installation candidates in a list [[ros_package_name, aptget_package], ...]
        This is based on the package names provided as the parameter
    :param ros_package_names:
    :return:
    """
    apt_packages = get_package_locations()
    return filter((lambda x: x[0] in ros_package_names), apt_packages)


def start_aptget_installation(installation_candidates):
    """
    Installs the packages that are provided in aa package list
    :param installation_candidates:
    :return:
    """
    if (len(installation_candidates) == 0): return

    print "VIKI is going to install the following missing packages using apt-get: \n"
    print ",".join(map((lambda x: x[0]+" ("+x[1]+")"), installation_candidates))
    print "It may ask for your sudo password, to be able to execute apt-get"
    input = None

    # Ask for confirmation, pressing enter yields Y as default
    while input not in ["y", "n", "Y", "N", ""]:
        print "Are you OK with that?"
        input = raw_input("[Y/n]: ")

    if input == "n" or input == "N":
        print "Aborting installation..."
        return

    command = ["sudo", "apt-get", "install"] + map((lambda x: x[1]), installation_candidates)
    subprocess.call(command)

def start_vcs_installation(missing_vcs_packages):
    if (len(missing_vcs_packages) == 0): return

    print "VIKI is going to install the following missing packages using version control: \n"
    print "\n".join(map((lambda x: x['name']+" ("+x['src']+")"), missing_vcs_packages))
    input = None

    # Ask for confirmation, pressing enter yields Y as default
    while input not in ["y", "n", "Y", "N", ""]:
        print "Are you OK with that?"
        input = raw_input("[Y/n]: ")

    if input == "n" or input == "N":
        print "Aborting installation..."
        return

    if not os.path.exists("../viki_dependencies"):
        os.makedirs("../viki_dependencies")

    for package in missing_vcs_packages:
        command = []
        if package['type'] == 'git':
            command = ['git', 'clone', package['src'], '../viki_dependencies/'+package['name']]

        if command != []:
            subprocess.call(command)
    return None

def get_modules(viki_config):
    # TODO: Cache the module list in here, so that we don't scan all files again and again, and again...
    # if module_list == None:

    module_list = scan.getAvailableModules(viki_config)
    return module_list

def get_distinct_packages(modules):
    packages_per_module = filter((lambda x: len(x) > 0), map((lambda x: x.package_dependencies), modules))
    packages_list = reduce((lambda x,y: x+y), packages_per_module)
    return list(set(packages_list)) # filter out duplicates..


