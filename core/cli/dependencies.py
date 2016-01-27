__author__ = 'robin'

"""
    Python file with a set of functions that handles basically all dependency management concerning ROS for viki

    A couple of kinds of dependencies:
    - First level dependencies are the dependencies of ROS packages that are needed directly for modules
    - Second level dependencies are dependencies of ROS packages, these can be
        - System level, to be installed with rosdep
        - ROS package leven, to be installed (parsed, pulled, etc.) with rospack
"""

from core.aero import scan
import subprocess


def check_installed_packages():
    """
    Checks for every module that is available,
    if the ROS packages needed for it are installed on the system

    :return:
    """
    missing_packages = get_missing_packages()

    if len(missing_packages) > 0:
        print "[WARNING] - There are missing packages for full VIKI support:"
        print "\n".join(missing_packages)
    else:
        print "[OK] - All ROS package dependencies are met!"
        print "Note: only second level dependencies of already installed packages have been checked"

def get_installed_packages():
    """
    Gets the currently installed ROS packages and returns the names as an array
    :return:
    """
    p = subprocess.Popen(['rospack', 'list-names'], stdout=subprocess.PIPE)
    packages = p.stdout.read().split()

    return packages

def get_missing_packages():
    """
    Looks for missing ROS packages that are defined as dependency in a module
    :return: List with names of the missing ROS packages
    """
    installed_packages = get_installed_packages()
    modules_on_system = get_modules()

    missing_packages = []

    # if multiple modules use that package
    for module in modules_on_system:
        for package in module.package_dependencies:
            if not package in installed_packages:
                # Only add the package if it is not yet in the list
                if not package in missing_packages:
                    missing_packages.append(package)
                module.missing_packages.append(package)

    return missing_packages

def get_second_level_dependencies():
    p = subprocess.Popen(['rosdep', 'check', '--from-paths', '../'], stdout=subprocess.PIPE)
    print p.stdout.read()

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


def start_installation(installation_candidates):
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

def get_modules():
    # TODO: Cache the module list in here, so that we don't scan all files again and again, and again...
    # if module_list == None:
    module_list = scan.getAvailableModules()

    return module_list

def get_distinct_packages(modules):
    packages_per_module = filter((lambda x: len(x) > 0), map((lambda x: x.package_dependencies), modules))
    packages_list = reduce((lambda x,y: x+y), packages_per_module)
    return list(set(packages_list)) # filter out duplicates..