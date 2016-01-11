__author__ = 'robin'

from core.aero import scan
import subprocess

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
    Looks for missing packages and returnes the names of those, if found
    :return:
    """
    installed_packages = get_installed_packages()
    modules_on_system = scan.getAvailableModules()

    missing_packages = []

    for module in modules_on_system:
        for package in module.package_dependencies:
            if not package in installed_packages:
                missing_packages.append(package)
                module.missing_packages.append(package)

    return missing_packages

def check_installed_packages():
    """
    Checks for every module that is available,
    if the ROS packages needed for it are installed on the system

    :return:
    """
    missing_packages = get_missing_packages()

    if len(missing_packages) > 0:
        print "[WARNING] - There are missing packages: {}".format(missing_packages)
    else:
        print "[OK] - All ROS package dependencies are met!"

