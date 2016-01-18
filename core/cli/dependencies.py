__author__ = 'robin'

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
    modules_on_system = scan.getAvailableModules()

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
    apt_packages = get_package_locations()
    return filter((lambda x: x[0] in ros_package_names), apt_packages)


def start_installation(installation_candidates):
    if (len(installation_candidates) == 0): return

    print "VIKI is going to install the following missing packages using apt-get: \n"
    print ",".join(map((lambda x: x[0]+" ("+x[1]+")"), installation_candidates))
    print "It may ask for your sudo password, to be able to execute apt-get"
    input = None

    while input not in ["y", "n", "Y", "N", ""]:
        print "Are you OK with that?"
        input = raw_input("[Y/n]: ")

    if input == "n" or input == "N":
        print "Aborting installation..."
        return

    command = ["sudo", "apt-get", "install"] + map((lambda x: x[1]), installation_candidates)
    subprocess.call(command)

def fix_system_dependencies():
    subprocess.call(['rosdep', 'install', '--all', '-r'])