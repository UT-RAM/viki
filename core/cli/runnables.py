__author__ = 'robin'

import subprocess

import  dependencies
import repositories

# Be careful to import stuff here, this is the very low level of VIKI, where not all dependencies are met yet
# If something is imported, test thoroughly!

def run(options):
    """
        Run VIKI :D
        :return:
    """
    from core import __main__ as viki_core
    viki_core.run()

def configure(options):
    """
        Usually the first command to run. This will install apt-get dependencies to run VIKI properly,
        and do some more configuration like creating a desktop entry
    :return:
    """
    # Install the right dependencies
    to_install_packages = ['python-webkit', 'python-gtk2', 'python-simplejson']
    subprocess.call(['sudo', 'apt-get', 'install']+to_install_packages)

    # Create fancy desktop entry

    return None


def check_packages(options):
    """
        Checks if all packages that should be installed are installed
        First-level: ROS packages that are required for VIKI directly
        Second-level: Dependencies of ROS-packages
        :return:
    """
    print '\033[1;33m# Checking direct VIKI dependencies\033[1;m'
    installed_ok = dependencies.check_installed_packages()
    print '\n\033[1;33m# Checking second level ROS dependencies, using rosdep\033[1;m'
    second_level_ok = dependencies.get_second_level_dependencies()

    if installed_ok and second_level_ok:
        print '\033[1;32mAll dependencies satisfied!\033[1;m'
    else:
        print '\033[1;31mTry running [viki install-dependencies] to install the dependencies\033[1;m'

def install_packages(options):
    """
        Installs packages that the 'check_packages' function determines as missing
        This can either be with apt-get, or git, something else is not yet supported
        :return:
    """
    missing_ros_packages = dependencies.get_missing_packages()
    if len(missing_ros_packages) == 0:
        print "[OK] - All ROS package dependencies are met, noting to install!"

    missing_aptget_packages = map((lambda x: x['name']), filter((lambda x: x['type'] == 'apt-get'), missing_ros_packages))
    missing_vcs_packages = filter((lambda x: x['type'] == 'git'), missing_ros_packages)

    installation_candidates = dependencies.get_aptget_packages(missing_aptget_packages)
    dependencies.start_aptget_installation(installation_candidates)
    dependencies.start_vcs_installation(missing_vcs_packages)

def add_module_repository(options):
    # does not work yet really...
    repositories.clone_module_repository('core')
    # install direct dependencies
    install_packages()
    # install using rosdep, for build dependencies
    dependencies.install_second_level_dependencies()
    # build
    # repositories.catkin_make()
    # TODO: Make this work!