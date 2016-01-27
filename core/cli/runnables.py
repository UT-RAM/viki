__author__ = 'robin'

import subprocess

from core import __main__ as viki_core

import  dependencies
import repositories

def run():
    viki_core.run()

def configure():
    # Install the right dependencies
    to_install_packages = ['python-webkit', 'python-gtk2', 'python-simplejson']
    subprocess.call(['sudo', 'apt-get', 'install']+to_install_packages)

    # Create fancy desktop entry

    return None


def check_packages():
    print '\033[1;33m# Checking direct VIKI dependencies\033[1;m'
    dependencies.check_installed_packages()
    print '\n\033[1;33m# Checking second level ROS dependencies, using rosdep\033[1;m'
    dependencies.get_second_level_dependencies()

    print '\033[1;32mTry running [viki install-dependencies] to install the dependencies\033[1;m'

def install_packages():
    missing_ros_packages = dependencies.get_missing_packages()
    if len(missing_ros_packages) == 0:
        print "[OK] - All ROS package dependencies are met, noting to install!"

    installation_candidates = dependencies.get_aptget_packages(missing_ros_packages)
    dependencies.start_installation(installation_candidates)

def add_module_repository():
    # does not work yet really...
    repositories.clone_module_repository('core')
    # install direct dependencies
    install_packages()
    # install using rosdep, for build dependencies
    dependencies.install_second_level_dependencies()
    # build
    repositories.catkin_make()