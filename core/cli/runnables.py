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
     dependencies.check_installed_packages()

def install_packages():
    missing_ros_packages = dependencies.get_missing_packages()
    if len(missing_ros_packages) == 0:
        print "[OK] - All ROS package dependencies are met, noting to install!"

    installation_candidates = dependencies.get_aptget_packages(missing_ros_packages)
    dependencies.start_installation(installation_candidates)

def add_module_repository():
    repositories.clone_module_repository('core')