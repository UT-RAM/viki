__author__ = 'robin'

import subprocess
import os
import stat
import shutil

import  dependencies
import repositories
from configuring import Configuring

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
    configuring = Configuring()
    configuring.run()
    config = configuring.config

    # Install the right dependencies
    print_to_terminal("Installing extra dependencies (webkit, gth and simplejson)")
    to_install_packages = ['python-webkit', 'python-gtk2', 'python-simplejson']
    subprocess.call(['sudo', 'apt-get', 'install']+to_install_packages)

    # Create fancy desktop entry
    for file in ['viki.desktop', 'viki_env.sh', 'viki_launch.sh']:
        process_template(file, config)
        os.chmod(file, os.stat(file).st_mode | stat.S_IEXEC)
    app_dir = os.path.expanduser('~/.local/share/applications')
    command = "desktop-file-install --dir={} {}/viki.desktop".format(app_dir, os.getcwd())
    print command
    subprocess.call(command)

    return None

def process_template(file, viki_config):
    print viki_config.config.keys()
    template = open('file_templates/'+file+'.template', 'r').read()
    for option in viki_config.config.keys():
        print option
        template = template.replace('{{'+option+'}}', viki_config.get_option(option))
    print template
    with open(file, 'w') as write_file:
        write_file.write(template)


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


def print_to_terminal(string, color='black'):
    print '\033[1;33m# {} \033[1;m'.format(string)
    return 0
