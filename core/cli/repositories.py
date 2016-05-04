__author__ = 'robin'

import subprocess

"""
    Package that handles repository installation.
    Right now, this is very basic, but can be more elaborate in the future
"""

repositories = {
    'core':  {
        'url': 'https://github.com/UT-RAM/viki-modules',
        'type': 'git',
        'branch': 'dev'
    }
}

def clone_module_repository(repo, viki_config):
    """
    Clone a repository into the viki_modules folder
    :param repo:
    :return:
    """
    repository = repositories[repo]

    if repository == None:
        raise Exception("Repository {} is not a valid configuration".format(repo))

    # TODO: Check if repository exists!

    target_directory = "{}/{}".format(viki_config.get_option('root_module_directory'), repo)
    command = []
    if repository['type'] == 'hg':
        command = ['hg', 'clone', repository['url'], target_directory, '-r', repository['branch']]
    elif repository['type'] == 'git':
        command = ['git', 'clone', repository['url'], target_directory, '-b', repository['branch']]

    subprocess.call(command)

def catkin_make():
    # does not work yet!
    p = subprocess.Popen('. ~/.bashrc && catkin_make --directory ../..', stdout=subprocess.PIPE)
    print p.stdout.read()
    p.communicate()