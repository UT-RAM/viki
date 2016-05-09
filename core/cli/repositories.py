__author__ = 'robin'

import subprocess
import os

"""
    Package that handles repository installation.
    Right now, this is very basic, but can be more elaborate in the future
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

    target_directory = os.path.expanduser("{}/{}".format(viki_config.get_option('root_module_directory'), repo))
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