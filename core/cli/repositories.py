__author__ = 'robin'

import subprocess

repositories = {
    'core_old': {
        'url': 'https://hg.ram.ewi.utwente.nl/viki_modules',
        'type': 'hg',
        'branch': 'dev'
    },
    'core':  {
        'url': 'https://github.com/UT-RAM/VIKI_modules',
        'type': 'git',
        'branch': 'dev'
    }
}

def clone_module_repository(repo):
    repository = repositories[repo]

    if repository == None:
        print "Not known repository"

    command = []
    if repository['type'] == 'hg':
        command = ['hg', 'clone', repository['url'], '../viki_modules/'+repo, '-r', repository['branch']]
    elif repository['type'] == 'git':
        command = ['git', 'clone', repository['url'], '../viki_modules/'+repo, '-b', repository['branch']]

    subprocess.call(command)

def catkin_make():
    # does not work yet!
    p = subprocess.Popen('. ~/.bashrc && catkin_make --directory ../..', stdout=subprocess.PIPE)
    print p.stdout.read()
    p.communicate()