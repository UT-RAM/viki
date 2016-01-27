__author__ = 'robin'

import subprocess

repositories = {
    'core': {
        'url': 'https://hg.ram.ewi.utwente.nl/viki_modules',
        'type': 'hg',
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

    subprocess.call(command)

def catkin_make():
    # does not work yet!
    p = subprocess.Popen('cd ../.. && catkin_make', stdout=subprocess.PIPE)
    print p.stdout.read()
    p.communicate()