__author__ = 'robin'

repositories = {
    'default': {
        'url': 'https://hg.ram.ewi.utwente.nl/viki_modules',
        'type': 'hg'
    }
}

def clone_module_repository(repo):
    repository = repositories[repo]

    if repository == None:
        print "Not known repository"

    print repository['url']