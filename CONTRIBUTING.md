#Contributing to VIKI

We love third-party contributions to VIKI! This guide should make it easy for you to make changes and get your changes into VIKI. 

## Core vs. Modules

This repository holds the core of VIKI. If you want to make any changes in that, that's great! If you want to add functionality to VIKI by adding an extra module, please go to `https://github.com/UT-RAM/viki-modules` and place your contribution there. For more information about the difference between the core and modules, have a look at the documentation.

## Getting started

- Make sure you have a GitHub account
- Fork the repository on GitHub
- Create an issue so that we know what you're working on and can discuss what needs to be changed

## Making changes

We follow the git branching model as described at `http://nvie.com/posts/a-successful-git-branching-model/`, meaning contributions go in a feature branch, named after the issue that was created. 

1. Create a topic-branch where you want to base your work
    - Command: `git checkout -b {feat,fix,etc.}-my_contribution dev
    - Usually this will be the dev branch
    - Unless you're aiming for a specific fix on a release branch
    - Avoid working directly off master
1. Make commits of logical units on your new branch

## Submitting changes

1. Push your changes to the topic-branch of your own fork. Make sure you rebase on the most recent commit on the master branch of the main repository.
1. Submit a pull request to the appropriate branch within the origin
1. Wait for us to comment on your pull request. We may suggest some changes or improvements or alternatives.
1. Be happy and awesome, because you're making the robotics community more awesome!

