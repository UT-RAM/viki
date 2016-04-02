<!-- VIKI_VERSION_INFO
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
END_VERSION_INFO -->
#Contributing to VIKI

We love third-party contributions to VIKI! This guide should make it easy for you to make changes and get your changes into VIKI. 

## Core vs. Modules

This repository holds the core of VIKI. If you want to make any changes in that, that's great! If you want to add functionality to VIKI by adding an extra module, please go to `https://github.com/UT-RAM/viki-modules` and place your contribution there. For more information about the difference between the core and modules, have a look at the documentation.

## Getting started

- Make sure you have a GitHub account
- Fork the repository on GitHub
- Create an issue so that we know what you're working on and can discuss what needs to be changed

## Making changes

1. Create a topic-branch where you want to base your work
    - Command: `git checkout -b {feat,fix,etc.}-my_contribution dev
    - Usually this will be the dev branch
    - Unless you're aiming for a specific fix on a release branch
    - Avoid working directly off master
1. Make commits of logical units on your new branch

## Submitting changes

1. Push your changes to the topic-branch of your own fork
1. Submit a pull request to the appropriate branch within the origin
1. Wait for us to comment on your pull request. We may suggest some changes or improvements or alternatives.

