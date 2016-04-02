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
#VIKI

VIKI is a GUI tool for running ROS experiments. It helps you setting up experiments quickly and offers a lot of flexibility. 
VIKI is designed to aid in setting up ROS packages and finding the right tools for you. 
 
 ![viki_screenshot](/docs/viki_screenshot.png)
 
##Installation

Basic installation of VIKI consists of pulling this repository and letting VIKI configure itself. After installing the core of VIKI, 
you can add module repositories, so you are able to actually use stuff.

###Dependencies

Make sure you have at least git and python installed. Usually these are installed already, but if not:

    sudo apt-get install git python
    
Also, make sure that you have ROS installed. If not, you can install it using  [this guide](http://wiki.ros.org/jade/Installation/Ubuntu). 
Make sure to also follow points 1.5 and 1.6 with the rosdep initialization and setup of your environment. 

###VIKI
After that, create a new ROS workspace and clone this repository into the src folder of that.

     mkdir -p catkin_ws/src
     cd catkin_ws/src
     catkin_init_workspace
     
     git clone https://github.com/UT-RAM/viki
     
This pulls the files needed right into your catkin_workspace. VIKI supports a commandline tool that you can use
to further setup your system. To configure VIKI, from within VIKI directory, run 
    
    ./viki configure
    
Now you're all set! We are ready to launch VIKI. Try running:
    
    ./viki run
    

Congratulations! You can now start using VIKI, look further into the documentation on how to use VIKI and how to add modules.

## Documentation

Right now, the documentation is not yet hosted to the public, but included in the repository. After cloning the repository locally, open the file `` <repo_dir>/docs/_build/html/index.html `` in your favourite browser to read the docs.
