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

The documentation for VIKI is hosted on [Read the Docs](http://viki.readthedocs.io).

Documentation is included in the repository as well. Open ``docs/_build/html/index.html `` in your favourite browser to read this. To launch the documentation using a terminal, from within the VIKI directory, run

    xdg-open docs/_build/html/index.html
    
## Issues and contact

If you encounter any bugs or have ideas for improving VIKI, please do not hesitate to contact us. This can either be done by openining an issue, or contact us directly using the contact information in the documentation.

## Licensing

VIKI is provided under a MIT license. 