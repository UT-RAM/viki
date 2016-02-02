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
to further setup your system. From within VIKI directory, run 

    ./viki configure
    
To let VIKI configure itself _(does not work yet fully)_. Now, run

    ./viki run
    
To launch VIKI. Congratulations! You can now start using VIKI, look further into the documentation on how to use VIKI and how to add modules.
