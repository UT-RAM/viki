#VIKI

VIKI is a GUI tool for running ROS experiments.
 
##Installation

Basic installation of VIKI consists of pulling this repository and letting VIKI configure itself. After installing the core of VIKI, 
you can add module repositories, so you are able to actually use stuff.

Make sure you have at least mercurial and python installed by running:

    sudo apt-get install mercurial python
    
After that, create a new ROS workspace and clone this repository into the src folder of that.

     mkdir -p catkin_ws/src
     cd catkin_ws/src
     catkin_init_workspace
     
     hg clone https://hg.ram.ewi.utwente.nl/viki -r dev