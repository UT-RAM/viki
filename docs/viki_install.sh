#!/bin/sh

echo " --  Starting installation of VIKI  --"
echo " -- University of Twente, RaM, Oktober 2015 --\n"

echo " -- Installation of packages -- "
sudo apt-get install mercurial subversion

echo "\n -- Pulling VIKI repositories -- "
core_repo="https://hg.ram.ewi.utwente.nl/viki"
module_repo="https://hg.ram.ewi.utwente.nl/viki_modules"

# creation of workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ../..

# pulling repos
hg clone $core_repo catkin_ws/src/viki -r dev
hg clone $module_repo catkin_ws/src/viki_modules/core_modules -r dev

# After this, we need extra ros packages. There are a couple 
# of ways to install these. From source, or using apt-get
# This provides both ways to support the largest set
prefix="ros-jade"
sudo apt-get install "$prefix-joy"

echo "\n -- Installing extra dependencies -- "
#python viki stuff
sudo apt-get install python python-webkit python-gtk2 python-wstool python-rosinstall
#apriltags stuff
sudo apt-get install libopencv-dev libeigen3-dev libv4l-dev

echo "\n -- Installing needed extra ROS packages for VIKI modules -- "
sudo apt-get install ros-jade-image-view ros-indigo-libuvc-camera ros-indigo-turtlesim ros-indigo-teleop-twist-keyboard 
cd catkin_ws
touch src/.rosinstall
wstool merge -t src src/viki_modules/core_modules/.rosinstall
rosinstall src

sudo chmod -R 777 src/viki_modules/core_modules
