# Script for the creation of a catkin workspace

# Create a folder at your home (~) level named "catkin_ws/src"
mkdir -p ~/student-catkin-ws/src

# Change to the previously created "catkin_ws" directory
cd ~/student-catkin-ws/

# Create an empty catkin workspace that you can afterwards use to place your new ROS packages. 

catkin_make

# Upon execution of this command, two new folders are created, namely:
# build: This directory will keep the intermediate built files related to the new ROS packages
# devel: This directory will keep the binaries (libraries, executables), i.e. the targets of your project and a number of "setup" files that allow your ROS packages to be seen from your installed ROS system and vice versa.
# Finally there should also be a newly created CMakeLists.txt link file in the src/ folder. This link file will serve to build all new ROS packages that you may create in the future in the catkin workspace

