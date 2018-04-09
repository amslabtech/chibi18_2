#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch chibi18_control basis.launch"
