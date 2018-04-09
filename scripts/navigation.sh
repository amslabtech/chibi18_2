#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

gnome-terminal -e "bash roomba.sh"
gnome-terminal -e "bash system.sh"

sleep 2s
~
