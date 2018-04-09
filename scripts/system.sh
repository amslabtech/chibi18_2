#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc


gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch chibi18_control local_planner.launch"
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch chibi18_control global_planner.launch"
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch chibi18_control localizer.launch"
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch chibi18_control map.launch"

sleep 2s

