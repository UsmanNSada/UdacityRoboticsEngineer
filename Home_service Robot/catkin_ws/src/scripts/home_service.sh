#!/bin/sh
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch rviz_config_file:=$(pwd)/../rvizConfig/home_service.rviz " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " rosrun pick_objects pick_objects" &
sleep 5
xterm -e " rosrun add_markers add_markers"

