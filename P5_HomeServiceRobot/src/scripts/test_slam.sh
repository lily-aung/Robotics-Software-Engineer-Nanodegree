#!/bin/bash

cd $(pwd)/../..; catkin_make

xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/worlds/new_office_world.world" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
