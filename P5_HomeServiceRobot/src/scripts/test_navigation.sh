#!/bin/sh
cd $(pwd)/../..; catkin_make

xterm  -e "source devel/setup.bash; export ROBOT_INITIAL_POSE='-x -5 -y -2 -z 0 -R 0 -P 0 -Y 0'; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/worlds/new_office_world.world" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/myMap.yaml" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &

