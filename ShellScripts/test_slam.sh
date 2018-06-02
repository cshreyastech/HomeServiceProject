#!/bin/sh

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot homeservicerobot.launch world_file:=/home/nvidia/catkin_ws/src/HomeServiceProject/homeservicerobot/World/simple_world.world" &
sleep 5

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot slam_gmapping.launch" &
sleep 5

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot view_navigation.launch" &
sleep 5

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5
