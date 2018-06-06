#!/bin/sh

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot homeservicerobot.launch world_file:=/home/nvidia/catkin_ws/src/HomeServiceProject/homeservicerobot/World/simple_world.world" &
sleep 5

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot amcl_demo.launch" &
sleep 5

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; roslaunch homeservicerobot view_navigation.launch" &
sleep 10

xterm  -e  "source /home/nvidia/catkin_ws/devel/setup.bash; rosrun add_markers_with_pick_objects add_markers_with_pick_objects" &
