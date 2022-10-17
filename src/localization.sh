#! /bin/bash 
 
source /opt/ros/melodic/setup.bash  
roscore &

sleep 2 &

catkin_make
source devel/setup.bash 
roslaunch lio_sam localization.launch
