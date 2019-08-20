#!/bin/bash

echo "Clearing Gazebo, PX4 and rosmaster..."
{
	killall gzserver gzclient rosmaster px4
} >> /dev/null

gnome-terminal --window-with-profile=sim --working-directory=/home/user/src/Firmware -x sudo make px4_sitl_default gazebo_iris_opt_flow no_sim=1
rosrun joy joy_node&

cd ~/src/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris_opt_flow.world&
sleep 3
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14540"&
#rosrun rqt_image_view rqt_image_view&

#gnome-terminal --window-with-profile=velocity -x rostopic echo /mavros/setpoint_velocity/cmd_vel/twist/linear -c&
#gnome-terminal --window-with-profile=position -x rostopic echo /mavros/local_position/odom/pose/pose/position -c&


