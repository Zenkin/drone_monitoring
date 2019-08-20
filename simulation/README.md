# Simulation PX4 + 3DR Iris + Gazebo

Bash script for setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).

## How to install:

    sudo ./full_installation.sh

**Installs:**
 - git
 - htop
 - SublimeText
 - tmux
 - ROS Kinetic (including Gazebo7)
 - GeographicLib 
 - MAVROS
 - common dependencie
 - FastRTPS
 - PX4/Firmware
 
**Remove:**
 - Modemmanager
 
**Upgrade:**
 - Cryptography
 
 ## How to run
 ### Simple run:
 
    sudo start_flight_sim.sh
 
 ### What's Happening Behind the Scenes:
 
    cd ~/src/Firmware
    sudo make px4_sitl_default gazebo_iris_opt_flow no_sim=1
    
In the new terminal:

    roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris_opt_flow.world
    
In the new terminal:

    roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14540"
    
## Test Flight:

    rosservice call /mavros/cmd/arming "value: true"; rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 180.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
    rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}" 
    
