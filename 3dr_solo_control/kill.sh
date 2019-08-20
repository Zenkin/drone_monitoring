#!/bin/bash

echo "Clearing Gazebo, PX4 and rosmaster..."
{
	killall gzserver gzclient rosmaster px4
} >> /dev/null
