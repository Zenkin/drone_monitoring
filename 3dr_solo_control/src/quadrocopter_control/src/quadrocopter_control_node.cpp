/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "auPX4.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quadrocopter_control_node");
    auPX4 solo3dr;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);
    solo3dr.getOdometry("GPS");
    solo3dr.control(1);
    //solo3dr.arm();
    //solo3dr.takeoff(4.0);
    /*while(ros::ok() && solo3dr.connect()){
        ROS_INFO("Current height: %f",solo3dr.getHeight());
        rate.sleep();
        //ros::spin();
    }*/
    //solo3dr.takeoff(3);
    //solo3dr.land();
    //solo3dr.emergency();
    //solo3dr.cmdVel(10,10,10,90);
    return 0;
}