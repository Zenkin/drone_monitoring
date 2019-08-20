/**
 * @file Quadrocopter.cpp
 * @brief Quadrocopter class
 */

#include "../include/Quadrocopter.h"

using namespace std;

Quadrocopter::Quadrocopter() {
}

Quadrocopter::~Quadrocopter() {
}


bool Quadrocopter::setFlightMode(const char *str) {
    ROS_INFO("Setting OFBOARD mode...");
    offbSetMode.request.custom_mode = str;
    if(currentState.mode != str){
        if(setModeClient.call(offbSetMode) && offbSetMode.response.mode_sent) {
            ROS_INFO("OFFBOARD enabled");
            return true;
        } else {
            ROS_INFO("ERROR: Can not set OFFBOARD mode");
            return false;
        }
    } else return true;
}

bool Quadrocopter::arm() {
    ROS_INFO("Arming...");
    armCmd.request.value = true;
    if(!currentState.armed){
        if(armingClient.call(armCmd) && armCmd.response.success) {
            ROS_INFO("Vehicle armed");
            return true;
        } else {
            ROS_INFO("ERROR: Can not arm");
            return false;
        }
    } else return true;
}

void Quadrocopter::stateListener(const mavros_msgs::State::ConstPtr &stateMsg) {
    currentState = *stateMsg;
}

void Quadrocopter::checkFCUconnection() {
    ros::Rate rate(20.0);
    while(ros::ok() && !currentState.connected){
        ROS_INFO_STREAM("Waiting FCU connection");
        ros::spinOnce();
        rate.sleep();
    }
}

void Quadrocopter::takeoff(double attitude) {
    ros::Rate rate(20);
    ROS_INFO_STREAM("Takeoff...");
    if(attitude < 2) {
        ROS_INFO_STREAM("Warning: minimal attitude is 2m. Set 2m");
        attitude = 2;
    }
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = attitude;
    last_request = ros::Time::now();
    for(int i = 100; ros::ok() && i > 0; --i){
        localPosPub.publish(pose);
        rate.sleep();
    }
}

void Quadrocopter::preFlight() {
    stateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &Quadrocopter::stateListener, this);
    //odomSub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, odom_cb);
    localPosPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    //cmdVelPub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");   
    ros::AsyncSpinner spinner(4);
    spinner.start();
}
