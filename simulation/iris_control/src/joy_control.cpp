/**
 * @file joy_control.cpp
 * @brief Joystick control
 */

#include <ros/ros.h>
// For local_pos_pub
#include <geometry_msgs/PoseStamped.h>
// For cmd_vel_pub
#include <geometry_msgs/TwistStamped.h>
// For arming_client
#include <mavros_msgs/CommandBool.h>
// For set_mode_client
#include <mavros_msgs/SetMode.h>
// For state_sub
#include <mavros_msgs/State.h>
// Joystick
#include <sensor_msgs/Joy.h>
// For odometry msg
#include <nav_msgs/Odometry.h>

// Set takeoff height, X and Y pose
const double DEFAULT_HEIGHT = 3;
const double DEFAULT_X_POSE = 0;
const double DEFAULT_Y_POSE = 0;

// Set default linear twist
const double DEFAULT_X_LINEAR_TWIST = 0;
const double DEFAULT_Y_LINEAR_TWIST = 0;
const double DEFAULT_Z_LINEAR_TWIST = 0;

using namespace std;

// Callback function for MAV state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback function for MAV pose
float current_position_x = 0, current_position_y = 0, current_position_z = 0;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position_x = msg->pose.pose.position.x; 
    current_position_y = msg->pose.pose.position.y;  
    current_position_z = msg->pose.pose.position.z;
}

// Joystick
double left_stick_x_axes = 0, right_stick_x_axes = 0, right_stick_y_axes = 0,
       back_panel_LB = 0, back_panel_RB = 0;
// Callback function for joy data
void joystickListener(const sensor_msgs::JoyConstPtr &joy_msg) {
    back_panel_LB = joy_msg->buttons[4];
    back_panel_RB = joy_msg->buttons[5];
    left_stick_x_axes = joy_msg->axes[0];
    right_stick_x_axes = joy_msg->axes[3];
    right_stick_y_axes = joy_msg->axes[4]; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber joystick_sub = nh.subscribe("/joy", 1, joystickListener);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, odom_cb);

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped cmd;
    cmd.twist.linear.x = DEFAULT_X_LINEAR_TWIST;
    cmd.twist.linear.y = DEFAULT_Y_LINEAR_TWIST;
    cmd.twist.angular.z = DEFAULT_Z_LINEAR_TWIST;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = DEFAULT_X_POSE;
    pose.pose.position.y = DEFAULT_Y_POSE;
    pose.pose.position.z = DEFAULT_HEIGHT;

    // Send a few setpoints before starting
    for(int i = 5; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Setting custom mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO("Custom mode was seted");

    ROS_INFO_STREAM("Takeoff... Target height: " << DEFAULT_HEIGHT);
    ros::Time last_request = ros::Time::now();
    while(abs(DEFAULT_HEIGHT - current_position_z) > 0.5){

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("Takeoff was complete Current height: " << current_position_z);

    double keep_position_x = current_position_x, 
           keep_position_y = current_position_y,
           keep_position_z = current_position_z;
    ROS_INFO("Start joy control");
 	last_request = ros::Time::now();
    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        if(left_stick_x_axes != 0 || right_stick_x_axes != 0 || right_stick_y_axes != 0 || back_panel_RB != 0 || back_panel_LB != 0) {
            cmd.twist.linear.x = right_stick_y_axes * 2;
            cmd.twist.linear.y = right_stick_x_axes * 2;
            cmd.twist.angular.z = left_stick_x_axes * 2;
            if (back_panel_RB != 0 ) cmd.twist.linear.z = 1;
            if (back_panel_LB != 0 ) cmd.twist.linear.z = -1;
            keep_position_x = current_position_x;
            keep_position_y = current_position_y;
            keep_position_z = current_position_z;
            cmd_vel_pub.publish(cmd);
        } 
        else {
            cmd.twist.linear.x = 4*(keep_position_x - current_position_x);
            cmd.twist.linear.y = 4*(keep_position_y - current_position_y);
            if(abs(cmd.twist.linear.y) > 0.3) cmd.twist.linear.y = cmd.twist.linear.y/abs(cmd.twist.linear.y) * 0.3;
            if(abs(cmd.twist.linear.x) > 0.3) cmd.twist.linear.x = cmd.twist.linear.x/abs(cmd.twist.linear.x) * 0.3;
            cmd.twist.linear.z = keep_position_z - current_position_z; 
            cmd_vel_pub.publish(cmd);

            //cout << keep_position_x << " " << keep_position_y << " " << keep_position_z << endl;

        }   

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

