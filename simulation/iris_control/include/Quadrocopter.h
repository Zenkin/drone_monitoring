#ifndef QUADROCOPTER_H_
#define QUADROCOPTER_H_

// This header defines the standard ROS classes
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
#include <boost/thread.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <iostream>

using namespace std;

class Quadrocopter {

public:

        Quadrocopter();
        ~Quadrocopter();
        void preFlight();
        bool setFlightMode(const char *str);
        bool arm();
        void takeoff(double attitude);
        void land();
        void moveToPoint(double x, double y, double z);
        void disarm();

        mavros_msgs::State currentState;

        

private:

        void checkFCUconnection();
        void stateListener(const mavros_msgs::State::ConstPtr& stateMsg);

        geometry_msgs::PoseStamped pose;
        mavros_msgs::SetMode offbSetMode;
        mavros_msgs::CommandBool armCmd;
        ros::Time last_request;

        // rate MUST be faster than 2Hz
        int rate = 20;
    
        // publishers
        ros::Publisher localPosPub;
        ros::Publisher cmdVelPub;
        
        // subscribers
        ros::Subscriber stateSub;
        //ros::Subscriber odomSub;
        
        // clients
        ros::ServiceClient armingClient;
        ros::ServiceClient setModeClient;

        // handle
        ros::NodeHandle nh;

};

#endif /* QUADROCOPTER_H_ */
