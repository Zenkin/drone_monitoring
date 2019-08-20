#ifndef AUPX4_H
#define AUPX4_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <quadrocopter_joystick_driver/joystick_data.h>
#include <mavros_msgs/ExtendedState.h>

class auPX4{
	struct Vector3{
		float x;
		float y;
		float z;
	};

	struct Quaternion{
		float x;
		float y;
		float z;
		float w;		
	};

	struct IMU{
		Quaternion orientation;
		Vector3 angular_velocity;
		Vector3 linear_acceleration;
	};

public:
	auPX4();
	~auPX4();
    bool connect();
    bool arm();
    bool land();
	bool takeoff(float altitude);
	bool emergency();
	IMU getIMU();
	void cmdVel(float linearX, float linearY, float linearZ, float angularZ);
    Vector3* getOdometry(std::string sourceData);
    float getHeight();
    void control(int controlType);
private:
	ros::NodeHandle nh;

	ros::Subscriber state_sub;
	mavros_msgs::State current_state;
	void state_cb(const mavros_msgs::State::ConstPtr& msg);

	ros::Subscriber imu_data_sub;
	sensor_msgs::Imu imu_data_msg;
	void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg);
    IMU imu_data;

    ros::Subscriber gps_data_sub;
	sensor_msgs::NavSatFix gps_data_msg;
	void gps_data_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
	Vector3 odometry;

	ros:: Subscriber local_pos_sub;
	geometry_msgs::PoseStamped local_pos_msg;
	void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

	ros::Subscriber height_sub;
	sensor_msgs::Range height_msg;
	void height_cb(const sensor_msgs::Range::ConstPtr& msg);

	ros::Subscriber joy_sub;
	quadrocopter_joystick_driver::joystick_data joy_msg;
	void joy_cb(const quadrocopter_joystick_driver::joystick_data::ConstPtr& msg);

	ros::Subscriber extended_state_sub;
	mavros_msgs::ExtendedState extended_state;
	void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);

    ros::Publisher local_pos_pub;
    geometry_msgs::PoseStamped pose;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    ros::ServiceClient arming_client;
    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient set_mode_client;
    mavros_msgs::SetMode offb_set_mode;

	ros::ServiceClient takeoff_client;
	mavros_msgs::CommandTOL takeoff_cmd;

	ros::ServiceClient land_client;
	mavros_msgs::CommandTOL land_cmd;
    
    bool is_gps_connected = false;
    bool is_sensor_connected = false;
};

#endif // AUPX4_H