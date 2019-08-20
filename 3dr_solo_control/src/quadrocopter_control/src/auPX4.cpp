#include "auPX4.h"

auPX4::auPX4(){
    ros::Rate rate(50);
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &auPX4::state_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &auPX4::imu_data_cb, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    gps_data_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, &auPX4::gps_data_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &auPX4::local_pos_cb, this);
    height_sub = nh.subscribe<sensor_msgs::Range>("/mavros/px4flow/ground_distance", 10, &auPX4::height_cb, this);
    joy_sub = nh.subscribe<quadrocopter_joystick_driver::joystick_data>("/joystick_data", 10, &auPX4::joy_cb, this);state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &auPX4::state_cb, this);state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &auPX4::state_cb, this);
    extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &auPX4::extended_state_cb, this);

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok() && !connect()){
        ROS_INFO("Trying to connect...");
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if(current_state.mode != "OFFBOARD"){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        } else {
            ROS_INFO("ERROR: Can not set OFFBOARD mode");
        }
    } else {
        ROS_INFO("OFFBOARD mode is already enabled");
    }
}

auPX4::~auPX4(){

}

bool auPX4::connect(){
	return current_state.connected;
}

void auPX4::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    extended_state = *msg;
}

void auPX4::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void auPX4::imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data_msg = *msg;
}

void auPX4::gps_data_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    is_gps_connected = true;
    gps_data_msg = *msg;
}

void auPX4::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos_msg = *msg;
}

void auPX4::height_cb(const sensor_msgs::Range::ConstPtr& msg){
    is_sensor_connected = true;
    height_msg = *msg;
}

void auPX4::joy_cb(const quadrocopter_joystick_driver::joystick_data::ConstPtr& msg){
    joy_msg = *msg;
}

void auPX4::control(int controlType){
    ros::Rate rate(1000);
    if(controlType == 1){
        while(controlType == 1 && ros::ok() && connect()){
            if(!joy_msg.is_pushed){
                cmdVel(0, 0, 0, 0);
            } else {
                cmdVel(joy_msg.right_stick_y_axes, joy_msg.right_stick_x_axes, joy_msg.left_stick_y_axes, joy_msg.left_stick_x_axes);
                if(joy_msg.a_button == 1 && !current_state.armed){
                    arm();
                    sleep(2);
                }
                if(joy_msg.x_button == 1 && extended_state.landed_state == 1){
                    takeoff(2.5);
                }
                if(joy_msg.y_button == 1 && extended_state.landed_state == 2){
                    land();
                    sleep(2);
                }
                if(joy_msg.b_button == 1 && current_state.armed){
                    emergency();
                    sleep(2);
                }
                if(joy_msg.RB == 1){
                    controlType = 2;
                    sleep(2);
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
}



auPX4::IMU auPX4::getIMU(){
    ros::spinOnce();
    imu_data.orientation.x = imu_data_msg.orientation.x;
    imu_data.orientation.y = imu_data_msg.orientation.y;
    imu_data.orientation.z = imu_data_msg.orientation.z;
    imu_data.orientation.w = imu_data_msg.orientation.w;
    imu_data.angular_velocity.x = imu_data_msg.angular_velocity.x;
    imu_data.angular_velocity.y = imu_data_msg.angular_velocity.y;
    imu_data.angular_velocity.z = imu_data_msg.angular_velocity.z;
    imu_data.linear_acceleration.x = imu_data_msg.linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_data_msg.linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_data_msg.linear_acceleration.z;
    return imu_data;
}

bool auPX4::takeoff(float altitude){
    ros::Rate rate(4.0);
    float error = 0.05;
    ros::spinOnce();
    if(!current_state.armed){
        ROS_INFO("ERROR: Vehicle isn't armed");
        return false;
    }
    if(is_sensor_connected){
        ROS_INFO("Trying to takeoff %fm", altitude);
        float diff = altitude - height_msg.range;
        while(diff < -error || diff > error && ros::ok() && connect()){
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = altitude + local_pos_msg.pose.position.z - height_msg.range;
            diff = altitude - height_msg.range;
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("takeoff DONE");
        return true;
    }else{
        ROS_INFO("ERROR: Sensor isn't connected, trying to use GPS");
        if(!is_gps_connected){
            ROS_INFO("ERROR: GPS isn't connected, can't takeoff")
            return false;
        }
        ROS_INFO("Trying to takeoff %fm", altitude);
        float diff = altitude - gps_data_msg.altitude;
        while(diff < -error || diff > error && ros::ok() && connect()){
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = altitude + local_pos_msg.pose.position.z - gps_data_msg.altitude;
            diff = altitude - gps_data_msg.altitude;
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("takeoff DONE");
        return true;
    }
    
}

bool auPX4::arm(){
    ROS_INFO("Arming...");
    arm_cmd.request.value = true;
    if(!current_state.armed){
        if(arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            return true;
        } else {
            ROS_INFO("ERROR: Can not arm");
            return false;
        }
    } else {
        ROS_INFO("Vehicle is already armed");
    }
    return true; 
}

bool auPX4::land(){
    try{
        ros::Rate rate(50.0);
        land_cmd.request.yaw = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.altitude = 0;
        ROS_INFO("Trying to land");
        while (!(land_client.call(land_cmd) && land_cmd.response.success)){
            ros::spinOnce();
            rate.sleep();
        }
        while(extended_state.landed_state != 1){
            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if(current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            } else {
                ROS_INFO("ERROR: Can not set OFFBOARD mode");
            }
        }
        ROS_INFO("Landing success");
        return true;
    }catch(...){
        return false;
    }
}

bool auPX4::emergency(){
    ROS_INFO("Trying to turning off engines...");
    arm_cmd.request.value = false;
    if(current_state.armed){
        if(arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Engines turned off");
            return true;
        } else {
            ROS_INFO("ERROR");
            return false;
        }
    } else {
        ROS_INFO("Engines already off");
    }
    return true;
}

void auPX4::cmdVel(float linearX, float linearY, float linearZ, float angularZ){
    cmd_vel_msg.linear.x = linearX;
    cmd_vel_msg.linear.y = linearY;
    cmd_vel_msg.linear.z = linearZ;
    cmd_vel_msg.angular.z = angularZ;
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();
}

auPX4::Vector3* auPX4::getOdometry(std::string sourceData){
    ros::spinOnce();
    if(sourceData == "GPS"){
        if(!is_gps_connected){
            ROS_INFO("ERROR: GPS isn't connected");
            return NULL;
        }
        odometry.x = gps_data_msg.latitude;
        odometry.y = gps_data_msg.longitude;
        odometry.z = gps_data_msg.altitude;
    }else if(sourceData == "local"){
        odometry.x = local_pos_msg.pose.position.x;
        odometry.y = local_pos_msg.pose.position.y;
        odometry.z = local_pos_msg.pose.position.z;
    }else{
        ROS_INFO("ERROR: wrong data source");
        return NULL;
    }
    return &odometry;
}

float auPX4::getHeight(){
    ros::spinOnce();
    if(!is_sensor_connected){
        ROS_INFO("ERROR: Sensor isn't connected, return 0");
        return 0;
    }
    return height_msg.range;
}

