#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <quadrocopter_joystick_driver/joystick_data.h>

class Joystick {

    struct Left_stick {
        float x_axes;
        float y_axes;
    };

    struct Right_stick {
        float x_axes;
        float y_axes;
    };

    struct Back_panel {
        float LT, RT;
        int LB, RB;
    };

    struct Buttons {
        int y, x, a, b;
        int left, right, up, down;
    };

public:
    Joystick();
    ~Joystick();
    void run();

private:

    void joystick_callback(const sensor_msgs::JoyConstPtr &msg);
    void get_joystick_data(const sensor_msgs::JoyConstPtr& msg);
    bool is_pushed();
    bool is_joystick_connected();
    void check_connection();
    void print_header();

    ros::Publisher publish_message;
    ros::NodeHandle nh;
    Buttons buttons;
    Back_panel back_panel;
    Right_stick right_stick;
    Left_stick left_stick;
    boost::shared_ptr<diagnostic_msgs::DiagnosticArray const> joy_diagnostic_msg;
    char diagnostic_topic[12] = "diagnostics";
    char joystick_topic[5] = "/joy";
    char publish_joystick_data_topic[14] = "joystick_data";
    char OPEN_JOYSTICK_ERROR[19] = "Joystick not open.";
    int diagnostic_duration_time = 1;
    bool joystick_not_open = false;
    int queue_size = 1000;
};

#endif // JOYSTICK_H
