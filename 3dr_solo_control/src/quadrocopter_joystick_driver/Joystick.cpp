#include "Joystick.h"

using namespace std;

Joystick::Joystick() {
    print_header();
}

Joystick::~Joystick() {

}

void Joystick::get_joystick_data(const sensor_msgs::JoyConstPtr &msg) {
    left_stick.x_axes = msg->axes[0];
    left_stick.y_axes = msg->axes[1];
    right_stick.x_axes = msg->axes[3];
    right_stick.y_axes = msg->axes[4];
    back_panel.LT = msg->axes[2];
    back_panel.RT = msg->axes[5];
    back_panel.LB = msg->buttons[4];
    back_panel.RB = msg->buttons[5];
    buttons.a = msg->buttons[0];
    buttons.b = msg->buttons[1];
    buttons.y = msg->buttons[3];
    buttons.x = msg->buttons[2];
}

void Joystick::joystick_callback(const sensor_msgs::JoyConstPtr &msg) {
    check_connection();
    ros::Rate loop_rate(1000);
    get_joystick_data(msg);
    quadrocopter_joystick_driver::joystick_data message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/joystick_data";
    message.left_stick_x_axes = left_stick.x_axes;
    message.left_stick_y_axes = left_stick.y_axes;
    message.right_stick_x_axes = right_stick.x_axes;
    message.right_stick_y_axes = right_stick.y_axes;
    message.a_button = buttons.a;
    message.b_button = buttons.b;
    message.x_button = buttons.x;
    message.y_button = buttons.y;
    message.LT = back_panel.LT;
    message.RT = back_panel.RT;
    message.RB = back_panel.RB;
    message.LB = back_panel.LB;
    message.is_pushed = is_pushed();
    publish_message.publish(message);
    loop_rate.sleep();

}

bool Joystick::is_pushed() {
    if (left_stick.x_axes != 0) return true;
    if (left_stick.y_axes != 0) return true;
    if (right_stick.x_axes != 0) return true;
    if (right_stick.y_axes != 0) return true;
    if (buttons.a != 0) return true;
    if (buttons.b != 0) return true;
    if (buttons.x != 0) return true;
    if (buttons.y != 0) return true;
    return false;
}

void Joystick::run() {
    check_connection();
    ros::Subscriber sub = nh.subscribe(joystick_topic,
                                       queue_size,
                                       &Joystick::joystick_callback,
                                       this);
    ros::spin();
}


bool Joystick::is_joystick_connected() {
    if (joy_diagnostic_msg != NULL)
        if (joy_diagnostic_msg->status[0].message == OPEN_JOYSTICK_ERROR) {
            joystick_not_open = true;
            return false;
        } else {joystick_not_open = false; return true;}
    else {joystick_not_open = false; return false;}
}

void Joystick::check_connection() {
    publish_message = nh.advertise<quadrocopter_joystick_driver::joystick_data>(publish_joystick_data_topic,
                                                                                queue_size);
    while (!is_joystick_connected()) {
        joy_diagnostic_msg = ros::topic::waitForMessage
                             <diagnostic_msgs::DiagnosticArray>(diagnostic_topic,
                                                                ros::Duration(diagnostic_duration_time));
        if (joy_diagnostic_msg == NULL)
            ROS_INFO("No messages received");
        else {
            if (joystick_not_open) {
                if (joy_diagnostic_msg->status[0].message == "Joystick not open.") {
                    ROS_INFO_STREAM("STATUS: " << joy_diagnostic_msg->status[0].message
                                               << " Reconnection");
                } else ROS_INFO_STREAM("STATUS: " << joy_diagnostic_msg->status[0].message);
            }else{
                ROS_INFO("Connected");
            }
        }
    }
}

void Joystick::print_header() {
    cout << endl;
    cout << "             Joystick controller node\n" ;
    cout << "__________________________________________________________________\n\n";
    ROS_INFO_STREAM("Subscribe topics:\n\n"
    << "                     " << joystick_topic << "\n");
    ROS_INFO_STREAM("Publish topics:\n\n"
    << "                     /" << publish_joystick_data_topic << "\n"
    << "                     /" << diagnostic_topic << "\n");
    ROS_INFO_STREAM("queue_size: " << queue_size);
    ROS_INFO_STREAM("diagnostic_duration_time: " << diagnostic_duration_time << " sec");
    cout << "__________________________________________________________________\n\n";
}

