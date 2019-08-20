#include "Joystick.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_controll");
    Joystick joystick;
    joystick.run();

    return 0;
}
