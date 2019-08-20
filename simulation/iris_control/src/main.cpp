#include "../include/Quadrocopter.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "quadrocopter_controller");
	Quadrocopter iris;
	// iris.checkMavrosServises(); ToDo
	iris.preFlight();
	while(!iris.setFlightMode("OFFBOARD"));
	while(!iris.arm());
	iris.takeoff(5);
	//iris.land(); ToDo

	return 0;

}