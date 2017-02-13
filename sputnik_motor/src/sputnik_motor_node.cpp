#include <ros/ros.h>
#include "sputnik_motor/sputnik_motor.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "sputnik_motor");
	SputnikMotorEvaluator motors;
	
	if(motors.ok()) {
		ros::spin();
	} else {
		return -1;
	}
}

