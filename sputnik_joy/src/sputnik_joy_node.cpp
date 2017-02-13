#include "sputnik_joy/sputnik_joy.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "sputnik_joy");
	SputnikJoy controller;

	ros::spin();
}
