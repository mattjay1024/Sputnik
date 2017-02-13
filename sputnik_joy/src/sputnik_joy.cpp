#include "sputnik_joy/sputnik_joy.h"

SputnikJoy::SputnikJoy() :
	linear(1),
	angular(0),
	deadman(4),
	//boost(0),
	lock(5),
	unlock(3),
	l_scale_max(1.2),
	l_scale_min(0.0),
	a_scale_max(1.2),
	a_scale_min(0.0),
	locked(false)
{
	node.param("linear_axis", linear, linear);
	node.param("angular_axis", angular, angular);
	node.param("deadman_switch", deadman, deadman);
	//node.param("boost_switch", boost, boost);
	node.param("lock_switch", lock, lock);
	node.param("l_scale_max", l_scale_max, l_scale_max);
	node.param("l_scale_min", l_scale_min, l_scale_min);
	node.param("a_scale_max", a_scale_max, a_scale_max);
	node.param("a_scale_min", a_scale_min, a_scale_min);

	motor_node = node.advertise<sputnik_motor::Response>("/sputnik/motor", 1);
	joy_node = node.subscribe<sensor_msgs::Joy>("joy", 10, &SputnikJoy::joyCallback, this);
}

void SputnikJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& Joy) {
	sputnik_motor::Response response;

	response.header.stamp = ros::Time::now();
	response.request_from = "joystick";
	response.urgency = 1.0;
	
	if(Joy->buttons.size() < 3 || Joy->buttons.empty()) {
		ROS_INFO("Too few buttons!, ignoring message");
		return;
	}
	
	if(Joy->buttons[deadman]) {
		if(Joy->buttons[lock]) {
			locked = true;
			linear_maintain = scaleLinear(Joy->axes[linear]);
		} else if(Joy->buttons[unlock]) {
			locked = false;
		}
		locked ? response.velocities.linear.x = linear_maintain : response.velocities.linear.x = scaleLinear(Joy->axes[linear]);
		response.velocities.angular.z = scaleAngular(Joy->axes[angular]);
	} else {
		response.velocities.linear.x = 0.0;
		response.velocities.angular.z = 0.0;
	}

	motor_node.publish(response);
}

double SputnikJoy::scaleLinear(double input) {
	return (((l_scale_max - l_scale_min) * (input - l_scale_min)) / (l_scale_max - l_scale_min)) + l_scale_min;
}

double SputnikJoy::scaleAngular(double input) {
	return (((a_scale_max - a_scale_min) * (input - a_scale_min)) / (a_scale_max - a_scale_min)) + a_scale_min;
}
