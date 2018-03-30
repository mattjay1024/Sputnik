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
	locked(false),
	mode(0)
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
	node.param("mode", mode, mode);

	switch(mode) {
		case 0: // direct mode
			motor_node = node.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
			joy_node = node.subscribe<sensor_msgs::Joy>("joy", 10, &SputnikJoy::joyCallbackDirect, this);
		break;
		case 1: // merged mode
			motor_node = node.advertise<sputnik_motor::Response>("/sputnik/motor", 1);
			joy_node = node.subscribe<sensor_msgs::Joy>("joy", 10, &SputnikJoy::joyCallbackMerged, this);
		break;
	}		
}

void SputnikJoy::joyCallbackMerged(const sensor_msgs::Joy::ConstPtr &Joy) { // use when combining with multiple inputs
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

void SputnikJoy::joyCallbackDirect(const sensor_msgs::Joy::ConstPtr &Joy) { // use when just operating by joystick
	geometry_msgs::Twist velocities;
	
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
		locked ? velocities.linear.x = linear_maintain : velocities.linear.x = scaleLinear(Joy->axes[linear]);
		velocities.angular.z = scaleAngular(Joy->axes[angular]);
	} else {
		velocities.linear.x = 0.0;
		velocities.angular.z = 0.0;
	}

	motor_node.publish(velocities);
}

double SputnikJoy::scaleLinear(double input) {
	return (((l_scale_max - l_scale_min) * (input - l_scale_min)) / (l_scale_max - l_scale_min)) + l_scale_min;
}

double SputnikJoy::scaleAngular(double input) {
	return (((a_scale_max - a_scale_min) * (input - a_scale_min)) / (a_scale_max - a_scale_min)) + a_scale_min;
}
