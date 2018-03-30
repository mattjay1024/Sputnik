#ifndef _SPUTNIK_JOY_H_
#define _SPUTNIK_JOY_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sputnik_motor/Response.h>

class SputnikJoy {
	public:
		SputnikJoy();
		void joyCallbackMerged(const sensor_msgs::Joy::ConstPtr &Joy);
		void joyCallbackDirect(const sensor_msgs::Joy::ConstPtr &Joy);
	private:
		double scaleLinear(double input);
		double scaleAngular(double input);

		ros::NodeHandle node;

		int linear, angular, deadman, lock, unlock, mode;
		double l_scale_max, l_scale_min, linear_maintain;
		double a_scale_max, a_scale_min;
		bool locked;
		ros::Publisher motor_node;
		ros::Subscriber joy_node;
};

#endif
