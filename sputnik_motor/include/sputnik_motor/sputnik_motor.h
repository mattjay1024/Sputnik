#ifndef _SPUTNIK_MOTOR_H_
#define _SPUTNIK_MOTOR_H_

#include <ros/ros.h>
#include <string>
#include <vector>

#include "sputnik_motor/modes.h"

#include <geometry_msgs/Twist.h>
#include <sputnik_motor/Response.h>

const std::string m_dir = "/home/matt/sputnik_modes";

class SputnikMotorEvaluator {
	public:
		SputnikMotorEvaluator();
		void responseCallback(const sputnik_motor::Response::ConstPtr &response);
		bool ok() { return status; };
	private:
		Mode currentMode;
		
		ModeStore modes;
		ros::NodeHandle node;
		ros::Publisher output;
		ros::Subscriber inputs;

		bool status;

		bool updateInput(std::string input, float urgency, float linear, float rot);
};

#endif
