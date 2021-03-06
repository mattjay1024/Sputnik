#include "sputnik_motor/sputnik_motor.h"

SputnikMotorEvaluator::SputnikMotorEvaluator() : 
	modes(m_dir)
{
	boost::optional<Mode> mode = modes.load("joystick");
	if(mode) {
		currentMode = *mode;
		output = node.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
		inputs = node.subscribe("/sputnik/motor", 100, &SputnikMotorEvaluator::responseCallback, this);
	} else {
		status = false;
		ROS_ERROR("Could not load 'joystick' mode");
	}
}

void SputnikMotorEvaluator::responseCallback(const sputnik_motor::Response::ConstPtr &response) {
	if (updateInput(response->request_from, response->urgency, response->velocities.linear.x, response->velocities.angular.z)) {
		std::sort(currentMode.begin(), currentMode.end());

		/*ROS_INFO("Request from '%s', urgency=%f, linear=%f, rot=%f [set_linear=%f, set_rot=%f]", 
			response->request_from.c_str(), 
			response->urgency,
			response->velocities.linear.x, 
			response->velocities.angular.z, 
			currentMode[0].set_linear, 
			currentMode[0].set_rot
		); */
	
		geometry_msgs::Twist movement;
		movement.linear.x = currentMode[0].set_linear;
		movement.angular.z = currentMode[0].set_rot;

		output.publish(movement);
		
		return;
	}

	ROS_WARN("Invalid input for current mode. [Mode: %s, Request: %s]", modes.getCurrentMode().c_str(), response->request_from.c_str()); 	
}

bool SputnikMotorEvaluator::updateInput(std::string input, float urgency, float linear, float rot) {
	bool found = false;

	for(auto &set : currentMode) {
		auto search = set.inputs.find(input);
		if(search != set.inputs.end()) {
			search->second.linear = linear;
			search->second.rot = rot;
			set.set_weight -= search->second.weight; // take out the weight of the updated input from the total
			search->second.weight = (search->second.base_weight * urgency); // recalculate the weight
			set.set_weight += search->second.weight; // add it back

			set.normalize(); // normalize the input weights
			set.weigh();
			found = true;
		}
	}

	return found;
}

