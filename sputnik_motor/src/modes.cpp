#include "sputnik_motor/modes.h"

ModeStore::ModeStore(std::string mode_directory) :
	base_directory(mode_directory)
{
	namespace fs = boost::filesystem;
	if(!exists(base_directory)) {
		fs::create_directory(base_directory);
	}
}

boost::optional<Mode> ModeStore::load(std::string name) {
	namespace rx = rapidxml;
	namespace fs = boost::filesystem;

	std::string filename = name + ".mode";
	fs::path f(base_directory / filename);

	if(fs::exists(f)) {
		ROS_INFO("Loading %s", f.string().c_str());
	
		std::ifstream file(f.string());
		rx::xml_document<char> doc;
		rx::xml_node<> *root;
		Mode mode;
		int size = fs::file_size(f);

		if(size > 0) {
			char *contents = doc.allocate_string("", size);
			file.read(contents, size);
			doc.parse<0>(contents);
		} else {
			return boost::optional<Mode>();
		}

		file.close();
		
		if((root = doc.first_node("mode"))) {
			current_mode = name;
			for(rx::xml_node<> *it_set = root->first_node("set"); it_set; it_set = it_set->next_sibling("set")) {
				ROS_INFO("Descending into new set");
				
				InputSet set;
				set.set_weight = 0.0;
				set.set_linear = 0.0;
				set.set_rot = 0.0;
				
				for(rx::xml_node<> *it_input = it_set->first_node("input"); it_input; it_input = it_input->next_sibling("input")) { 				

					Input input;
					input.base_weight = boost::lexical_cast<int>(it_input->first_attribute("weight")->value());
					input.weight = 0.0;
					input.norm_weight = 0.0;				
					input.linear = 0.0;
					input.rot = 0.0;
					set.inputs.insert({it_input->first_attribute("name")->value(), input});

					ROS_INFO("Found input '%s', base_weight=%d", it_input->first_attribute("name")->value(), input.base_weight);
				}
				
				mode.push_back(set);
			}
			return boost::optional<Mode>(mode);
		} else {
			ROS_ERROR("Errors occurred parsing %s. Did not load.", filename.c_str());
		}
	} else {
		ROS_ERROR("File not found: %s in %s", filename.c_str(), base_directory.string().c_str());
	}
	
	return boost::optional<Mode>();
}		

void InputSet::normalize() {
	for(auto &input : inputs) { // rejigger all the weights based off the new one that was calculated
		input.second.norm_weight = input.second.weight / set_weight;
	} 
}

void InputSet::weigh() {
	set_linear = 0.0;
	set_rot = 0.0;

	for(auto &input : inputs) { // calculate the set's desired linear motion, and it's rotational motion
		set_linear += (input.second.norm_weight * input.second.linear);
		set_rot += (input.second.norm_weight * input.second.rot);
	}
}		

