#ifndef _SPUTNIK_MODES_H_
#define _SPUTNIK_MODES_H_

#include <sstream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <map>
#include <ros/console.h>
#include "rapidxml/rapidxml.hpp"

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

struct Input {
	int base_weight;
	float weight, norm_weight, linear, rot;
};

struct InputSet {
	std::map<std::string, Input> inputs;
	float set_weight, set_linear, set_rot;
	void normalize();
	void weigh();

	bool operator < (const InputSet &rhs) const { return (set_weight < rhs.set_weight); };
	bool operator > (const InputSet &rhs) const { return (set_weight > rhs.set_weight); };
};

typedef std::vector<InputSet> Mode;

class ModeStore {
	public:
		ModeStore(std::string mode_directory);
		boost::optional<Mode> load(std::string name);
		// void save(const Mode &mode);
		// void saveAs(const Mode &mode, std::string name);
		std::string getCurrentMode() { return current_mode; };
	private:
		std::string current_mode;
		boost::filesystem::path base_directory;
};

#endif
