/*
 * script.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

#include "model_ctl/ModelScripter.h"

using namespace gazebo;

int main(int argc, char** argv) {
	if (!client::setup(argc, argv)) return 1;
	transport::NodePtr node(new transport::Node());
	node->Init("default");
	transport::PublisherPtr pub = node->Advertise<ModelScript>("~/model_script");
	std::cout << "Waiting for subscriber connection ..." << std::endl;
	pub->WaitForConnection();
	std::cout << "Parsing script ... " << std::endl << "-------------------------------------------" << std::endl;

	ModelScript script;
	std::skipws(std::cin);
	for (;;) {
		std::string joint; double target, start, end;
		std::cin >> joint >> target >> start >> end;
		if (std::cin.eof()) break;
		if (!std::cin) {
			std::cerr << "Error reading command!" << std::endl;
			break;
		}

		auto jointIter = std::find(idToName.begin(), idToName.end(), joint);
		if (jointIter == idToName.end()) {
			std::cerr << "Motor " << joint << " isn't a valid ID" << std::endl;
			continue;
		}
		JointId jointIndex = (JointId) std::distance(idToName.begin(), jointIter);

		if (!std::isfinite(target)) {
			std::cerr << "Target " << target << " isn't finite" << std::endl;
			continue;
		}
		if (!std::isfinite(start) || start < 0) {
			std::cerr << "Start time " << start << " isn't finite and non-negative" << std::endl;
			continue;
		}
		if (!std::isfinite(end) || end < 0) {
			std::cerr << "End time " << end << " isn't finite and non-negative" << std::endl;
			continue;
		}
		if (start > end) {
			std::cerr << "End time " << end << " is before start time " << start << std::endl;
			continue;
		}

		ModelScriptAction* action = script.add_actions();
		action->set_joint(jointIndex);
		action->set_target(target);
		action->set_starttime(start);
		action->set_endtime(end);
		std::cout << "Moving joint " << jointIndex << " to " << target << " from " << start << " to " << end << std::endl;
	}

	std::cout << "-------------------------------------------" << std::endl << "Publishing script ..." << std::endl;
	pub->Publish(script, true);
	//pub->SendMessage();)
	transport::fini();
	client::shutdown();
	return 0;
}
