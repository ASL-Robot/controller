/*
 * client.cpp
 *
 *  Created on: Apr 16, 2016
 *      Author: wtarro
 */

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

#include "model_ctl/ModelController.h"

using namespace gazebo;

int main(int argc, char** argv) {
	if (!client::setup(argc, argv)) return 1;
	transport::NodePtr node(new transport::Node());
	node->Init("default");
	transport::PublisherPtr pub = node->Advertise<ModelCmd>("~/model_cmd");
	pub->WaitForConnection();

	ModelCmd cmd;
	std::skipws(std::cin);
	for (;;) {
		std::string action, motor; double target, duration;
		std::cout << "cmd> "; std::cout.flush();
		std::cin >> action;
		if (!std::cin) {
			std::cerr << "Error reading command!" << std::endl;
			break;
		}
		if (action == "exit" || action == "quit") break;

		if (action == "pose") {
			std::cout << "Pose will be echoed to Gazebo's log." << std::endl;
			cmd.set_action(ModelCmdAction::ECHO_POSE);
		} else if (action == "reset") {
			std::cout << "Resetting robot's pose ..." << std::endl;
			cmd.set_action(ModelCmdAction::MODEL_RESET);
		} else if (action == "move" || action == "tp") {
			std::cin >> motor >> target;

			auto motorIter = std::find(idToName.begin(), idToName.end(), motor);
			if (motorIter == idToName.end()) {
				std::cerr << "Motor " << motor << " isn't a valid ID" << std::endl;
				continue;
			}
			JointId motorIndex = (JointId) std::distance(idToName.begin(), motorIter);
			cmd.set_joint(motorIndex);

			if (!std::isfinite(target)) {
				std::cerr << "Target " << target << " isn't finite" << std::endl;
				continue;
			}
			cmd.set_target(target);

			if (action == "move") {
				std::cin >> duration;
				if (!std::isfinite(duration) || duration <= 0) {
					std::cerr << "Duration " << duration << " isn't finite and positive" << std::endl;
					continue;
				}
				cmd.set_duration(duration);
			}

			std::cout << "Using motor id " << motorIndex << " and target " << target << ": ";
			if (action == "move") {
				std::cout << "Moving for " << duration << " ..." << std::endl; cmd.set_action(ModelCmdAction::JOINT_MOVE);
			} else if (action == "tp") {
				std::cout << "Teleporting ..." << std::endl; cmd.set_action(ModelCmdAction::JOINT_TELEPORT);
			}
		} else {
			std::cerr << "Invalid command '" << action << "'" << std::endl;
			continue;
		}

		pub->Publish(cmd);
	}

	transport::fini();
	client::shutdown();
	return 0;
}
