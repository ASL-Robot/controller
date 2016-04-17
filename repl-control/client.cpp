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

using namespace gazebo;

int main(int argc, char** argv) {
	if (!client::setup(argc, argv)) return 1;
	transport::NodePtr node(new transport::Node());
	node->Init("default");
	transport::PublisherPtr pub = node->Advertise<msgs::GzString>("~/model_cmd");

	std::string line = "";
	std::skipws(std::cin);
	if (!std::getline(std::cin, line))
		std::cerr << "Error reading command!" << std::endl;
	else do {
		std::cerr << "Sending command: " << line << std::endl;
		msgs::GzString msg; msg.set_data(line);
		pub->Publish(msg);
		if (!std::getline(std::cin, line)) {
			std::cerr << "Error reading command!" << std::endl;
			break;
		}
	} while (line != "exit");
	transport::fini();
	gazebo::client::shutdown();
	return 0;
}
