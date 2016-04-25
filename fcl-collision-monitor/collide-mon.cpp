/*
 * collide-mon.cpp
 *
 *  Created on: Apr 25, 2016
 *      Author: wtarro
 */

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

#include "CollisionModel.h"
#include "model_ctl/ModelController.h"

using namespace gazebo;

std::shared_ptr<aslr::CollisionModel> model;

void cb(ConstModelStatePtr& msg) {
	model->SetPose({
		msg->rsp(), msg->rsr(), msg->rsy(), msg->rep(), msg->rwr(), msg->rwp(), msg->rwy(),
		msg->lsp(), msg->lsr(), msg->lsy(), msg->lep(), msg->lwr(), msg->lwp(), msg->lwy()
	});
	if (model->CheckSelfCollision())
		std::cout << "Self-collision occurred at " << msgs::Convert(msg->time()).Double() << std::endl;
}

int main(int argc, char** argv) {
	std::cout << "Initializing ..." << std::endl;
	if (!client::setup(argc, argv)) return 1;
	transport::NodePtr node(new transport::Node());
	node->Init("default");

	std::cout << "Creating collision model ..." << std::endl;
	model = std::make_shared<aslr::CollisionModel>();

	std::cout << "Subscribing to robot pose topic ..." << std::endl;
	transport::SubscriberPtr sub = node->Subscribe("~/model_pose", cb);
	for (;;) common::Time::MSleep(100);

	std::cout << "Shutting down ..." << std::endl;
	transport::fini();
	client::shutdown();
	return 0;
}

