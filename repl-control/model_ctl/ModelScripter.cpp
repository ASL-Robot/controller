/*
 * ModelScripter.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#include "ModelScripter.h"

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

void ModelScripter::Init(gazebo::physics::ModelPtr parent) {
	model = parent;
	animateTrigger = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelScripter::AnimateFrame, this));
}

void ModelScripter::StartAnimation() {
	if (script.empty()) {
		gzwarn << "Tried to start animation with empty script!" << std::endl;
		return;
	}
	startTime = model->GetWorld()->GetSimTime();
	running = true;
}

void ModelScripter::AbortAnimation() {
	running = false;
	script.clear();
}

void ModelScripter::AnimateFrame() {
	if (!running || script.empty()) return;
	gazebo::common::Time curTime = model->GetWorld()->GetSimTime();
	gazebo::common::Time deltaT = curTime - startTime;
	if (deltaT <= gazebo::common::Time::Zero) return;

	do {
		Movement move = script.front();
		if (move.start > deltaT.Double()) break;
		gzmsg << "Starting movement of joint " << move.joint << " to " << move.target << " at " << move.start << " for " << move.duration << std::endl;
		controller.MoveJoint(move.joint, move.target, move.duration);
		script.pop_front();
	} while (!script.empty());
}

void ModelScripter::Load(std::list<Movement> moves) {
	AbortAnimation();
	script = moves;
}

