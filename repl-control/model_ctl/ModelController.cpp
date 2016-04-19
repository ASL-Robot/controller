/*
 * ModelController.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: wtarro
 */

#include "ModelController.h"

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

bool ModelController::Init(gazebo::physics::ModelPtr loadedModel) {
	model = loadedModel;
	state.set_allocated_time(new gazebo::msgs::Time);

#define ASSIGN_JOINT(_index, _protobuf_name, _name, _axis) \
	joints[_index] = (JointData) { &ModelState::_protobuf_name, &ModelState::set_##_protobuf_name, model->GetJoint(_name), _axis, 0.0, 0.0 }; \
	if (!joints[_index]) return false

	ASSIGN_JOINT(RSP, rsp, "j_Rshoulder_pitch", 0);
	ASSIGN_JOINT(RSR, rsr, "j_Rshoulder_roll" , 0);
	ASSIGN_JOINT(RSY, rsy, "j_Relbow"         , 0);
	ASSIGN_JOINT(REP, rep, "j_Relbow"         , 1);
	ASSIGN_JOINT(RWR, rwr, "j_Rwrist"         , 0);
	ASSIGN_JOINT(RWP, rwp, "j_Rwrist"         , 1);
	ASSIGN_JOINT(RWY, rwy, "j_Rwrist_yaw"     , 0);
	ASSIGN_JOINT(LSP, lsp, "j_Lshoulder_pitch", 0);
	ASSIGN_JOINT(LSR, lsr, "j_Lshoulder_roll" , 0);
	ASSIGN_JOINT(LSY, lsy, "j_Lelbow"         , 0);
	ASSIGN_JOINT(LEP, lep, "j_Lelbow"         , 1);
	ASSIGN_JOINT(LWR, lwr, "j_Lwrist"         , 0);
	ASSIGN_JOINT(LWP, lwp, "j_Lwrist"         , 1);
	ASSIGN_JOINT(LWY, lwy, "j_Lwrist_yaw"     , 0);

#undef ASSIGN_JOINT

	node.reset(new gazebo::transport::Node());
	node->Init(model->GetWorld()->GetName());
	posePub = node->Advertise<ModelState>("~/model_pose");
	pubTrigger = gazebo::event::Events::ConnectWorldUpdateEnd(boost::bind(&ModelController::PublishPose, this));
	moveTrigger = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController::SeekPose, this));

	return true;
}

void ModelController::SeekPose() {
	gazebo::common::Time curTime = model->GetWorld()->GetSimTime();
	gazebo::common::Time deltaT = curTime - prevUpdateTime;
	prevUpdateTime = curTime;
	if (deltaT <= gazebo::common::Time::Zero) return;

	for (auto joint : joints) if (joint) {
		double curPos = joint->GetAngle(joint.axis).Radian();
		if (curPos == joint.target) continue;
		double fullAngle = joint.target - curPos;
		double moveAngle = deltaT.Double() * joint.velocity;
		if (std::signbit(fullAngle) != std::signbit(moveAngle))
			moveAngle = std::copysign(moveAngle, fullAngle); // make velocity always towards target
		// pick delta that's closer to zero
		double deltaP = std::signbit(fullAngle) ? std::max(fullAngle, moveAngle) : std::min(fullAngle, moveAngle);
		joint->SetPosition(joint.axis, curPos + deltaP);
	}
}

void ModelController::PublishPose() {
	for (auto joint : joints) if (joint)
		(state.*joint.setFun)(joint->GetAngle(joint.axis).Radian());
	gazebo::common::Time simTime = model->GetWorld()->GetSimTime();
	gazebo::msgs::Set(state.mutable_time(), simTime);
	posePub->Publish(state);
}

gazebo::math::Angle ModelController::GetSingleJointPosition(JointId id) { return joints[id]->GetAngle(joints[id].axis); }
std::array<gazebo::math::Angle, JointId::NUM_JOINTS> ModelController::GetAllJointPositions() {
	std::array<gazebo::math::Angle, JointId::NUM_JOINTS> ret;
	for (unsigned i = 0; i < JointId::NUM_JOINTS; ++i)
		if (joints[i]) ret[i] = joints[i]->GetAngle(joints[i].axis);
		else ret[i] = std::numeric_limits<double>::quiet_NaN();
	return ret;
}

void ModelController::MoveJoint(JointId id, gazebo::math::Angle target, double duration) {
	double curPos = joints[id]->GetAngle(joints[id].axis).Radian();
	joints[id].target = target.Radian();
	joints[id].velocity = (joints[id].target - curPos) / duration;
}

void ModelController::MoveAllJoints(std::array<std::pair<gazebo::math::Angle, double>, JointId::NUM_JOINTS> targets) {
	for (unsigned i = JointId::RSP; i < JointId::NUM_JOINTS; ++i)
		if (std::isfinite(targets[i].first.Radian()) && std::isfinite(targets[i].second) && targets[i].second > 0.0)
			MoveJoint(JointId(i), targets[i].first, targets[i].second);
}

void ModelController::TeleportJoint(JointId id, gazebo::math::Angle target) {
	joints[id].target = target.Radian();
	joints[id].velocity = 0;
	joints[id]->SetPosition(joints[id].axis, joints[id].target);
}

void ModelController::TeleportAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets) {
	for (unsigned i = JointId::RSP; i < JointId::NUM_JOINTS; ++i)
		if (std::isfinite(targets[i].Radian())) TeleportJoint(JointId(i), targets[i]);
}
