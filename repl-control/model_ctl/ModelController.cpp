/*
 * ModelController.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: wtarro
 */

#include "../model_ctl/ModelController.h"

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/gz_string.pb.h>

// Default PID parameters from gazebo::physics::JointController
//#define DEFAULT_PID() gazebo::common::PID(1, 0.1, 0.01, 1, -1, 1000, -1000)
// Altered parameters for tuning (Kp, Ki, Kd, iMax, iMin, cmdMax, cmdMin)
#define DEFAULT_PID() gazebo::common::PID(1, 0.1, 0, 1, -1, 1, -1)

bool ModelController::Init(gazebo::physics::ModelPtr loadedModel) {
	model = loadedModel;

#define ASSIGN_JOINT(_index, _name, _axis) \
	joints[_index] = { model->GetJoint(_name), _axis, DEFAULT_PID(), 0 }; \
	if (!joints[_index]) return false

	ASSIGN_JOINT(RSP, "j_Rshoulder_pitch", 0);
	ASSIGN_JOINT(RSR, "j_Rshoulder_roll" , 0);
	ASSIGN_JOINT(RSY, "j_Relbow"         , 0);
	ASSIGN_JOINT(REP, "j_Relbow"         , 1);
	ASSIGN_JOINT(RWR, "j_Rwrist"         , 0);
	ASSIGN_JOINT(RWP, "j_Rwrist"         , 1);
	ASSIGN_JOINT(RWY, "j_Rwrist_yaw"     , 0);
	ASSIGN_JOINT(LSP, "j_Lshoulder_pitch", 0);
	ASSIGN_JOINT(LSR, "j_Lshoulder_roll" , 0);
	ASSIGN_JOINT(LSY, "j_Lelbow"         , 0);
	ASSIGN_JOINT(LEP, "j_Lelbow"         , 1);
	ASSIGN_JOINT(LWR, "j_Lwrist"         , 0);
	ASSIGN_JOINT(LWP, "j_Lwrist"         , 1);
	ASSIGN_JOINT(LWY, "j_Lwrist_yaw"     , 0);

#undef ASSIGN_JOINT

	node.reset(new gazebo::transport::Node());
	node->Init(model->GetWorld()->GetName());
	posePub = node->Advertise<gazebo::msgs::GzString>("~/model_pose");

	pidTrigger = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController::ProcessPid, this));
	pubTrigger = gazebo::event::Events::ConnectWorldUpdateEnd(boost::bind(&ModelController::PublishPose, this));

	return true;
}

void ModelController::ProcessPid() {
	gazebo::common::Time curTime = model->GetWorld()->GetSimTime();
	gazebo::common::Time delta = curTime - prevUpdateTime;
	prevUpdateTime = curTime;
	if (delta <= gazebo::common::Time::Zero) return;

	for (auto joint : joints) if (joint) {
		double cmd = joint.pid.Update(joint->GetAngle(joint.axis).Radian() - joint.target, delta);
		joint->SetForce(joint.axis, cmd * joint->GetEffortLimit(joint.axis));
		//joint->SetPosition(joint.axis, joint.target); // freeze it in place for testing
	}
}

void ModelController::PublishPose() {

}

gazebo::math::Angle ModelController::GetSingleJointPosition(JointId id) { return joints[id]->GetAngle(joints[id].axis); }
std::array<gazebo::math::Angle, JointId::NUM_JOINTS> ModelController::GetAllJointPositions() {
	std::array<gazebo::math::Angle, JointId::NUM_JOINTS> ret;
	for (unsigned i = 0; i < JointId::NUM_JOINTS; ++i)
		if (joints[i]) ret[i] = joints[i]->GetAngle(joints[i].axis);
		else ret[i] = std::numeric_limits<double>::quiet_NaN();
	return ret;
}

void ModelController::MoveSingleJoint(JointId id, gazebo::math::Angle target) { joints[id].target = target.Radian(); joints[id].pid.Reset(); }
void ModelController::MoveAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets) {
	for (unsigned i = JointId::RSP; i < JointId::NUM_JOINTS; ++i)
		if (std::isfinite(targets[i].Radian())) MoveSingleJoint(JointId(i), targets[i]);
}

void ModelController::TeleportSingleJoint(JointId id, gazebo::math::Angle target) {
	joints[id]->SetPosition(joints[id].axis, joints[id].target);
	joints[id]->SetForce(joints[id].axis, 0);
	MoveSingleJoint(id, target);
}
void ModelController::TeleportAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets) {
	for (unsigned i = JointId::RSP; i < JointId::NUM_JOINTS; ++i)
		if (std::isfinite(targets[i].Radian())) TeleportSingleJoint(JointId(i), targets[i]);
}
