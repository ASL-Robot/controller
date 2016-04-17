/*
 * ModelController.h
 *
 *  Created on: Apr 15, 2016
 *      Author: wtarro
 */

#ifndef MODELCONTROLLER_H_
#define MODELCONTROLLER_H_

#include <array>
#include <gazebo/math/Angle.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include "model_ctl.pb.h"

/*enum JointId : unsigned {
	// shoulder (pitch/roll/yaw), elbow (pitch), wrist (roll/pitch/yaw), hand gesture
	RSP, RSR, RSY, REP, RWR, RWP, RWY, RHG, // RHG is unused (model doesn't have fingers)
	LSP, LSR, LSY, LEP, LWR, LWP, LWY, LHG, // LHG is unused (model doesn't have fingers)
	NUM_JOINTS
};*/

static const std::array<std::string, JointId::NUM_JOINTS> idToName = {
		"RSP", "RSR", "RSY", "REP", "RWR", "RWP", "RWY", "RHG",
		"LSP", "LSR", "LSY", "LEP", "LWR", "LWP", "LWY", "LHG"
};

// Adapted from gazebo::physics::JointController to handle joints with multiple DoFs
class ModelController {
private:
	struct JointData {
		gazebo::physics::JointPtr ptr;
		unsigned                  axis;
		gazebo::common::PID       pid;
		double                    target;

		gazebo::physics::Joint* operator ->() const noexcept(noexcept(ptr.operator bool())) { return ptr.operator ->(); }
		explicit operator bool() const noexcept(noexcept(ptr.operator bool())) { return ptr.operator bool(); }
		bool operator !() const noexcept(noexcept(ptr.operator !())) { return ptr.operator !(); }
	};
	std::array<JointData, JointId::NUM_JOINTS> joints;
	gazebo::physics::ModelPtr model;
	gazebo::common::Time prevUpdateTime;

	gazebo::transport::NodePtr node;
	gazebo::transport::PublisherPtr posePub;
	gazebo::event::ConnectionPtr pidTrigger;
	gazebo::event::ConnectionPtr pubTrigger;

public:
	bool Init(gazebo::physics::ModelPtr loadedModel);
	void ProcessPid();
	void PublishPose();

	gazebo::math::Angle GetSingleJointPosition(JointId id);
	std::array<gazebo::math::Angle, JointId::NUM_JOINTS> GetAllJointPositions();
	void MoveSingleJoint(JointId id, gazebo::math::Angle target);
	void MoveAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets);
	void TeleportSingleJoint(JointId id, gazebo::math::Angle target);
	void TeleportAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets);
	double GetJointGainP(JointId id) { return joints[id].pid.GetPGain(); }
	void SetJointGainP(JointId id, double gain) { return joints[id].pid.SetPGain(gain); }
	double GetJointGainI(JointId id) { return joints[id].pid.GetIGain(); }
	void SetJointGainI(JointId id, double gain) { return joints[id].pid.SetIGain(gain); }
	double GetJointMaxI(JointId id) { return joints[id].pid.GetIMax(); }
	void SetJointMaxI(JointId id, double val) { return joints[id].pid.SetIMax(val); }
	double GetJointMinI(JointId id) { return joints[id].pid.GetIMin(); }
	void SetJointMinI(JointId id, double val) { return joints[id].pid.SetIMin(val); }
	double GetJointGainD(JointId id) { return joints[id].pid.GetDGain(); }
	void SetJointGainD(JointId id, double gain) { return joints[id].pid.SetDGain(gain); }
	double GetJointMaxCmd(JointId id) { return joints[id].pid.GetCmdMax(); }
	void SetJointMaxCmd(JointId id, double val) { return joints[id].pid.SetCmdMax(val); }
	double GetJointMinCmd(JointId id) { return joints[id].pid.GetCmdMin(); }
	void SetJointMinCmd(JointId id, double val) { return joints[id].pid.SetCmdMin(val); }
};

#endif /* MODELCONTROLLER_H_ */
