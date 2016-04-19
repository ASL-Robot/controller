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
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include "model_ctl.pb.h"

using ModelStatePtr = boost::shared_ptr<ModelState>;
using ConstModelStatePtr = const boost::shared_ptr<const ModelState>;
using ModelCmdPtr = boost::shared_ptr<ModelCmd>;
using ConstModelCmdPtr = const boost::shared_ptr<const ModelCmd>;

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
		double (ModelState::* getFun)() const;
		void   (ModelState::* setFun)(double);
		gazebo::physics::JointPtr ptr;
		unsigned                  axis;
		double                    target;
		double                    velocity;

		gazebo::physics::Joint* operator ->() const noexcept(noexcept(ptr.operator bool())) { return ptr.operator ->(); }
		explicit operator bool() const noexcept(noexcept(ptr.operator bool())) { return ptr.operator bool(); }
		bool operator !() const noexcept(noexcept(ptr.operator !())) { return ptr.operator !(); }
	};
	std::array<JointData, JointId::NUM_JOINTS> joints;
	gazebo::physics::ModelPtr model;
	gazebo::common::Time prevUpdateTime;
	gazebo::event::ConnectionPtr moveTrigger;

	ModelState state;
	gazebo::transport::NodePtr node;
	gazebo::transport::PublisherPtr posePub;
	gazebo::event::ConnectionPtr pubTrigger;

public:
	bool Init(gazebo::physics::ModelPtr loadedModel);
	void SeekPose();
	void PublishPose();

	gazebo::math::Angle GetSingleJointPosition(JointId id);
	std::array<gazebo::math::Angle, JointId::NUM_JOINTS> GetAllJointPositions();
	void MoveJoint(JointId id, gazebo::math::Angle target, double duration);
	void MoveAllJoints(std::array<std::pair<gazebo::math::Angle, double>, JointId::NUM_JOINTS> targets);
	void TeleportJoint(JointId id, gazebo::math::Angle target);
	void TeleportAllJoints(std::array<gazebo::math::Angle, JointId::NUM_JOINTS> targets);
};

#endif /* MODELCONTROLLER_H_ */
