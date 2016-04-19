/*
 * plugin.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: wtarro
 */

#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "model_ctl/ModelController.h"

class AslrReplPlugin : public gazebo::ModelPlugin {
private:
	gazebo::physics::ModelPtr model;
	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr sub;
	ModelController control;

public:
	void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr element) {
		model = parent;
		if (!control.Init(parent)) {
			gzerr << "Couldn't load joints into controller!" << std::endl;
			return;
		}
		for (auto link : model->GetLinks()) link->SetKinematic(true); // would otherwise require tuning PID just for this
		node.reset(new gazebo::transport::Node());
		node->Init(model->GetWorld()->GetName());
		sub = node->Subscribe("~/model_cmd", &AslrReplPlugin::OnCmdRecv, this);
		gzmsg << "Finished loading AslrReplPlugin" << std::endl;
	}

	void OnCmdRecv(ConstModelCmdPtr& cmd) {
		if (cmd->action() == ModelCmdAction::ECHO_POSE) {
			auto positions = control.GetAllJointPositions();
			gzmsg << "Robot Pose:" << std::endl << "\t";
			for (unsigned i = RSP; i < RHG; ++i) gzmsg << "\b\b\b\b\b\b" << idToName[i] << " = " << positions[i] << ", ";
			gzmsg << "\b\b\b\b\b\b\b\b\033[0K" << std::endl << "\t";
			for (unsigned i = LSP; i < LHG; ++i) gzmsg << "\b\b\b\b\b\b" << idToName[i] << " = " << positions[i] << ", ";
			gzmsg << "\b\b\b\b\b\b\b\b\033[0K" << std::endl << std::endl;
			return;
		}

		if (!cmd->has_joint() || cmd->joint() >= JointId::NUM_JOINTS || !cmd->has_target() || !std::isfinite(cmd->target())) return;
		gzmsg << "Using motor id " << cmd->joint() << " and target " << cmd->target() << ": ";
		if (cmd->action() == ModelCmdAction::JOINT_MOVE) {
			if (!cmd->has_duration() || !std::isfinite(cmd->duration()) || cmd->duration() <= 0) return;
			gzmsg << "\b\b\b\b\b\b" << "Moving for " << cmd->duration() << " ..." << std::endl;
			control.MoveJoint(cmd->joint(), cmd->target(), cmd->duration());
		} else if (cmd->action() == ModelCmdAction::JOINT_TELEPORT) {
			gzmsg << "\b\b\b\b\b\b" << "Teleporting ..." << std::endl;
			control.TeleportJoint(cmd->joint(), cmd->target());
		}
	}
};

GZ_REGISTER_MODEL_PLUGIN(AslrReplPlugin)
