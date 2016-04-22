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
#include "model_ctl/ModelScripter.h"

class AslrReplPlugin : public gazebo::ModelPlugin {
private:
	gazebo::physics::ModelPtr model;
	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr cmdSub;
	gazebo::transport::SubscriberPtr scriptSub;
	ModelController control;
	ModelScripter scripter = ModelScripter(control);

public:
	void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr element) {
		model = parent;
		if (!control.Init(parent)) {
			gzerr << "Couldn't load joints into controller!" << std::endl;
			return;
		}
		scripter.Init(parent);
		for (auto link : model->GetLinks()) link->SetKinematic(true); // would otherwise require tuning PID just for this
		node.reset(new gazebo::transport::Node());
		node->Init(model->GetWorld()->GetName());
		cmdSub = node->Subscribe("~/model_cmd", &AslrReplPlugin::OnCmdRecv, this);
		scriptSub = node->Subscribe("~/model_script", &AslrReplPlugin::OnScriptRecv, this);
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
		if (cmd->action() == ModelCmdAction::MODEL_RESET) {
			gzmsg << "Resetting robot's pose ..." << std::endl;
			std::array<gazebo::math::Angle, JointId::NUM_JOINTS> arr;
			arr.fill(0);
			control.TeleportAllJoints(arr);
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

	void OnScriptRecv(ConstModelScriptPtr& script) {
		std::list<ModelScripter::Movement> moves;
		gzmsg << "Received script containing " << script->actions_size() << " moves ..." << std::endl;
		for (int i = 0; i < script->actions_size(); ++i) {
			const ModelScriptAction& action = script->actions(i);
			ModelScripter::Movement move = { action.joint(), action.target(), action.starttime(), action.endtime() - action.starttime() };
			gzmsg << "Scripted movement of joint " << move.joint << " to " << move.target << " at " << move.start << " for " << move.duration << std::endl;
			moves.push_back(move);
		}
		moves.sort([](const ModelScripter::Movement& a, const ModelScripter::Movement& b) { return a.start < b.start; });
		scripter.Load(moves);
		scripter.StartAnimation();
	}
};

GZ_REGISTER_MODEL_PLUGIN(AslrReplPlugin)
