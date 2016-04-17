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
		//for (auto link : model->GetLinks()) link->SetKinematic(true); // would otherwise require tuning PID just for this
		node.reset(new gazebo::transport::Node());
		node->Init(model->GetWorld()->GetName());
		sub = node->Subscribe("~/model_cmd", &AslrReplPlugin::OnCmdRecv, this);
	}

	void OnCmdRecv(const std::string& cmd) {
		gzmsg << "Received command: " << cmd << std::endl;
		std::stringstream input(cmd);
		std::string action, motor; double target;
		input >> action;

		if (action.length() < 2) {
			gzerr << "Invalid command '" << action << "'" << std::endl;
			return;
		}
		if (action[0] < 'A') { action = action.substr(1, action.length() - 1); gzmsg << "Deleted leading control character." << std::endl; }
		if (action.length() == 4 && action == "seti") gzmsg << "Received problematic 'seti' command." << std::endl;

		if (action == "pose") {
			auto positions = control.GetAllJointPositions();
			gzmsg << "Robot Pose:" << std::endl << "\t";
			for (unsigned i = RSP; i < RHG; ++i) gzmsg << idToName[i] << " = " << positions[i] << ", ";
			gzmsg << std::endl << "\t";
			for (unsigned i = LSP; i < LHG; ++i) gzmsg << idToName[i] << " = " << positions[i] << ", ";
			gzmsg << std::endl << std::endl;
			return;
		}
		if (action != "move" && action != "tp" &&
			(action.length() != 4 || action[1] != 'e' || action[2] != 't' || (action[0] != 's' && action[0] != 'g') ||
				(action[3] != 'p' && action[3] != 'i' && action[3] != 'd' && action[3] != 'u' && action[3] != 'o' && action[3] != 'x' && action[3] != 'v'))) {
			gzerr << "Invalid command '" << action << "'" << std::endl;
			return;
		}

		input >> motor;
		auto motorIter = std::find(idToName.begin(), idToName.end(), motor);
		if (motorIter == idToName.end()) {
			gzerr << "Motor " << motor << " isn't a valid ID" << std::endl;
			return;
		}
		JointId motorIndex = (JointId) std::distance(idToName.begin(), motorIter);
		gzmsg << "Using motor id " << motorIndex << std::endl;
		if (action[0] != 'g') {
			input >> target;
			if (!std::isfinite(target)) {
				gzerr << "Target " << target << " isn't finite" << std::endl;
				return;
			}
			gzmsg << "Using target " << target << std::endl;
		}

		if (action == "move") {
			gzmsg << "Moving ..." << std::endl; control.MoveSingleJoint(motorIndex, target);
		} else if (action == "tp") {
			gzmsg << "Teleporting ..." << std::endl; control.TeleportSingleJoint(motorIndex, target);
		} else if (action[0] == 'g') {
			switch (action[3]) {
				case 'p': gzmsg << "Current P gain = " << control.GetJointGainP(motorIndex) << std::endl; break;
				case 'i': gzmsg << "Current I gain = " << control.GetJointGainI(motorIndex) << std::endl; break;
				case 'd': gzmsg << "Current D gain = " << control.GetJointGainD(motorIndex) << std::endl; break;
				case 'u': gzmsg << "Current I max = " << control.GetJointMaxI(motorIndex) << std::endl; break;
				case 'o': gzmsg << "Current I min = " << control.GetJointMinI(motorIndex) << std::endl; break;
				case 'x': gzmsg << "Current Cmd max = " << control.GetJointMaxCmd(motorIndex) << std::endl; break;
				case 'v': gzmsg << "Current Cmd min = " << control.GetJointMinCmd(motorIndex) << std::endl; break;
			}
		} else if (action[0] == 's') {
			switch (action[3]) {
				case 'p': gzmsg << "Set P gain = " << target << std::endl; control.SetJointGainP(motorIndex, target); break;
				case 'i': gzmsg << "Set I gain = " << target << std::endl; control.SetJointGainI(motorIndex, target); break;
				case 'd': gzmsg << "Set D gain = " << target << std::endl; control.SetJointGainD(motorIndex, target); break;
				case 'u': gzmsg << "Set I max = " << target << std::endl; control.SetJointMaxI(motorIndex, target); break;
				case 'o': gzmsg << "Set I min = " << target << std::endl; control.SetJointMinI(motorIndex, target); break;
				case 'x': gzmsg << "Set Cmd max = " << target << std::endl; control.SetJointMaxCmd(motorIndex, target); break;
				case 'v': gzmsg << "Set Cmd min = " << target << std::endl; control.SetJointMinCmd(motorIndex, target); break;
			}
		}
	}
};

GZ_REGISTER_MODEL_PLUGIN(AslrReplPlugin)
