/*
 * common.h
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <string>
#include <array>
#include <memory>
#include <fcl/data_types.h>
#include <boost/smart_ptr.hpp> // only until FCL moves to C++11 types

namespace aslr {

enum JointId : unsigned {
	// shoulder (pitch/roll/yaw), elbow (pitch), wrist (roll/pitch/yaw)
	RSP, RSR, RSY, REP, RWR, RWP, RWY,
	LSP, LSR, LSY, LEP, LWR, LWP, LWY,
	RHG, LHG, // RHG and LHG are unused (model doesn't have fingers)
	NUM_JOINTS
};

static const std::array<std::string, JointId::NUM_JOINTS> idToName = {
		"RSP", "RSR", "RSY", "REP", "RWR", "RWP", "RWY",
		"LSP", "LSR", "LSY", "LEP", "LWR", "LWP", "LWY",
		"RHG", "LHG"
};

enum LinkId : unsigned {
	base, // head collision object belongs to base link
	Rshoulder, Rarm_upper, Relbow, Rarm_lower, Rwrist, Rwrist_yaw, Rhand,
	Lshoulder, Larm_upper, Lelbow, Larm_lower, Lwrist, Lwrist_yaw, Lhand,
	NUM_LINKS
};

using ArmPose = std::array<fcl::FCL_REAL, JointId::LSP>;
using ModelPose = std::array<fcl::FCL_REAL, JointId::RHG>;

}

#endif /* COMMON_H_ */
