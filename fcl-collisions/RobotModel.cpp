/*
 * CollisionModel.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#include "RobotModel.h"

namespace aslr {

RobotModel::RobotModel() {
	// Create the model object
	model = boost::make_shared<fcl::Model>();

	// Create all the model links
#define MAKE_LINK(_index) links[LinkId::_index] = boost::make_shared<fcl::Link>(#_index); model->addLink(links[LinkId::_index])
	MAKE_LINK(base);
	MAKE_LINK(Rshoulder);
	model->initRoot({{"Rshoulder", "base"}}); // Can't manually set the root, so do it now when the least extra has to be done
	MAKE_LINK(Rarm_upper);
	MAKE_LINK(Relbow);
	MAKE_LINK(Rarm_lower);
	MAKE_LINK(Rwrist);
	MAKE_LINK(Rwrist_yaw);
	MAKE_LINK(Rhand);
	MAKE_LINK(Lshoulder);
	MAKE_LINK(Larm_upper);
	MAKE_LINK(Lelbow);
	MAKE_LINK(Larm_lower);
	MAKE_LINK(Lwrist);
	MAKE_LINK(Lwrist_yaw);
	MAKE_LINK(Lhand);
#undef MAKE_LINK

	// Create all the model joints
	// Params: jointId, parentLink, childLink, (x,y,z) from parent, (x,y,z) of rotation axis, min. angle deg, max. angle deg
#define MAKE_JOINT(_index, _pIndex, _cIndex, _T, _R, _loStop, _hiStop) \
	joints[JointId::_index] = boost::make_shared<fcl::RevoluteJoint>( \
			links[LinkId::_pIndex], links[LinkId::_cIndex], fcl::Vec3f _T, #_index, fcl::Vec3f _R); \
	joints[JointId::_index]->setJointConfig(boost::make_shared<fcl::JointConfig>(joints[JointId::_index], 0, \
			(_loStop) * boost::math::constants::degree<fcl::FCL_REAL>(), \
			(_hiStop) * boost::math::constants::degree<fcl::FCL_REAL>())); \
	links[LinkId::_pIndex]->addChildJoint(joints[JointId::_index]); \
	links[LinkId::_cIndex]->setParentJoint(joints[JointId::_index]); \
	model->addJoint(joints[JointId::_index])

	// TODO re-calculate upper angle limits for elbow joints for newest CAD model
	// TODO decide if wrist roll/pitch rotation should be at end of lower arm link or at beginning of wrist yaw link
	MAKE_JOINT(RSP, base      , Rshoulder , ( 25.717, 0,       0), ( 1,  0,  0), - 45, 135);
	MAKE_JOINT(RSR, Rshoulder , Rarm_upper, ( 25.717, 0,       0), ( 0, -1,  0),    0, 150);
	MAKE_JOINT(RSY, Rarm_upper, Relbow    , (      0, 0, -24.331), ( 0,  0,  1), - 45, 135);
	MAKE_JOINT(REP, Relbow    , Rarm_lower, (      0, 0,       0), ( 1,  0,  0),    0, 130); // TODO re-calculate upper angle limit
	MAKE_JOINT(RWR, Rarm_lower, Rwrist    , (      0, 0, -19.242), ( 0,  0,  1), - 90,  90); // TODO this makes wrist roll/pitch occur at end of lower arm link
	MAKE_JOINT(RWP, Rwrist    , Rwrist_yaw, (      0, 0,       0), ( 0, -1,  0), - 90,  90);
	MAKE_JOINT(RWY, Rwrist_yaw, Rhand     , (      0, 0, - 9.725), ( 1,  0,  0), - 90,  90);

	MAKE_JOINT(LSP, base      , Lshoulder , (-25.717, 0,       0), ( 1,  0,  0), - 45, 135);
	MAKE_JOINT(LSR, Lshoulder , Larm_upper, (-25.717, 0,       0), ( 0, -1,  0), -150,   0);
	MAKE_JOINT(LSY, Larm_upper, Lelbow    , (      0, 0, -24.331), ( 0,  0,  1), -135,  45);
	MAKE_JOINT(LEP, Lelbow    , Larm_lower, (      0, 0,       0), ( 1,  0,  0),    0, 130); // TODO re-calculate upper angle limit
	MAKE_JOINT(LWR, Larm_lower, Lwrist    , (      0, 0, -19.242), ( 0,  0,  1), - 90,  90); // TODO this makes wrist roll/pitch occur at end of lower arm link
	MAKE_JOINT(LWP, Lwrist    , Lwrist_yaw, (      0, 0,       0), ( 0, -1,  0), - 90,  90);
	MAKE_JOINT(LWY, Lwrist_yaw, Lhand     , (      0, 0, - 9.725), ( 1,  0,  0), - 90,  90);
#undef MAKE_JOINT
}

RobotModel::~RobotModel() {
	// TODO Auto-generated destructor stub
}

}

