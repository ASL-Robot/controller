/*
 * CollisionModel.h
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

#include "common.h"

#include <fcl/articulated_model/joint.h>
#include <fcl/articulated_model/joint_config.h>
#include <fcl/articulated_model/link.h>
#include <fcl/articulated_model/model.h>
#include <fcl/articulated_model/model_config.h>

namespace aslr {

// Since this model isn't used for computing forces or torques:
// ALL DISTANCE MEASUREMENTS ARE IN CENTIMETERS FOR CONSISTENCY WITH
// DOCUMENTATION OF OTHER PROJECT PARTS (e.g. mechanical design)
class RobotModel {
protected:
	boost::shared_ptr<fcl::Model> model; // currently not terribly useful, but might be in the future
	std::array<boost::shared_ptr<fcl::Joint>, JointId::RHG> joints;
	std::array<boost::shared_ptr<fcl::Link>, LinkId::NUM_LINKS> links;
public:
	RobotModel();
	virtual ~RobotModel(); // virtual so sub-class destructors properly invoked
};

}

#endif /* ROBOTMODEL_H_ */
