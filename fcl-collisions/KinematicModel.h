/*
 * KinematicModel.h
 *
 *  Created on: Apr 25, 2016
 *      Author: wtarro
 */

#ifndef KINEMATICMODEL_H_
#define KINEMATICMODEL_H_

#include "RobotModel.h"

namespace aslr {

class KinematicModel : public RobotModel {
public:
	KinematicModel();
	virtual ~KinematicModel();
};

}

#endif /* KINEMATICMODEL_H_ */
