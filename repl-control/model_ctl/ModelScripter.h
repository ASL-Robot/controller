/*
 * ModelScripter.h
 *
 *  Created on: Apr 21, 2016
 *      Author: wtarro
 */

#ifndef MODEL_CTL_MODELSCRIPTER_H_
#define MODEL_CTL_MODELSCRIPTER_H_

#include "ModelController.h"
#include <list>

using ModelScriptPtr = boost::shared_ptr<ModelScript>;
using ConstModelScriptPtr = const boost::shared_ptr<const ModelScript>;

class ModelScripter {
public:
	struct Movement { JointId joint; double target, start, duration; };
private:
	std::list<Movement> script;
	gazebo::common::Time startTime;
	bool running = false;

	gazebo::event::ConnectionPtr animateTrigger;
	gazebo::physics::ModelPtr model;
	ModelController& controller;
public:
	ModelScripter(ModelController& control) : controller(control) {}
	~ModelScripter() = default;

	void Init(gazebo::physics::ModelPtr model);

	void Load(std::list<Movement> moves);
	void StartAnimation();
	void AbortAnimation();
	void AnimateFrame();
};

#endif /* MODEL_CTL_MODELSCRIPTER_H_ */
