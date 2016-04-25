/*
 * CollisionModel.cpp
 *
 *  Created on: Apr 25, 2016
 *      Author: wtarro
 */

#include "CollisionModel.h"
#include <fcl/shape/geometric_shapes.h>

namespace aslr {

const fcl::CollisionRequest CollisionModel::collisionRequest;

inline void CollisionModel::MakeCollider(boost::shared_ptr<fcl::Link> link, boost::shared_ptr<fcl::CollisionGeometry> geom,
							             fcl::Vec3f offset, fcl::Vec3f rotAxis, fcl::FCL_REAL rotAngle) {
	boost::shared_ptr<fcl::CollisionObject> obj = boost::make_shared<fcl::CollisionObject>(geom);
	fcl::Quaternion3f quat; quat.fromAxisAngle(rotAxis, rotAngle);
	fcl::Transform3f tf { quat, offset };
	colliders.emplace(link.get(), std::make_pair(obj.get(), tf));
	link->addObject(obj);

	colliderOwners[obj.get()] = link.get();
	broadphase->registerObject(obj.get());
}

CollisionModel::CollisionModel(bool useAdjacency) : broadphase(boost::make_shared<fcl::DynamicAABBTreeCollisionManager>()) {
	// TODO Auto-generated constructor stub

	// Create all the collision objects for the model

	MakeCollider(links[LinkId::base], boost::make_shared<fcl::Cylinder>(11, 1), {0, 3.3315, 20.73},
				 {1, 0, 0}, boost::math::constants::half_pi<fcl::FCL_REAL>()); // head
	MakeCollider(links[LinkId::base], boost::make_shared<fcl::Box>(25, 15, 132.66), {0, 0, -70.83}); // torso
	MakeCollider(links[LinkId::base], boost::make_shared<fcl::Box>(40, 15, 9)); // collar

	MakeCollider(links[LinkId::Rarm_upper], boost::make_shared<fcl::Box>(4.296, 11.25, 18.802), {0, 0, -9.401}); // upper arm // TODO make lower half thinner?
	MakeCollider(links[LinkId::Rarm_upper], boost::make_shared<fcl::Box>(9.003, 11.25, 7.963), {-0.696, 0, -17.8205}); // shoulder yaw servo

	MakeCollider(links[LinkId::Larm_upper], boost::make_shared<fcl::Box>(4.296, 11.25, 18.802), {0, 0, -9.401}); // upper arm // TODO make lower half thinner?
	MakeCollider(links[LinkId::Larm_upper], boost::make_shared<fcl::Box>(9.003, 11.25, 7.963), {0.696, 0, -17.8205}); // shoulder yaw servo

	MakeCollider(links[LinkId::Rarm_lower], boost::make_shared<fcl::Box>(7.8, 4.332, 18.503), {0, 0, -9.2515}); // lower arm
	MakeCollider(links[LinkId::Rarm_lower], boost::make_shared<fcl::Box>(7.8, 8.818, 7.302), {0, 0.401, -17.852}); // wrist roll servo

	MakeCollider(links[LinkId::Larm_lower], boost::make_shared<fcl::Box>(7.8, 4.332, 18.503), {0, 0, -9.2515}); // lower arm
	MakeCollider(links[LinkId::Larm_lower], boost::make_shared<fcl::Box>(7.8, 8.818, 7.302), {0, 0.401, -17.852}); // wrist roll servo

	MakeCollider(links[LinkId::Rwrist_yaw], boost::make_shared<fcl::Box>(4.2, 3.4, 5.907), {0, 0, -2.9535}); // wrist yaw servo

	MakeCollider(links[LinkId::Lwrist_yaw], boost::make_shared<fcl::Box>(4.2, 3.4, 5.907), {0, 0, -2.9535}); // wrist yaw servo

	MakeCollider(links[LinkId::Rhand], boost::make_shared<fcl::Box>(3.404, 10.637, 12), {0, 0, -4.923}); // palm
	MakeCollider(links[LinkId::Rhand], boost::make_shared<fcl::Cylinder>(6, 10), {-3.1345, 3.8311, -8.5}); // cylinder of thumb movement
	MakeCollider(links[LinkId::Rhand], boost::make_shared<fcl::Box>(8.5, 11, 20), {-2.75, 0.5, -15}); // box of finger movement

	MakeCollider(links[LinkId::Lhand], boost::make_shared<fcl::Box>(3.404, 10.637, 12), {0, 0, -4.923}); // palm
	MakeCollider(links[LinkId::Lhand], boost::make_shared<fcl::Cylinder>(6, 10), {3.1345, 3.8311, -8.5}); // cylinder of thumb movement
	MakeCollider(links[LinkId::Lhand], boost::make_shared<fcl::Box>(8.5, 11, 20), {2.75, 0.5, -15}); // box of finger movement

	broadphase->setup();
	UpdateCollisionFK();

	if (useAdjacency) {
		for (unsigned j = JointId::RSP; j < JointId::RHG; ++j) {
			adjacencyMap[joints[j]->getParentLink().get()].insert(joints[j]->getChildLink().get());
			adjacencyMap[joints[j]->getChildLink().get()].insert(joints[j]->getParentLink().get());
		}

		// Manually add adjacency information for links separated by dummy links inside multi-axis joints

		adjacencyMap[links[LinkId::base      ].get()].insert(links[LinkId::Rarm_upper].get());
		adjacencyMap[links[LinkId::Rarm_upper].get()].insert(links[LinkId::base      ].get());
		adjacencyMap[links[LinkId::base      ].get()].insert(links[LinkId::Larm_upper].get());
		adjacencyMap[links[LinkId::Larm_upper].get()].insert(links[LinkId::base      ].get());

		adjacencyMap[links[LinkId::Rarm_upper].get()].insert(links[LinkId::Rarm_lower].get());
		adjacencyMap[links[LinkId::Rarm_lower].get()].insert(links[LinkId::Rarm_upper].get());
		adjacencyMap[links[LinkId::Larm_upper].get()].insert(links[LinkId::Larm_lower].get());
		adjacencyMap[links[LinkId::Larm_lower].get()].insert(links[LinkId::Larm_upper].get());

		adjacencyMap[links[LinkId::Rarm_lower].get()].insert(links[LinkId::Rwrist_yaw].get());
		adjacencyMap[links[LinkId::Rwrist_yaw].get()].insert(links[LinkId::Rarm_lower].get());
		adjacencyMap[links[LinkId::Larm_lower].get()].insert(links[LinkId::Lwrist_yaw].get());
		adjacencyMap[links[LinkId::Lwrist_yaw].get()].insert(links[LinkId::Larm_lower].get());
	}
}

CollisionModel::~CollisionModel() {
	// TODO Auto-generated destructor stub
}

void CollisionModel::SetPose(ModelPose pose, bool autoUpdateCollision) {
	for (unsigned j = JointId::RSP; j < JointId::RHG; ++j)
		joints[j]->getJointConfig()->getValue(0) = pose[j];
	if (autoUpdateCollision) UpdateCollisionFK();
}

void CollisionModel::UpdateCollisionFK() {
	fcl::Transform3f accumulator;
	for (unsigned j = JointId::RSP; j < JointId::LSP; ++j) {
		accumulator *= joints[j]->getLocalTransform();
		fcl::Link* link = joints[j]->getChildLink().get();
		for (auto bounds = colliders.equal_range(link); bounds.first != bounds.second; ++bounds.first)
			bounds.first->second.first->setTransform(accumulator * bounds.first->second.second);
	}
	accumulator.setIdentity();
	for (unsigned j = JointId::LSP; j < JointId::RHG; ++j) {
		accumulator *= joints[j]->getLocalTransform();
		fcl::Link* link = joints[j]->getChildLink().get();
		for (auto bounds = colliders.equal_range(link); bounds.first != bounds.second; ++bounds.first)
			bounds.first->second.first->setTransform(accumulator * bounds.first->second.second);
	}
	broadphase->update();
}

bool CollisionModel::BroadPhaseCollider(fcl::CollisionObject* o1, fcl::CollisionObject* o2) {
	// Stop if we already found a valid collision
	if (collisionRequest.isSatisfied(collisionResult)) return true;

	auto o1Link = colliderOwners.find(o1), o2Link = colliderOwners.find(o2);
	// Only filter on adjacency if it's really a self-collision check
	if (o1Link != colliderOwners.end() && o2Link != colliderOwners.end()) {
		fcl::Link *link1 = o1Link->second, *link2 = o2Link->second;
		// Ignore collisions inside a single link
		if (link1 == link2) return false;
		// Ignore collisions in adjacent links
		if (adjacencyMap[link1].find(link2) != adjacencyMap[link1].end()) return false;
	}
	return fcl::collide(o1, o2, collisionRequest, collisionResult) > 0;
}

bool CollisionModel::CheckSelfCollision() {
	collisionResult.clear();
	broadphase->collide(this, &BroadPhaseCollisionCallback);
	return collisionRequest.isSatisfied(collisionResult);
}

}
