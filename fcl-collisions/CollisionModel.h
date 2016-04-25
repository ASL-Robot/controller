/*
 * CollisionModel.h
 *
 *  Created on: Apr 25, 2016
 *      Author: wtarro
 */

#ifndef COLLISIONMODEL_H_
#define COLLISIONMODEL_H_

#include "RobotModel.h"
#include <unordered_map>
#include <unordered_set>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/narrowphase/narrowphase.h>

namespace aslr {

class CollisionModel : public RobotModel {
protected:
	// List of adjacent links which shouldn't be considered for collision
	std::unordered_map<fcl::Link*, std::unordered_set<fcl::Link*>> adjacencyMap;
	// list of collision objects for each link along with their base transforms from the link frame
	std::unordered_multimap<fcl::Link*, std::pair<fcl::CollisionObject*, fcl::Transform3f>> colliders;
	// collision objects in same link or (optionally) adjacent links won't collide with each other
	std::unordered_map<fcl::CollisionObject*, fcl::Link*> colliderOwners;

	inline void MakeCollider(boost::shared_ptr<fcl::Link> link, boost::shared_ptr<fcl::CollisionGeometry> geom,
					         fcl::Vec3f offset = fcl::Vec3f(), fcl::Vec3f rotAxis = fcl::Vec3f(), fcl::FCL_REAL rotAngle = 0);

	boost::shared_ptr<fcl::BroadPhaseCollisionManager> broadphase;
	static const fcl::CollisionRequest collisionRequest;
	fcl::CollisionResult collisionResult;
	bool BroadPhaseCollider(fcl::CollisionObject* o1, fcl::CollisionObject* o2);

	static bool BroadPhaseCollisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data) {
		return static_cast<CollisionModel*>(data)->BroadPhaseCollider(o1, o2);
	}
public:
	CollisionModel(bool useAdjacency = true);
	virtual ~CollisionModel();

	void UpdateCollisionFK();
	void SetPose(ModelPose pose, bool autoUpdateCollision = true);
	bool CheckSelfCollision();
};

}

#endif /* COLLISIONMODEL_H_ */
