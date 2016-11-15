//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: graspitCollision.h,v 1.6 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#ifndef _graspitcollision_h_
#define _graspitcollision_h_

/*! \file
	Implementation of the collision detection interface. This collision
	engine was written from scratch as part of GraspIt.
*/

#include <map>
#include <set>
#include <list>
#include <vector>

#include "collisionInterface.h"

namespace Collision{
	class CollisionModel;
}
using namespace Collision;

class GraspitCollision : public CollisionInterface
{
private:

	//! Stores the pointers to collision models sorted in ascending order of pointer comparisons
	std::vector<CollisionModel*> mModels;

	//! Maps GraspIt Body pointers to internal collision model pointers
	std::map<const Body*, CollisionModel*> mModelMap;
	//! Maps internal collision model pointers to GraspIt Body pointers
	std::map<const CollisionModel*, Body*> mBodyMap;

	inline Body* getBody(const CollisionModel* model);
	inline CollisionModel* getModel(const Body* body);

	typedef std::multimap<const CollisionModel*, 
						  const CollisionModel*>::iterator DisabledIterator;
	//!Stores all the disabled pairs of bodies
	/*! When inserting a new pair in here, always make sure that the *key* pointer and the 
		value pointer compare such that key < value. This allows us to know, for any two
		pointers, which is supposed to be the key for indexing here. It also allows for more
		efficient traversal of the complete list of bodies */
	std::multimap<const CollisionModel*, const CollisionModel*> mDisabledMap;

	typedef std::pair<const CollisionModel*, const CollisionModel*> CollisionPair;
	//! Returns all the pairs of bodies that collision should be cheked for
	void getActivePairs(std::list<CollisionPair> *activePairs, 
						const std::set<CollisionModel*> *interestList);

	void convertInterestList(const std::vector<Body*> *inList, std::set<CollisionModel*> *outSet);

public:
	GraspitCollision(){}
	virtual ~GraspitCollision(){}

	//adding and moving bodies
	virtual bool addBody(Body *body, bool ExpectEmpty = false);
        virtual bool updateBodyGeometry(Body* body, bool ExpectEmpty = false);
	virtual void removeBody(Body *body);
	virtual void cloneBody(Body*, const Body*);
	virtual void setBodyTransform(Body *body, const transf &t);

	//enable / disable collision
	virtual void activateBody(const Body*, bool);
	virtual void activatePair(const Body*, const Body*, bool);
	virtual bool isActive(const Body*, const Body*);

	//collision detection
	virtual int allCollisions(DetectionType type, CollisionReport *report, 
							  const std::vector<Body*> *interestList);

	//contact
	virtual int allContacts(CollisionReport *report, double threshold, 
							const std::vector<Body*> *interestList);
	virtual int contact(ContactReport *report, double threshold, 
							const Body *body1, const Body *body2);

	//point-to-body and body-to-body distances
	virtual double pointToBodyDistance(const Body *body1, position point,
									   position &closestPoint, vec3 &closestNormal);
	virtual double bodyToBodyDistance(const Body *body1, const Body *body2,
									  position &p1, position &p2);

	//region on a body around a point
	virtual void bodyRegion(const Body *body, position point, vec3 normal, 
							double radius, Neighborhood *neighborhood);

	//show bounding box hierarchy
	virtual void getBoundingVolumes(const Body*, int, std::vector<BoundingBox>*);

	//threading
	virtual void newThread(){}
	virtual int getThread(){return 0;}
};

Body* 
GraspitCollision::getBody(const CollisionModel* model)
{
	std::map<const CollisionModel*, Body*>::const_iterator it = mBodyMap.find(model);
	if (it==mBodyMap.end()) return NULL;
	return (*it).second;
}

CollisionModel*
GraspitCollision::getModel(const Body* body)
{
	std::map<const Body*, CollisionModel*>::const_iterator it = mModelMap.find(body);
	if (it==mModelMap.end()) return NULL;
	return (*it).second;
}

#endif
