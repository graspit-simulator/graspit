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
// $Id: PQPCollision.h,v 1.6 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#ifndef _pqpcollision_h_
#define _pqpcollision_h_

#include <map>

#include "collisionInterface.h"

class Body;
class VCollide;

struct PQP_ContactResult;

/*! Implementation of the graspit collison interface using modified versions of the 
	PQP and VCollisde systems from UNC. See \a CollisionInterface class for interface 
	reference. The original versions of PQP and VCollide are copyright 1997 The 
	University of North Carolina at Chapel Hill. 
*/
class PQPCollision : public CollisionInterface
{
private:
	const static int MAXCOLLISIONS = 256;

	VCollide *mVc;
	std::map<const Body*, int> mIds;
	std::map<int, Body*> mBodies;

	inline int getId(const Body * body);
	inline Body* getBody(int id);
	bool getIdsFromList(const std::vector<Body*> *interestList, int **iList, int *iSize);

	void convertContactReport(PQP_ContactResult *pqpReport, ContactReport *report);

public:
	PQPCollision();
	~PQPCollision();

	//adding and moving bodies
	virtual bool addBody(Body *body, bool ExpectEmpty = false);
        virtual bool updateBodyGeometry(Body*, bool);
	virtual void cloneBody(Body *clone, const Body *original);
	virtual void setBodyTransform(Body *body, const transf &t);

	//enable / disable collision
	virtual void activateBody(const Body *body, bool active);
	virtual void activatePair(const Body *body1, const Body *body2, bool active);
	virtual bool isActive(const Body *body1, const Body *body2 = NULL);

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
	virtual void getBoundingVolumes(const Body* body, int depth, std::vector<BoundingBox> *bvs);

	//threading
	//!Also checks if threading is enabled inside VCollide
	virtual void newThread();
};

int PQPCollision::getId(const Body *body)
{
	std::map<const Body*, int>::const_iterator it = mIds.find(body);
	if ( it == mIds.end() ) return -1;
	return it->second;
}

Body* PQPCollision::getBody(int id)
{
	std::map<int, Body*>::iterator it = mBodies.find(id);
	if ( it == mBodies.end() ) return NULL;
	return it->second;
}

#endif
