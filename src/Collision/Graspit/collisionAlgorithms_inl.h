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
// $Id: collisionAlgorithms_inl.h,v 1.4 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

/*! \file
	The implementations of the recursion callback functions that can be 
	inlined. Right now, all of these are virtual functions so can not be
	inlined, but in the future we might switch to a templated architecture.
*/

//#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(COLLISION_RECURSION);
PROF_DECLARE(CONTACT_LEAF);
PROF_DECLARE(CONTACT_QUICK);
PROF_DECLARE(COLLISION_LEAF);
PROF_DECLARE(COLLISION_QUICK);
PROF_DECLARE(DISTANCE_LEAF);
PROF_DECLARE(DISTANCE_QUICK);

namespace Collision {

//////////////////////////////////////////////////////////////////////////////
//                           Collision
//////////////////////////////////////////////////////////////////////////////

/*! Check if triangles collide. If they do, set reply to true */
void CollisionCallback::leafTest(const Leaf *l1, const Leaf *l2)
{
	PROF_TIMER_FUNC(COLLISION_LEAF);
	assert(l1); assert(l2);
	const std::list<Triangle>*	list1 = l1->getTriangles();
	const std::list<Triangle>*	list2 = l2->getTriangles();
	std::list<Triangle>::const_iterator it1, it2;
	for (it1=list1->begin(); it1!=list1->end() && !mCollision; it1++) {
		Triangle t1(*it1);
		//convert one triangle to the reference system of the other leaf
		t1.applyTransform(mTran1To2);
		for (it2 = list2->begin(); it2!=list2->end() && !mCollision; it2++) {
			mNumTriangleTests++;
			if ( triangleIntersection(t1, *it2) ) {
				mCollision = true;
			}
		}
	}
	mNumLeafTests++;
}

/*! Tests if boxes overlap. Returns -1 if they do, 1 if they don't */
double CollisionCallback::quickTest(const Node *n1, const Node *n2)
{
	PROF_TIMER_FUNC(COLLISION_QUICK);
	assert(n1); assert(n2);
	mNumQuickTests++;
	if(bboxOverlap(n1->getBox(), n2->getBox(), mTran2To1 )) {
		return -1.0;
	}
	return 1.0;
}

//////////////////////////////////////////////////////////////////////////////
//                           Contact
//////////////////////////////////////////////////////////////////////////////

/*! Distance between bounding boxes */
double ContactCallback::quickTest(const Node *n1, const Node *n2)
{
	PROF_TIMER_FUNC(CONTACT_QUICK);
	assert(n1); assert(n2);
	mNumQuickTests++;
	return bboxDistanceSq(n1->getBox(), n2->getBox(), mTran2To1);
}

/*! Check if triangles touch. Add contact to result set if they do */
void ContactCallback::leafTest(const Leaf *l1, const Leaf *l2)
{
	PROF_TIMER_FUNC(CONTACT_LEAF);
	assert(l1); assert(l2);
	const std::list<Triangle>*	list1 = l1->getTriangles();
	const std::list<Triangle>*	list2 = l2->getTriangles();
	std::list<Triangle>::const_iterator it1, it2;
	for (it1=list1->begin(); it1!=list1->end(); it1++) {
		Triangle t1(*it1);
		//convert one triangle to the reference system of the other
		t1.applyTransform(mTran1To2);
		for (it2 = list2->begin(); it2!=list2->end(); it2++) {
			mNumTriangleTests++;
			
			//this version adds ALL contacting points from the two triangles
			//no duplicate removal is done here; we rely on the higher levels
			//to do it for us. Should ofer more stable contact regions than
			//the version below
			std::vector< std::pair<position,position> > contactPoints;
			int numContacts = triangleTriangleContact(t1, *it2, mThreshold*mThreshold,
													  &contactPoints);
			if (numContacts < 0) {
				DBGA("Collision found when looking for contacts");
			}
			std::vector< std::pair<position,position> >::iterator it;
			for(it=contactPoints.begin(); it!=contactPoints.end(); it++) {
				position p1 = (*it).first;
				position p2 = (*it).second;
				//compute normals
				vec3 n1 = normalise(p1 - p2);
				vec3 n2 = normalise(p2 - p1);
				//convert one contact point back to the coordinate system of the leaf l1
				p1 = p1 * mTran2To1;
				n1 = n1 * mTran2To1;
				//add new contact to list. No check for duplication is performed
				mReport.push_back( ContactData( p1, p2, n1, n2, (p1-p2)%(p1-p2) ) );
			}
			
			/*
			//this version only adds the single closest contact point from two triangles
			//it also checks for duplicates when it inserts the contact into the list
			//however, it uses the same contact threshold as duplicate threshold as well.
			//in general, we want a different duplicate threshold, which is defined and
			//used at a higher level
			position p1,p2;
			//both positions are in the coordinate system of the leaf l2
			double distSq = triangleTriangleDistanceSq(t1, *it2, p1, p2);
			if (distSq < 0.0) {
				DBGP("Ccollision found when looking for contacts");				
			}else if (distSq < mThreshold * mThreshold) {
				//compute normals
				vec3 n1 = normalise(p1 - p2);
				vec3 n2 = normalise(p2 - p1);
				//convert one contact point back to the coordinate system of the leaf l1
				p1 = p1 * mTran2To1;
				n1 = n1 * mTran2To1;
				//we need to insert it and avoid duplication
				//we use the same threshold to check for duplicates that we use for
				//actual contact detection
				insertContactNoDuplicates(p1, p2, n1, n2, distSq, mThreshold);				
			}
			*/
		}
	}
	mNumLeafTests++;
}

//////////////////////////////////////////////////////////////////////////////
//                           Distance
//////////////////////////////////////////////////////////////////////////////

double DistanceCallback::quickTest(const Node *n1, const Node *n2)
{
	PROF_TIMER_FUNC(DISTANCE_QUICK);
	assert(n1); assert(n2);
	mNumQuickTests++;
	return bboxDistanceSq(n1->getBox(), n2->getBox(), mTran2To1);
}

void DistanceCallback::leafTest(const Leaf *l1, const Leaf *l2)
{
	PROF_TIMER_FUNC(DISTANCE_LEAF);
	mNumLeafTests++;
	assert(l1); assert(l2);
	const std::list<Triangle>*	list1 = l1->getTriangles();
	std::list<Triangle>::const_iterator it1, it2;
	for (it1=list1->begin(); it1!=list1->end() && mMinDistSq >= 0.0; it1++) {
		Triangle t1(*it1);
		t1.applyTransform(mTran1To2);
		//distance between two sets of triangles
		const std::list<Triangle>*	list2 = l2->getTriangles();
		for (it2 = list2->begin(); it2!=list2->end() && mMinDistSq >= 0.0; it2++) {
			mNumTriangleTests++;
			position p1,p2;
			//both points get computed in the coordinate system of leaf 2
			double distSq = triangleTriangleDistanceSq(t1, *it2, p1, p2);
			if (distSq < mMinDistSq) {
				p1 = p1 * mTran2To1;
				if (distSq >= 0.0) {
					mP1 = p1; mP2 = p2;
				} else {
					mP1 = mP2 = position(0,0,0);
				}	
				mMinDistSq = distSq;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
//                           Closest Point
//////////////////////////////////////////////////////////////////////////////

void 
ClosestPtCallback::leafTest(const Leaf *l1, const Leaf *l2)
{
	mNumLeafTests++;
	assert(l1); assert(!l2);
	const std::list<Triangle>*	list1 = l1->getTriangles();
	std::list<Triangle>::const_iterator it1;
	for (it1=list1->begin(); it1!=list1->end() && mMinDistSq >= 0.0; it1++) {
		mNumTriangleTests++;
		Triangle t1(*it1);
		//closest pt callback operates in body coordinates
		position p1 = closestPtTriangle(t1, mRefPosition);
		//distance sq from point to triangle
		double distSq = (p1 - mRefPosition).len_sq();//point triangle test
		if (distSq < mMinDistSq) {
			mMinDistSq = distSq;
			mClosestPt = p1;
		}
	}
}

double 
ClosestPtCallback::quickTest(const Node *n1, const Node *n2)
{
	assert(n1); assert(!n2);
	mNumQuickTests++;
	//closest pt callback operates in body coordinates
	return pointBoxDistanceSq(n1->getBox(), mRefPosition);
}

//////////////////////////////////////////////////////////////////////////////
//                           Region
//////////////////////////////////////////////////////////////////////////////

double 
RegionCallback::quickTest(const Node *n1, const Node *n2)
{
	assert(n1); assert(!n2);
	mNumQuickTests++;
	//the region callback operates in body coordinates
	return pointBoxDistanceSq(n1->getBox(), mRefPosition);
}

void 
RegionCallback::leafTest(const Leaf *l1, const Leaf *l2)
{
	mNumLeafTests++;
	assert(l1); assert(!l2);
	const std::list<Triangle>*	list1 = l1->getTriangles();
	std::list<Triangle>::const_iterator it1;
	for (it1=list1->begin(); it1!=list1->end(); it1++) {
		mNumTriangleTests++;
		//the region callback operates in body coordinates
		Triangle t1(*it1);
		//check normal first
		vec3 normal = t1.normal();
		//remember that contact normal points inwards
		if (normal % mNormal > 0) continue;
		position p1 = closestPtTriangle(t1, mRefPosition);
		double distSq = (p1 - mRefPosition).len_sq();
		if (distSq > mRadiusSq) continue;
		insertPoint(t1.v1);
		insertPoint(t1.v2);
		insertPoint(t1.v3);
	}
}

} //namespace Collision
