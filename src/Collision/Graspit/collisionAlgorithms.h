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
// $Id: collisionAlgorithms.h,v 1.6 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _collisionalgorithms_h_
#define _collisionalgorithms_h_

/*! \file
	Contains the actual collision detection mechanisms. This file implements
	the recursive calls and algorithms; most of the primitive interesection 
	and geometry routines are in the BoundingBox and Triangle classes and 
	files.

	There are 5 types of collision queries:
	<ul>
	<li> collision - answers whether two bodies are interpenetrating or not.
	If they are, we don't care where exactly.
	<li> contact - returns all the points where two bodies are separated by
	less than a contact threshold.
	<li> distance - computes the distance between two bodies and also returns
	the points on the bodies which are closest to each other. If the bodies 
	are colliding it returns -1.
	<li> closest point - computes the distance between a point and a body
	<li> region - returns all the points on a body which are within a 
	given threshold of a reference point.
	</ul>

	All collision detection tests are implemented as recursive calls, using
	the same recursion mechanism. The recursion is adapted to each type of
	query using a recursionCallback, which must implement three tests. At each
	iteration, the recursion will be called on two nodes. It will call the 
	following tests from its instance of the recursionCallback:
	<ul>
	<li> if both are leaves, it will call the \a leafTest and return
	<li> if not, we must decide which branch we split at this recursion,
	and in which order we recurse on its children. For now, we always split
	the branch whith larger bbox volume. Then we must decide if, and in which
	order, we process its children.
	<li> the \a quickTest computes a measure of how much we want to recurse
	on a particular child. Intuitively, is a child is far from the other node,
	we are less inclined to recurse on it.
	<li> the \a distanceTest takes in the result of the quickTest and decides
	if we actually want to recurse on a particular child at all.
	</li>
	See implementations of the recursion function and the recursionCallback 
	interface for details.
*/
#include <vector>

#include "mytools.h"
#include "collisionStructures.h"

namespace Collision {

class Node;
class Leaf;
class Branch;
class CollisionModel;

class RecursionCallback
{
protected:
	const CollisionModel *mModel1;
	const CollisionModel *mModel2;
	//! Precomputed transform from model 2 to model 1
	transf mTran2To1;
	//! Precomputed transform from model 1 to model 2
	transf mTran1To2;

	int mNumLeafTests, mNumTriangleTests, mNumQuickTests;
public:
	RecursionCallback(const CollisionModel *m1, const CollisionModel *m2);
	virtual void reset() {
		mNumLeafTests = 0;
		mNumTriangleTests = 0;
		mNumQuickTests = 0;
	}
	virtual ~RecursionCallback(){}
	virtual void leafTest(const Leaf *l1, const Leaf *l2) = 0;
	virtual double quickTest(const Node *n1, const Node *n2) = 0;
	virtual bool distanceTest(double d) = 0;
	virtual void printStatistics();
};

//! The entry point to the one and only recursion mechanism
void startRecursion(const CollisionModel *model1, const CollisionModel *model2, 
					RecursionCallback *rc);

/*! Recursion callback for the collision test.
	<ul>
	<li> leaf test - checks collision between leaf triangles
	<li> quick test - boolean check if the bounding boxes overlap
	<li> distance test - if bboxes don't overlap, no need to recurse.
	Also, if we already have a collision, no point in recursing 
	anywhere; just return. 
	</ul>
*/
class CollisionCallback : public RecursionCallback
{
private:
	bool mCollision;
public:
	CollisionCallback(const CollisionModel *m1, const CollisionModel *m2) : 
					  RecursionCallback(m1,m2){reset();}
	void reset(){
		RecursionCallback::reset();
		mCollision = false;
	}
	bool isCollision() const {return mCollision;}
	virtual void leafTest(const Leaf *l1, const Leaf *l2);
	virtual double quickTest(const Node *n1, const Node *n2);
	/*! Only recurse if the bboxes overlap, and we don't already have a
		detected collision. */
	virtual bool distanceTest(double d) {
		if (mCollision) return false;
		if (d<0) return true;
		return false;
	}
	void printStatistics();
};

/*! Recursion callback for the contact test.
	<ul>
	<li> leaf test - checks for contact points between triangles
	<li> quick test - returns the distance between bounding boxes
	<li> distance test - if distance between bounding boxes is more
	than contact threshold, no need to recurse.
	</ul>
*/
class ContactCallback : public RecursionCallback
{
private:
	double mThreshold;
	ContactReport mReport;

	void insertContactNoDuplicates(const position &p1, const position &p2, 
								   const vec3 &n1, const vec3 &n2, 
								   double distSq, double thresh);
public:
	ContactCallback(double threshold, const CollisionModel *m1, const CollisionModel *m2) : 
	                RecursionCallback(m1,m2), mThreshold(threshold) {reset();}

	virtual void reset() {
		RecursionCallback::reset();
		mReport.clear();
	}
	virtual void leafTest(const Leaf *l1, const Leaf *l2);
	virtual double quickTest(const Node *n1, const Node *n2);
	virtual bool distanceTest(double dSq) {
		if (dSq <= mThreshold * mThreshold) return true;
		return false;
	}
	const ContactReport& getReport(){return mReport;}
	void printStatistics();
};

/*! Recursion callback for the distance test.
	<ul>
	<li> leaf test - distance between triangles
	<li> quick test - returns the distance between bounding boxes
	<li> distance test - if distance between bounding boxes is more
	than smallest distance found so far, or the bodies have already
	been found to interpenetrate, no need to recurse.
	</ul>
*/
class DistanceCallback : public RecursionCallback
{
private:
	double mMinDistSq;
	position mP1, mP2;
public:
	DistanceCallback(const CollisionModel *m1, const CollisionModel *m2) : 
					 RecursionCallback(m1,m2) {reset();}
	virtual void reset() {
		RecursionCallback::reset();
		mMinDistSq = 1.0e10;
		mP1 = position(0,0,0); mP2 = position(0,0,0);
	}
	virtual void leafTest(const Leaf *l1, const Leaf *l2);
	virtual double quickTest(const Node *n1, const Node *n2);
	virtual bool distanceTest(double dSq) {
		if (mMinDistSq < 0.0) return false;
		if (dSq<0.0 || dSq <= mMinDistSq) return true;
		return false;
	}
	double getMin() const {if (mMinDistSq<0.0) return -1.0; return sqrt(mMinDistSq);}
	void getClosestPoints(position &p1, position &p2) const {p1 = mP1; p2 = mP2;}
	void printStatistics();
};

/*! Recursion callback for the closest point test.
	<ul>
	<li> leaf test - distance between point and triangles
	<li> quick test - returns the distance between point and bounding box
	<li> distance test - if distance between point and bounding box is more
	than smallest distance found so far, no need to recurse.
	</ul>
*/
class ClosestPtCallback : public RecursionCallback
{
private:
	double mMinDistSq;
	position mRefPosition, mClosestPt;
public:
	ClosestPtCallback(const CollisionModel *m1, const position &p) : 
					 RecursionCallback(m1, NULL), mRefPosition(p) {reset();}
	virtual void reset() {
		RecursionCallback::reset();
		mMinDistSq = 1.0e10;
	}
	virtual void leafTest(const Leaf *l1, const Leaf *l2);
	virtual double quickTest(const Node *n1, const Node *n2);
	virtual bool distanceTest(double dSq) {
		if (mMinDistSq < 0.0) return false;
		if (dSq<0.0 || dSq <= mMinDistSq) return true;
		return false;
	}
	double getMin() {if (mMinDistSq<0.0) return -1.0; return sqrt(mMinDistSq);}
	position getClosestPt() {return mClosestPt;}
	void printStatistics();
};

/*! Recursion callback for the region test.
	<ul>
	<li> leaf test - find all vertices within region
	<li> quick test - returns the distance between point and bounding box
	<li> distance test - if distance between point and bounding box is more
	than region threshold, no need to recurse.
	</ul>
*/
class RegionCallback : public RecursionCallback
{
private:
	Neighborhood mNeighborhood;
	position mRefPosition;
	vec3 mNormal;
	double mRadiusSq;

	void insertPoint(const position &point);
public:
	RegionCallback(const CollisionModel *m1, const position &p, const vec3 &n, double r) : 
				   RecursionCallback(m1, NULL), mRefPosition(p), 
				   mNormal(n), mRadiusSq(r * r) {reset();}
	virtual void reset() {
		RecursionCallback::reset();
		mNeighborhood.clear();
	}
	virtual void leafTest(const Leaf *l1, const Leaf *l2);
	virtual double quickTest(const Node *n1, const Node *n2);
	virtual bool distanceTest(double dSq) {
		if (dSq<0.0 || dSq <= mRadiusSq) return true;
		return false;
	}
	Neighborhood& getRegion() {return mNeighborhood;}
	void printStatistics();
};

} //namespace Collision

#endif
