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
// $Id: collisionModel.h,v 1.5 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _collisionmodel_h_
#define _collisionmodel_h_

#include <list>
#include <vector>

#include "matvec3D.h"
#include "bBox.h"
#include "triangle.h"

/*! \file
	Defines the collison detection hierarchy for a model. This consists of 
	models and their bounding box (bb) hierarchies.

	Other than implementing the interface, and thus being the public face
	of the collision detection system, this file also contains the 
	"broadphase" algorithms, and manages the activation state of the bodies
	in the system.
*/

class BoundingBox;

namespace Collision{

class Branch;
class CollisionModel;
//! A node in a bounding box hierarchy
/*! A node can be either a Leaf or a Branch. As such, it contains the 
	functionality that is common to both. Mainly, this means a bounding
	box and recursion functions.
*/
class Node {
protected:
	BoundingBox mBbox;
public:
	//! Just a stub
	Node() {}
	virtual ~Node(){}
	virtual bool isLeaf() const = 0;
	virtual Branch* split(){return NULL;}
	virtual void getBVRecurse(int currentDepth, int desiredDepth, std::vector<BoundingBox> *bvs);
	virtual int countRecurse(){return 1;}
	const BoundingBox& getBox() const {return mBbox;}
	double getBoxVolume() const {return mBbox.halfSize.x() * 
								 mBbox.halfSize.y() * mBbox.halfSize.z();}
	//for debugging
	virtual void markRecurse(bool m) const {mBbox.mMark = m;}
	virtual void mark(bool m) const {mBbox.mMark = m;}
};

//! A leaf contains actual geometry, in the form of a list of triangles
/*! The leaf is the only type of node that contains actual geoemtry. It
	also knows how to compute its own bounding box and to split itself
	into children.
*/
class Leaf : public Node {
private:
	//! Added around bounding boxes to guarantee they enclose the contents
	static const double TOLERANCE;
	//! The Bbox hierarchy is built until all leaves have at most this many triangles
	static const int MAX_LEAF_TRIANGLES;

	std::list<Triangle> mTriangles;
	void computeBboxAA();
	void computeBboxOO();
	position getMeanVertex();
	//! Finds the median projection point of all vertices unto a given axis
	double getMedianProjection(const vec3 &axis);
	void fitBox(const mat3 &R, vec3 &center, vec3 &halfSize);
	//! Distributes triangles between children based on projection of triangle centroid 
	void balancedSplit(vec3 axis, double sepPoint, Leaf *child1, Leaf *child2);
	//! Ensures equal number of triangles per child by distributing triangles randomly
	void randomSplit(Leaf *child1, Leaf *child2);
	//! Computes the split point along the given axis that ensures min volume of child bounding boxes
	void optimalSplit(const vec3 &x, const vec3 &y, const vec3 &z, Leaf *child1, Leaf *child2);
	//! Computes the covariance matrix of all triangles weighted by triangle area
	void areaWeightedCovarianceMatrix(double covMat[3][3]);
public:
	//! Just a stub
	Leaf() : Node() {}
	bool isLeaf() const {return true;}
	//! Splits this leaf into 2 children; returns a new branch with the new children
	Branch* split();
	void addTriangle(Triangle t){mTriangles.push_back(t);}
	void computeBbox(){computeBboxOO();}
	int getNumTriangles() const {return mTriangles.size();}
	void clearTriangles() {mTriangles.clear();}
	const std::list<Triangle>* getTriangles() const {return &mTriangles;}
};

//! A Branch is a node with two children
/*! The Branch is the only type of node to have children. 
*/
class Branch : public Node {
private:
	Node *mChild1, *mChild2;
	Branch(Leaf *c1, Leaf *c2, const BoundingBox &box) : Node(), mChild1(c1), mChild2(c2) {
		mBbox.setTran( box.getTran() );
		mBbox.halfSize = box.halfSize;
	}
	friend Branch* Leaf::split();
public:
	~Branch() {if(mChild1) delete mChild1; if (mChild2) delete mChild2;}
	bool isLeaf() const {return false;}
	/*! Splits both children, then recursively calls splitRecurse() on the 
		resulting branches (if any). Stops recursing when the current
		depth hits the desired depth, or recurses until no longer possible
		if desired depth < 0.
	*/
	void splitRecurse(int currentDepth, int desiredDepth) {
		if (desiredDepth >= 0 && currentDepth >= desiredDepth) return;
		Branch *tmp = mChild1->split();
		if (tmp) {
			delete mChild1;
			mChild1 = tmp;
			tmp->splitRecurse(currentDepth+1, desiredDepth);
		}
		tmp = mChild2->split();
		if (tmp) {
			delete mChild2;
			mChild2 = tmp;
			tmp->splitRecurse(currentDepth+1, desiredDepth);
		}
	}
	const Node* child1() const {return mChild1;}
	const Node* child2() const {return mChild2;}
	virtual void getBVRecurse(int currentDepth, int desiredDepth, std::vector<BoundingBox> *bvs);
	virtual int countRecurse(){return 1 + mChild1->countRecurse() + mChild2->countRecurse();}
	virtual void markRecurse(bool m) const {
		Node::markRecurse(m);
		//only negative marks get percolated down
		if (!m) {
			mChild1->markRecurse(m);
			mChild2->markRecurse(m);
		}
	}
};

/*! The collision model contains its bb hierarchy, a transform and a flag
	indicating whether it is active or not. It is the only public interface
	with the bb hierarchy: a model is created with a number of triangles,
	then it is asked to build its own bb hierarchy.
*/
class CollisionModel {
private:
	Node *mRoot;
	bool mActive;
	transf mTran;
	//! Tells us if this model is a clone of another model
	bool mClone;
	//! The id of the thread that this body was added in
	/*! This marker helps us keep track of bodies that are specific to a given thread.
		See the threading functions of the CollisionInterface for details. */
	int mThreadId;
public:
	//! Creates an empty collision model with no geometry and a tree with just an empty Leaf root
	CollisionModel(int threadId) {
		mRoot = new Leaf();
		mActive = true;
		mClone = false;
		mThreadId = threadId;
	}
	//! Also deletes the bb hierarchy, if this model is not a clone
	~CollisionModel(){if(!mClone) delete mRoot;}
	//! This model will share the bbox hierarchy of the original
	void cloneModel(CollisionModel *original);
	//! Adds a triangle to the model. Only works if bb hierarchy is not built yet.
	void addTriangle(Triangle t);
	const Node* getRoot() const {return mRoot;}
	//! Destroys the bb hierarchy, but does not save the triangles that it contained
	void reset();
	//! Builds the bbox hierarchy. Recursively calls split on the root.
	void build();
	void getBoundingVolumes(int depth, std::vector<BoundingBox> *bvs);
	void setTran(const transf &t){mTran = t;}
	const transf& getTran() const {return mTran;}
	bool isActive() const {return mActive;}
	void setActive(bool act){mActive = act;}
	int getThreadId() const {return mThreadId;}
};

}

#endif
