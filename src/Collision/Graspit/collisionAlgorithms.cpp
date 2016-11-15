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
// $Id: collisionAlgorithms.cpp,v 1.6 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#include "collisionAlgorithms.h"
#include "collisionModel.h"

//#define GRASPITDBG
#include "debug.h"

//for now, these are always here, as they are virtual fctns and therefore
//can not be inlined
#include "collisionAlgorithms_inl.h"

namespace Collision {

//////////////////////////////////////////////////////////////////////////////
//                           Recursion
//////////////////////////////////////////////////////////////////////////////

void recursion(const Node *n1, const Node *n2, RecursionCallback *rc)
{
	PROF_START_TIMER(COLLISION_RECURSION);
	// if both are leaves, we will test them and return. But if not, we have
	// at least a branch which we will continue the recursion on.
	// here we determine which is the branch that we recurse on.
	const Node* s;
	const Branch *r;
	// we must preserve the order of the two models in the calls, so n1
	// always belongs to model1 and n2 always belongs to model2. This 
	// variable is used to keep track of this.
	bool nodeSwap = false;
	if ( n1->isLeaf() ){
		if ( !n2 || n2->isLeaf() ) {
			//both are leaves; call leaf test and we are done
			rc->leafTest(static_cast<const Leaf*>(n1), static_cast<const Leaf*>(n2));
			PROF_STOP_TIMER(COLLISION_RECURSION);
			return;
		} else {
			//n1 is a leaf, but n2 is a branch; we will recurse on n2's children
			s = n1;
			r = static_cast<const Branch*>(n2);
			nodeSwap = true;
		}
	} else {
		if (!n2 || n2->isLeaf()) {
			//n2 is a leaf, but n1 is a branch; we will recurse on n1's children
			s = n2;
			r = static_cast<const Branch*>(n1);
		} else {
			//both are branches; we will recurse on the children of the branch 
			//with larger volume
			if ( n1->getBoxVolume() > n2->getBoxVolume() ) {
				r = static_cast<const Branch*>(n1); s = n2;
			} else {
				r = static_cast<const Branch*>(n2); s = n1;
				nodeSwap = true;
			}
		}
	}
	//we know what branch to recurse on; get its children
	const Node* c1 = r->child1();
	const Node* c2 = r->child2();

	//the "distance" is a measure of how much we want to recurse on this 
	//child. Compute it for both, then arrange them in order of increasing
	//distance to see if we recurse on them
	//make sure we don't change the order of the models
	double d1,d2;
	if (!nodeSwap) {
		d1 = rc->quickTest(c1, s);
		d2 = rc->quickTest(c2, s);
	} else {
		d1 = rc->quickTest(s, c1);
		d2 = rc->quickTest(s, c2);
	}
	if (d2 < d1) {
		std::swap(c1,c2);
		std::swap(d1,d2);
	}

	//check if we actually want to recurse on this child at all
	if ( rc->distanceTest(d1) ) {
		DBGST( c1->mark(true) );
		DBGST(  if (s) s->mark(true) );
		PROF_STOP_TIMER(COLLISION_RECURSION);
		//make sure we don't change the order of the models
		if (!nodeSwap) {
			recursion(c1,s,rc);
		} else {
			recursion(s,c1,rc);
		}
		PROF_START_TIMER(COLLISION_RECURSION);
	}
	//check if we actually want to recurse on this child at all
	if ( rc->distanceTest(d2) ) {
		DBGST( c2->mark(true) );
		DBGST(  if (s) s->mark(true) );
		PROF_STOP_TIMER(COLLISION_RECURSION);
		//make sure we don't change the order of the models
		if (!nodeSwap) {
			recursion(c2,s,rc);
		} else {
			recursion(s,c2,rc);
		}
		PROF_START_TIMER(COLLISION_RECURSION);
	}
	PROF_STOP_TIMER(COLLISION_RECURSION);
}

void startRecursion(const CollisionModel *model1, const CollisionModel *model2, RecursionCallback *rc)
{
	const Node *node1 = model1->getRoot(), *node2 = NULL;
	if (model2) node2 = model2->getRoot();

	DBGST( node1->markRecurse(false) );
	DBGST( if (node2) node2->markRecurse(false) );

	double d = rc->quickTest( node1, node2 );
	if (rc->distanceTest(d)) {
		DBGST( node1->mark(true) );
		DBGST( if (node2) node2->mark(true) );
		//the actual recursive call to contact detection
		recursion( node1, node2, rc);
	}
}

RecursionCallback::RecursionCallback(const CollisionModel *m1, const CollisionModel *m2)  : 
					  mModel1(m1), mModel2(m2) 
{
	if (mModel2) {
		mTran2To1 = mModel2->getTran() * mModel1->getTran().inverse();
		mTran1To2 = mModel1->getTran() * mModel2->getTran().inverse();
	}
}

void RecursionCallback::printStatistics()
{
	DBGA("   Quick tests: " << mNumQuickTests    );
	DBGA("    Leaf tests: " << mNumLeafTests    );
	DBGA("Triangle tests: " << mNumTriangleTests);
}

void CollisionCallback::printStatistics() {
	DBGA("Collision callback");
	RecursionCallback::printStatistics();
	DBGA("     Collision: " << mCollision << "\n" );
}

/*! Inserts a contact into the list and checks for duplicates */
void 
ContactCallback::insertContactNoDuplicates(const position &p1, const position &p2, 
										   const vec3 &n1, const vec3 &n2, 
										   double distSq, double thresh)
{
	bool insert = true;
	ContactReport::iterator cit = mReport.begin();
	while(cit!=mReport.end() && insert) {
		bool remove = false;
		if ( (cit->b1_pos - p1).len_sq() < thresh * thresh ) {
			if ( (cit->b2_pos - p2).len_sq() < thresh * thresh ) {
				//exact same contact, no need to insert
				insert = false;
			} else {
				if (cit->distSq < distSq) {
					insert = false;
				} else {
					remove = true;
				}
			}
		} else if ( (cit->b2_pos - p2).len_sq() < thresh * thresh ) {
			if (cit->distSq < distSq) {
				insert = false;
			} else {
				remove = true;
			}
		}
		if (remove) {
			cit = mReport.erase(cit);
		} else {
			cit++;
		}
	}
	if (insert) {
		mReport.push_back( ContactData(p1, p2, n1, n2, distSq) );
	}
}

void ContactCallback::printStatistics() {
	DBGA("Contact callback");
	RecursionCallback::printStatistics();
	DBGA("      Contacts: " << mReport.size() << "\n" );
}


void DistanceCallback::printStatistics() {
	DBGA("Distance callback");
	RecursionCallback::printStatistics();
	DBGA("      Min dist: " << getMin() << "\n" );
}

void ClosestPtCallback::printStatistics() {
	DBGA("Closest pt callback");
	RecursionCallback::printStatistics();
	DBGA("      Min dist: " << getMin());
	DBGA(" Closest point: " << mClosestPt << "\n");
}

void 
RegionCallback::printStatistics() {
	DBGA("Region Callback");
	RecursionCallback::printStatistics();
	DBGA(" Pts in region: " << mNeighborhood.size() << "\n");
}

void
RegionCallback::insertPoint(const position &point)
{
	Neighborhood::iterator it;
	for (it=mNeighborhood.begin(); it!=mNeighborhood.end(); it++) {
		if ( *it == point ) break;
	}
	if (it == mNeighborhood.end()) {
		mNeighborhood.push_back(point);
	}
}

} //namespace Collision
