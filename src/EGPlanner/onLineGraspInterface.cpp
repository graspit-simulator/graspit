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
// $Id: onLineGraspInterface.cpp,v 1.8 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "robot.h"
#include "matvec3D.h"
#include "barrett.h"
#include "onLineGraspInterface.h"
#include "searchState.h"

#ifdef HARDWARE_LIB
#include "BarrettHand.h"
#endif

#include "debug.h"

OnLineGraspInterface::OnLineGraspInterface(Hand *h)
{
	mHand = h;
	mBarrettHand = NULL;
	mAction = ACTION_PLAN;
}

void
OnLineGraspInterface::useRealBarrettHand(bool s)
{
	if (s) {
		if ( !mHand->isA("Barrett") ) {
			DBGA("Can't use real hand: this is not a Barrett!");
			mBarrettHand = NULL;
			return;
		}
		mBarrettHand = ((Barrett*)mHand)->getRealHand();
	} else {
		mBarrettHand = NULL;
	}
}


/*!	Given a solution (grasping position) this function finds 
	a correct finger posture for the CURRENT position of the 
	hand, so that it's as close as possible to the solution 
	without hitting the object.	It also checks if there is a 
	path between the CURRENT position of the fingers and the 
	desired one.
*/
bool
OnLineGraspInterface::getSuggestedDOF(const GraspPlanningState *s, double *initialDof, double *finalDof)
{
	s->readPosture()->getHandDOF( finalDof );
	mHand->forceDOFVals( finalDof );

	//close fingers gradually as we move closer to the target state
	transf handTran = mHand->getTran();
	transf solTran = s->getTotalTran();
	vec3 app = handTran.translation() - solTran.translation();
	double dist = app.len();
		
	//first find how much we should open the fingers, based on distance from solution
	double openFingers = dist / 200.0;
	DBGP("Open fingers to " << openFingers);
	if (!mHand->quickOpen(openFingers)) {
		DBGP("Open finger position not found");
		return false;
	} 
	mHand->getDOFVals(finalDof);

	//also check if we can get from here to there 
	mHand->forceDOFVals(initialDof);
	if ( mHand->checkDOFPath(finalDof, 0.16) ) {
		return true;
	} else {
		DBGP("Open finger found, but not reachable");
		return false;
	}	
}

/*!	Given a list of f-c grasps, this function chooses one and shapes 
	the finger of the hand accordingly. It is assumed that the list 
	has been pre-sorted in order of "desirability".
*/
GraspPlanningState*
OnLineGraspInterface::updateHand(const std::list<GraspPlanningState*> *solutionList)
{
	if (mAction != ACTION_PLAN) return NULL;

	double *finalDof = new double[mHand->getNumDOF()];
	double *savedDof = new double[mHand->getNumDOF()];
	mHand->getDOFVals( savedDof);

	GraspPlanningState *s = NULL;
	std::list<GraspPlanningState*>::const_iterator it = solutionList->begin();
	while (it!=solutionList->end()) {
		//try the grasps in the order that they are presented.
		if ( (*it)->getDistance() > 1.0) { 
			//don't use solutions that are too far away. 
			break;
		}
		if ( getSuggestedDOF(*it,savedDof, finalDof) ) {
			//if I can find a way to shape the fingers for this grasps, this is the one
			s = (*it);
			break;
		}
		it++;
	}
	if (s) {
		DBGP("Update hand success");
		mHand->forceDOFVals(finalDof);
#ifdef HARDWARE_LIB
		if (mBarrettHand) {
			mBarrettHand->MoveTogether( finalDof );
		}
#endif
	} else {
		DBGP("Update hand failed");
		mHand->forceDOFVals(savedDof);
	}
	delete finalDof;
	delete savedDof;
	return s;
}

void
OnLineGraspInterface::action(ActionType a)
{
	switch(a) {
		case ACTION_PLAN:
			break;
		case ACTION_GRASP:
			mHand->autoGrasp(false);
#ifdef HARDWARE_LIB
			if (mBarrettHand) {
				while (mBarrettHand->isBusy());
				mBarrettHand->Close(BARRETT_ALL_FINGERS);
			}
#endif
			break;
		case ACTION_OPEN:
			mHand->autoGrasp(false,-1.0);
#ifdef HARDWARE_LIB
			if (mBarrettHand) {
				while (mBarrettHand->isBusy());
				mBarrettHand->Open(BARRETT_ALL_FINGERS);
			}
#endif
			break;
	};
	mAction = a;
}
