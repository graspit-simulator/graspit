//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: onLineGraspInterface.h,v 1.4 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#ifndef _onlinegraspinterface_h_
#define _onlinegraspinterface_h_

#include <list>
#include "search.h"

class Hand;
class GraspPlanningState;
class BarrettHand;

/*! This class is designed to perform interactive grasping tasks where 
	GraspIt helps the user by planning grasps interactively. It uses a 
	list of planned grasps (presumably by some planner) and knows which 
	hand to shape.

	Planning good grasps is one thing; actually executing them is 
	another. It's not enough to know a number of f-c grasps. You have to 
	decide which one to choose, how to close the fingers gradually, etc. 
	In this context	you are also interacting with a human operator. You 
	have to decide which grasp to use depending on what the	operator is 
	doing, shape the fingers,close them gradually as the operator is 
	approaching the object etc.
	
	These are all complicated problems in themselves. My main goal is to 
	plan the grasps, how they are used is a different problem. There are 
	probably better ways of accomplishing this, this one is just a 
	proof-of-concept implementation to show how the OnLinePlanner can be 
	applied.
*/
class OnLineGraspInterface
{
private:
	ActionType mAction;
	Hand *mHand;
	//! When using a real Barrett hand this is the communicator with it
	BarrettHand *mBarrettHand;
	bool getSuggestedDOF(const GraspPlanningState *s, double *initialDof, double *finalDof);
public:
	OnLineGraspInterface(Hand *h);
	GraspPlanningState* updateHand(const std::list<GraspPlanningState*> *solutionList);
	void action(ActionType a);
	void useRealBarrettHand(bool s);
	ActionType getAction(){return mAction;}
};

#endif
