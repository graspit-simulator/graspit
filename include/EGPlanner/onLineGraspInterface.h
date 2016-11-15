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
