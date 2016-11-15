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
// $Id: listPlanner.h,v 1.7 2009/07/02 21:06:22 cmatei Exp $
//
//######################################################################

#ifndef _listplanner_h_
#define _listplanner_h_

#include "egPlanner.h"

#include <list>

class GraspPlanningState;

/*! The List Planner simply processes a list of grasps, computes the quality for
	each of them and then sorts the results. The list of grasps can be either
	passed in or computed on the spot by inherited versions of this class.

	Depending on the EnergyFunction used, the incoming list can be of grasps or
	pre-grasps.

	The original Primitive Planner can be re-cast as one of these. I am hoping to
	do that at some point. 
*/
class ListPlanner : public EGPlanner
{
protected:
	/*! The list of grasps to be tried by this planner. All HandObjectStates are deleted
		upon destruction of the planner */
	std::list<GraspPlanningState*> mInputList;

	//! The current position in the input list; shows the next grasp to be tried
	std::list<GraspPlanningState*>::iterator mPlanningIterator;

	//! Also resets the planning iterator
	virtual void resetParameters();

	//! The main planning function
	virtual void mainLoop();

	//! Pulls the state at the given index from the input list
	GraspPlanningState *getState(int index);
public:
	ListPlanner(Hand *h);
	~ListPlanner();
	virtual PlannerType getType(){return PLANNER_LIST;}

	//! True as long as there is something to test in the input list
	virtual bool initialized(){return !mInputList.empty();}

	/*! Sets the list of input grasps to test. This class will take ownership of the list
		and delete all the grasps inside in its own destructor */
	void setInput(std::list<GraspPlanningState*> input);

	//! Shows or hides visual markers of all input states
	void showVisualMarkers(bool show);

	//! Tests a the pre-grasp at the given index in the current input list
	void testState(int index);

	//! Shows the unprocessed state at the given index in the input list
	void showState(int index);

	//! A hack; shows the state and also does approachToContact as the SearchEnergy would
	void prepareState(int index);
};

#endif
