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
// $Id: loopPlanner.h,v 1.5 2009/12/01 18:45:28 cmatei Exp $
//
//######################################################################

#ifndef _loopplanner_h_
#define _loopplanner_h_

#include <list>
#include <QObject>

#include "simAnnPlanner.h"
#include "search.h"

class Hand;
class GraspPlanningState;

/*! The Loop Planner is a Sim Ann planner that will loop forever. After 
	each loop, it will place the best state found in this loop in the 
	list of avoidable states, so that at the next run it will search 
	somewhere else.

	After a couple of loops, the solutions found over the entire planning
	time are thus found in the avoid list, rather in the best list which
	just stores the solutions found in the current loop.
*/
class LoopPlanner : public SimAnnPlanner
{
	Q_OBJECT
protected:
	//! The list of states to be avoided during the current loop
	std::list<GraspPlanningState*> mAvoidList;
	//! The distance to be kept from the avoided states
	float mDistanceThreshold;
	//! Places the best solutions currently available in the avoid list
	virtual void resetParameters();
signals:
	//! Emmitted after a full loop is completed
	void loopUpdate();

public:
	LoopPlanner(Hand *h);
	~LoopPlanner();
	virtual PlannerType getType(){return PLANNER_LOOP;}

	//! Gets grasps from the avoid list instead of the best list
	virtual const GraspPlanningState* getGrasp(int i);
	//! Returns the size of the avoid list
	virtual int getListSize(){return mAvoidList.size();}
	//! Also clears the avoid list
	virtual void clearSolutions();

	//! Adds another state to the avoid list (and implicitly to the list of solutions)
	/*! Also takes ownership of the passed state, and will delete it on cleanup */
	void addToAvoidList(GraspPlanningState* state){mAvoidList.push_back(state);}
};
#endif
