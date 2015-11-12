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
// $Id: loopPlanner.h,v 1.4 2009/10/08 16:21:31 cmatei Exp $
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
};
#endif
