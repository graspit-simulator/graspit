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
// $Id: guidedPlanner.h,v 1.7 2009/07/24 14:39:59 cmatei Exp $
//
//######################################################################

#ifndef _guidedplanner_h_
#define _guidedplanner_h_

#include <vector>
#include <list>
#include <QObject>

#include "search.h"
#include "simAnnPlanner.h"

class Hand;
class GraspPlanningState;
class SearchEnergy;
class SimAnn;

//! A planner that will fire off child threads to investigate promising states in more detail
/*!	The GuidedPlanner is a more complicated version of the SimAnn planner. 
	It will run just like the SimAnn planner, but whenever it finds "good" 
	states it will fire of children that run in separate threads to 
	investigate the "good" states in more detail. The children will use a 
	different type of EnergyCalculator which actually closes the hand and 
	computes exact grasp quality for each state. If the time of planning 
	is not an issue (2 mins or more) this is the best planner to guarantee 
	finding exact force-closure grasps. Might be unstable due to 
	multi-threaded operation.

	States that have been sent to a child planner are remembered and 
	avoided - no point in exploring them anymore since a child planner is 
	exploring each of them in great detail.

	Whenever a child thread finishes, the main planner will take its best 
	solution and store it in its own solution list. Therefore, the list of
	solutions of the main planner (mBestList) is actually a list on 
	solutions, one from each child that has finished.

	In general, planners operating in eigengrasp space will always find
	"pre-grasps" that must usually be transformed into actual grasps by 
	closing the fingers, or doing some slightly modified version of 
	autoGrasp. The exact transformation from pre-grasp to final grasp is
	simple, but usually hidden inside the SearchEnergy class which analyzes
	pre-grasps. That class is kind of hard to understand, as it is old code
	in bad need of a shake-up.
	
	Starting in version 2.1.0, the main GuidedPlanner also stores the final
	grasp for any of its solutions. Therefore, for each solution, the best list
	of this planner will contain the pre-grasp followed by the final grasp. This
	wsa done in an attempt to make this planner usable "off the shelf" without
	having to worry about recomputing exact final grasps after its done. See the
	stopChild(...) function for details about how the solutions recovered from
	children threads are saved.
*/
class GuidedPlanner : public SimAnnPlanner
{
	Q_OBJECT
protected:
	//! The max number of children that canbe used. Usually correlated with the number of cores available.
	int mMaxChildren;
	//! The list of currently active child planners.
	std::vector<SimAnnPlanner*> mChildPlanners;

	//! A list of seeds that haven't been used yet
	std::list<GraspPlanningState*> mChildSeeds;

	//! A list of states that should be avoided by the search
	/*! They are avoded because they have either been used by a child or are 
		scheduled to be used by one. Essentially, it is the union of 
		mChildSeeds and mBestList.
	*/
	std::list<GraspPlanningState*> mAvoidList;

	//! Whether the children use clones of the current hand
	/*! True by default, should be set to false only for debugging purposes.*/
	bool mChildClones;
	//! Whether the children run in their own threads
	/*! True by default, should be set to false only for debugging purposes.*/
	bool mChildThreads;

	//! Fires of a child planner that will be seeded with the given state
	void startChild(const GraspPlanningState *s);
	//! Stops a child planner, gets its solutions and stops its thread
	void stopChild(SimAnnPlanner *pl);
	//! Checks if any of the currently active children have finished their execution
	void checkChildren();

	//! Stores states to be used as "seeds" for children, and fires new children when appropriate
	void mainLoop();
public:
	//! Number of child results that are saved 
	int mBestListSize;
	//! Max size of child seeds list
	int mChildSeedSize;
	//! Min distance separating child seeds
	float mDistanceThreshold;
	//! Min energy for a state to be deemed good enough to be used to seed a child
	float mMinChildEnergy;
	//! Energy computation type used by children
	SearchEnergyType mChildEnergyType;
	//! Max iterations done by a child
	int mMaxChildSteps;
	
	GuidedPlanner(Hand *h);
	~GuidedPlanner();
	virtual PlannerType getType(){return PLANNER_MT;}

	//! Also hides the friction cones on the object
	void startPlanner();
	//! Also stops all children, for good (stops their threads)
	void stopPlanner();
	//! Not properly implemented yet as this also stops children for good (instead of pausing them)
	void pausePlanner();
	//! Also clears the list of states to be avoided
	bool resetPlanner();
	void setMaxChildren(int c){mMaxChildren=c;}
};

#endif
