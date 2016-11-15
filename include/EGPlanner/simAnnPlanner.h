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
// $Id: simAnnPlanner.h,v 1.12 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#ifndef _simannplanner_h_
#define _simannplanner_h_

#include <vector>
#include <list>

#include "QObject"
#include "search.h"
#include "egPlanner.h"

class Hand;
class Body;
class GraspPlanningState;
class SoSensor;
class SearchEnergy;
class SimAnn;

/*!	This is the simplest implementation of the EGPlanner. It also has a 
	SimAnn class for doing simulated annealing over	all variables. This 
	is exactly what it does in the mainLoop() fctn. It buffers the best 
	states (with lowest energy)	as it finds them.
*/
class SimAnnPlanner : public EGPlanner
{
protected:
	//! The instance that is used to do simulated annealing
	SimAnn *mSimAnn;
	SimAnnPlanner(){}
	//! Calls a simulated annealing step and buffers the best solutions
	void mainLoop();
	//! Also resets the simulated annealer
	void resetParameters();
public:
	//! Also initializes the simulated annealer
	SimAnnPlanner(Hand *h);
	~SimAnnPlanner();
	virtual PlannerType getType(){return PLANNER_SIM_ANN;}
	void setAnnealingParameters(AnnealingType y);

	//! Checks if a model state has been set
	virtual bool initialized();
	//! Has to be called BEFORE any planning can begin. 
	/*! It tells the planner how the state it is searching on looks like (how many variables, etc). */
	virtual void setModelState(const GraspPlanningState *modelState);
};

#endif
