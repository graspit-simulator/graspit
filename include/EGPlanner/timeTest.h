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
// $Id: timeTest.h,v 1.8 2009/04/01 13:52:34 cmatei Exp $
//
//######################################################################

#include "simAnnPlanner.h"
#include <time.h>

class Hand;
class Body;

/*! \file
	These classes are ment to measure the performace of the system in 
	testing states. Mostly relevant for doing autograsp tests.
*/

//! Generates random states and tests them, but also keeps count of how many it has tested
class TimeTester : public SimAnnPlanner
{
private:
	void mainLoop();
protected:
	int mCount,mIllegalCount;
public:
	TimeTester(Hand *h) : SimAnnPlanner(h){}
	virtual PlannerType getType(){return PLANNER_TIME_TEST;}

	void startPlanner();
	int getCount(){return mCount;}
	int getIllegal(){return mIllegalCount;}
};

//! Starts a number of children of the TimeTester type, and times each of them
class MTTester : public TimeTester
{
private:
	TimeTester *startChild();
	std::vector<TimeTester*> mChildren;
	void mainLoop(){}
public:
	MTTester(Hand *h) : TimeTester(h){}
	void startPlanner();
	void pausePlanner();
};
