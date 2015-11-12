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
