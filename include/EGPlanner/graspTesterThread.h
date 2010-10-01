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
// $Id: graspTesterThread.h,v 1.10 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#ifndef _grasptesterthread_h_
#define _grasptesterthread_h_

#include <QThread>
#include <QMutex>
#include <list>

#include "egPlanner.h"

class GraspPlanningState;
class Hand;
class Body;

/*!	The GraspTesterThread is a helper class for the OnLinePlanner. It is 
	designed to have a list of possible grasps that is tests in its own
	thread. The main characteristic is that is allows external access to
	its list of "candidates": the user can place candidates in this planner's
	list, which will be tested when their turn comes. This class will place
	the results of testing in a "solution" list, where an external user
	can retrieve them from.

	How exactly the testing is being done should be more modular, right
	now it is hard-coded in. A lot of the functionality of how candidates
	are tested and solutions saved are customized for the on-line planner
	needs.
*/
class GraspTester : public EGPlanner
{
private:
	//! The mutex used for synchronizig access to the candidate list and the solution list
	QMutex mListMutex;
	//! Saves the results of testing the candidates
	std::list<GraspPlanningState*> mSolutionList;
	//! Stores candidates queued up for testing
	std::list<GraspPlanningState*> mCandidateList;
	//! The max number of candidates that this will accept to buffer for testing
	int mMaxCandidates;
	//! The current number of candidates that have been queued for testing
	int mNumCandidates;

	//! Retrieves the next candidate in line for testing
	GraspPlanningState* popCandidate();
	//! Posts a new solution in the list
	void postSolution(GraspPlanningState*);

	//! The actual testing routine
	void testGrasp(GraspPlanningState *s);

	//! Keeps checking the buffer of candidates and tests them when they are available
	void mainLoop();
public:
	//! Hard coded to loop forever and use STRICT_AUTOGRASP energy
	GraspTester(Hand *h);
	//! Also clears the internal buffers
	~GraspTester();
	virtual PlannerType getType(){return PLANNER_GT;}

	//! Hard-coded to only accept STRICT_AUTOGRASP energy, which is also set in constructor
	void setEnergyType(SearchEnergyType);
	//! Also clears internal buffers for candidates and solutions
	virtual bool resetPlanner();

	//! Add another candidate to the back of list for testing, if there is room
	bool postCandidate(GraspPlanningState *s);
	//! Retrieve the least recently posted solution in the list
	GraspPlanningState* popSolution();
	//! Clear both the candidate and the solution buffer
	void clearBuffers();

	//! Returns the number of candidates currently queued for testing
	int getNumCandidates(){return mNumCandidates;}
};

#endif
