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
// $Id: onLinePlanner.h,v 1.22 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#include <QObject>
#include <time.h>
#include <list>

#include "simAnnPlanner.h"

class Hand;
class GraspTester;
class OnLineGraspInterface;
class transf;

/*	An online planner has the following characteristics:
	<ul>
	<li> runs FAST simulated annealing planning again and again	
	<li> runs a tester in a separate thread that takes the solutions 
	and tests them for F-C
	</ul>	
	Right now it uses a SimulatedAnnealing optimization. Might use 
	others in the future (Gradient Descent, Sampling etc). This 
	planner ALWAYS uses a clone to perform the search. However, it 
	also keeps a pointer to the initial hand (as mRefHand) in order 
	to use it to receive input.

	Nothing in its structure actually defines the use of on-line 
	input. However, since it always uses a clone, the initial
	reference hand is free to be controlled by the user and as a 
	source of input. Wrist position input is taken into	account by 
	repeatedly setting the REFERENCE transform of the mCurrentState 
	to the transform of the reference hand. (see the mainLoop() 
	fctn). Then, as the variables in the HandObjectState will define 
	OFFSETS from this state, we can use a mInputState with all 
	the relevant variables set to 0. A mCurrentState with 0 
	translation and 0 rotation is identical	to the state of the 
	mRefHand.

	For hand posture input, we process the CyberGlove - this is 
	actually done up in the processInput() of the EGPlanner class.
*/

class OnLinePlanner : public SimAnnPlanner
{
private:
	/*! The on-line planner needs to know about a reference hand that is controlled by the user and which
		gives us a reference point for the search. It is not ideal to store it like this, but so far
		this is the working solution.*/
	Hand *mRefHand;

	//! This clone is used just to show the best solution currently available; mostly for debugging
	Hand *mSolutionClone;
	//! If this flag is set, the planner will show a visual indicator of each solution grasp it finds
	bool mMarkSolutions;
	//! How far the current solution is from the hand 
	/*! If solution distance is negative, we don't have a current solution */
	double mSolutionDistance;
	//! How far from the object we think we are (along app dir.)
	double mObjectDistance;
	//! Best state found so far
	GraspPlanningState *mCurrentBest;
	//! This one runs in a separate thread and tests grasps (usually through autograsp)
	GraspTester *mGraspTester;
	//! When using a grasp tester, this keeps a list of candidates to be sent to the tester
	std::list<GraspPlanningState*> mCandidateList;
	//! This class is used to interface with a real hand during grasping tasks. 
	/*! In theory, planning and execution are different things, so this class (who does the 
		planning) should know nothing about the class that actually executes the grasp. For 
		now however, this was easier to code.*/
	OnLineGraspInterface *mInterface;

	int mSolutionCount;

	OnLinePlanner(){}
	//! Creates another clone that is used for showing the current solution
	void createSolutionClone();
	//! A measure of how different two states are 
	/*! The online planner has a different idea on how to measure distance between 
		states. It looks just at position (not posture) and ignores changes along 
		the approach direction of the hand.*/
	double stateDistance(const GraspPlanningState *s1, const GraspPlanningState *s2);
	double distanceOutsideApproach(const transf &solTran, const transf &handTran);
	void updateSolutionList();

	void resetParameters();
	/*! The main loop here is divided into two parts. The mainLoop() itself manages 
		the mRefHand, looks at input, manages the sub-thread that does F-C testing 
		etc. */
	void mainLoop();
	/*! This part is called from the main loop and is responsible for the actual 
		grasp planning that is taking place at this level.*/
	void graspLoop();

public:
	OnLinePlanner(Hand *h);
	~OnLinePlanner();
	virtual PlannerType getType(){return PLANNER_ONLINE;}

	void startPlanner();
	void pausePlanner();
	bool resetPlanner();

	double getSolutionDistance() const {return mSolutionDistance;}
	double getObjectDistance() const {return mObjectDistance;}
	int getSABufferSize() const {return (int)mCandidateList.size();}
	int getFCBufferSize() const;
	//! Shows or hides the solution clone, not the planning clone
	void showSolutionClone(bool s);
	//! Shows or hides the planning clone, not the solution clone
	void showClone(bool s);

	//these just relay the actions to the OnLineGraspInterface. As actions have nothing to do with planning, this
	//shouldn't theoretically be here at all, but it was easier to interface with the action class through here
	void action(ActionType a);
	void useRealBarrettHand(bool s);
	ActionType getAction();
};
