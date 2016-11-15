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
// $Id: guidedPlanner.cpp,v 1.9 2009/07/24 14:40:10 cmatei Exp $
//
//######################################################################

#include "robot.h"
#include "world.h"
#include "guidedPlanner.h"
#include "searchState.h"
#include "searchEnergy.h"
#include "simAnn.h"

//#define GRASPITDBG
#include "debug.h"

GuidedPlanner::GuidedPlanner(Hand *h)
{
	mHand = h;
	init();
	mEnergyCalculator = new ClosureSearchEnergy();
	mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
	((ClosureSearchEnergy*)mEnergyCalculator)->setAvoidList( &mAvoidList );
	mSimAnn = new SimAnn();
	mChildClones = true;
	mChildThreads = true;
	mMaxChildren = 1;
	mRepeat = true;

	//default values set up for columbia dbase project
	mBestListSize = 20;
	mChildSeedSize = 20;
	mDistanceThreshold = 0.3f;
	mMinChildEnergy = -0.1f;
	mChildEnergyType = ENERGY_STRICT_AUTOGRASP;
	mMaxChildSteps = 200;

	((ClosureSearchEnergy*)mEnergyCalculator)->setThreshold(mDistanceThreshold);
}

GuidedPlanner::~GuidedPlanner()
{
	while (!mAvoidList.empty()) {
		delete mAvoidList.front();
		mAvoidList.pop_front();
	}
	while (!mChildSeeds.empty()) {
		delete mChildSeeds.front();
		mChildSeeds.pop_front();
	}
}

/*! Since child planner will actually compute actual contacts with
	the object, we disable rendering of friction cones, as rendering
	requests from different threads can cause problems.
*/
void
GuidedPlanner::startPlanner()
{
	mCurrentState->getObject()->showFrictionCones(false);
	SimAnnPlanner::startPlanner();
}

/*! After the children have been stopped, it also deletes them. Any
	current solutions of the children are lost.
*/
void
GuidedPlanner::stopPlanner()
{
	for (int i=0; i<(int)mChildPlanners.size(); i++) {
		mChildPlanners[i]->stopPlanner();
	}
	checkChildren();
	SimAnnPlanner::stopPlanner();
}

void
GuidedPlanner::pausePlanner()
{
	SimAnnPlanner::pausePlanner();
	//stop all the children for good; it's just simpler
	for (int i=0; i<(int)mChildPlanners.size(); i++) {
		mChildPlanners[i]->stopPlanner();
	}
	checkChildren();
	mCurrentState->getObject()->showFrictionCones(true);
}

bool
GuidedPlanner::resetPlanner()
{
	while (!mAvoidList.empty()) {
		delete mAvoidList.back();
		mAvoidList.pop_back();
	}
	return SimAnnPlanner::resetPlanner();
}

/*! Starts a child and gives it as an "input" state the \a seed. The child 
	will then be biased to spend more time in the vicinity of this state.
	The exact confidence levels in the seed state are hard-coded in.

	Also hard-coded are the annealing parameters that the child uses 
	(ANNEALING_STRICT) and other parameters for the child. The seed state
	is also marked visually to give some visual feedback, since the child
	planners typically use cloned hands which are hidden.
*/
void
GuidedPlanner::startChild(const GraspPlanningState *seed)
{
	DBGP("Creating a child...");
	SimAnnPlanner *child = new SimAnnPlanner(mHand);
	if (mChildThreads) {
		child->startThread();
		child->showClone(false);
	}else if (mChildClones) {
		child->createAndUseClone();
	}
	DBGA("Child created (and started)");
	child->setEnergyType(mChildEnergyType);
	if (mChildEnergyType == ENERGY_CONTACT) {
		child->setContactType(CONTACT_PRESET);
	}
	child->setAnnealingParameters(ANNEAL_STRICT);
	child->setMaxSteps(mMaxChildSteps);			

	child->setModelState(seed);
	child->resetPlanner();
	child->getTargetState()->copyFrom(seed);
	child->getTargetState()->getPosition()->setAllConfidences(0.65);
	child->getTargetState()->getPosition()->setAllFixed(true);
	child->getTargetState()->getPosture()->setAllConfidences(0.5);
	child->getTargetState()->getPosture()->setAllFixed(true);

	child->startPlanner();
	mChildPlanners.push_back(child);

	seed->setIVMarkerColor(0,1,0);
}

/*! Stops a child, and waits for the child thread to finish. After that,
	it retrieves the solutions found by the child, and places them in 
	the list of solutions and also in the list of states to be avoided in 
	the future. Solutions are also marked visually.

	Note that, for each solution recovered from a child, this function also
	reconstructs the final grasp (remember that solutions are usually 
	pre-grasps) and saves BOTH in the list of solutions.
*/
void
GuidedPlanner::stopChild(SimAnnPlanner *pl)
{
  DBGA("Child has finished!");
  //this sets the state to DONE which in turn waits for the thread to stop spinning
  pl->stopPlanner();
  DBGP("Thread has stopped.");
  int j = pl->getListSize();
  if (j) {
    //place a copy of the best state (which is the pre-grasp) in my best list
    GraspPlanningState *s = new GraspPlanningState( pl->getGrasp(0) );
    s->printState();
    //recall that child used its own cloned hand for planning
    s->changeHand(mHand, true);
    mBestList.push_back(s);
    
    //use this part to also save the final grasp from the child planner, 
    //not just the pre-grasp
    pl->showGrasp(0);
    GraspPlanningState *finalGrasp = new GraspPlanningState(pl->getGrasp(0));
    finalGrasp->setPositionType(SPACE_COMPLETE);
    //the final grasp is not in eigengrasp space, so we must save it in DOF space
    finalGrasp->setPostureType(POSE_DOF);
    finalGrasp->saveCurrentHandState();
    //change the hand
    finalGrasp->changeHand(mHand, true);
    //and save it
    mBestList.push_back(finalGrasp);		
    
    //place a copy of the pre-grasp in the avoid list and mark it red
    GraspPlanningState *s2 = new GraspPlanningState(s);
    mAvoidList.push_back(s2);
    mHand->getWorld()->getIVRoot()->addChild( s2->getIVRoot() );
    if (s2->getEnergy() < 10.0) {
      s2->setIVMarkerColor(1,0,0);
    } else {
      s2->setIVMarkerColor(0.1,0.1,0.1);
    }
    DBGA("Enrgy from child: " << s2->getEnergy());
    //the avoid list gets emptied at cleanup
  }
  else
  {
    DBGA("Child has no solutions");
  }  
}

/*! Does the usual main loop of a SimAnn planner, but checks if the current
	state is good enough to be placed in the list of seeds to be used for
	children. The list of seeds is also pruned to remove similar state,
	and only keep a list of "unique" seeds.
*/
void
GuidedPlanner::mainLoop()
{
	// call main simann iteration
	SimAnn::Result r = mSimAnn->iterate(mCurrentState, mEnergyCalculator);
	if (r==SimAnn::FAIL) return;

	//put result in list
	double bestEnergy;
	if ((int)mChildSeeds.size() < mChildSeedSize) {
		//only queue good states to begin with
		bestEnergy = mMinChildEnergy;
	} else {
		bestEnergy = mChildSeeds.back()->getEnergy();
	}
	if (r==SimAnn::JUMP && mCurrentState->getEnergy() < bestEnergy) {
		GraspPlanningState *insertState = new GraspPlanningState(mCurrentState);
		DBGP("New solution. Is it a candidate?");
		if (!addToListOfUniqueSolutions(insertState,&mChildSeeds,mDistanceThreshold)) {
			DBGP("No.");
			delete insertState;
		} else {
			DBGP("Yes");
			//place a visual marker in the world
			mHand->getWorld()->getIVRoot()->addChild( insertState->getIVRoot() );			
			mChildSeeds.sort(GraspPlanningState::compareStates);
			DBGP("Queued...");
			while ((int)mChildSeeds.size() > mChildSeedSize) {
				delete(mChildSeeds.back());
				mChildSeeds.pop_back();
			}
			DBGP("Done.");
		}
	}

	mCurrentStep = mSimAnn->getCurrentStep();
	render();

	if (mCurrentStep % 100 == 0) {
		emit update();
		checkChildren();
	}
}

/*! Deletes any children that are done. If there are seeds available and
	emtpty children slots it also fires of new children. If a new child
	is fired and the main planner is still in the area, the main planner 
	is also reset so it goes plan somewhere else.
*/
void
GuidedPlanner::checkChildren()
{
	//first check if any children have stopped
	SimAnnPlanner *pl;
	std::vector<SimAnnPlanner*>::iterator it;
	it = mChildPlanners.begin();
	while(it!=mChildPlanners.end()) {
		pl = (*it);
		if ( !pl->isActive() ) {
			stopChild(pl);
			it = mChildPlanners.erase( it );
			delete pl;
			DBGA("Child stopped.");
		} else {
			it++;
		}
	}

	//if the planner is paused, do not fire any new children
	if (!isActive()) return;

	//now check if we have seeds and children available
	while ((int)mChildPlanners.size() < mMaxChildren && !mChildSeeds.empty()) {	
		GraspPlanningState *seed = mChildSeeds.front();
		mChildSeeds.pop_front();
		//avoid this state in the future since it will be searched by a child
		mAvoidList.push_back(seed);
		startChild(seed);
		if (mCurrentState->distance(seed) < mDistanceThreshold) {
			//re-anneal to make sure we search in some other area
			resetParameters();
		}
	}
}
