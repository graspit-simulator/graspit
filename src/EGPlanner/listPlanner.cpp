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
// $Id: listPlanner.cpp,v 1.8 2009/07/02 21:05:12 cmatei Exp $
//
//######################################################################

#include "listPlanner.h"

#include "robot.h"
#include "searchState.h"
#include "searchEnergy.h"

//#define GRASPITDBG
#include "debug.h"

//this should be a parameter of the EGPlanner
//iros09
#define BEST_LIST_SIZE 2000

ListPlanner::ListPlanner(Hand *h)
{
	mHand = h;
	init();
	mEnergyCalculator = new SearchEnergy();
	mEnergyCalculator->disableRendering(false);
}

/*! The destructor also eliminates the list of input grasps, therefore
	care must be excercised when passing grasps to this class. 
*/
ListPlanner::~ListPlanner()
{
	while (!mInputList.empty()){
		delete mInputList.back();
		mInputList.pop_back();
	}
}

void 
ListPlanner::resetParameters(){
	EGPlanner::resetParameters();
	mPlanningIterator = mInputList.begin();
	if(mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState(*mPlanningIterator);
}

void 
ListPlanner::setInput(std::list<GraspPlanningState*> input)
{
	if (isActive()) {
		DBGA("Can not change input while planner is running");
		return;
	}
	while (!mInputList.empty()){
		delete mInputList.back();
		mInputList.pop_back();
	}
	mInputList = input;
	mMaxSteps = input.size();
	invalidateReset();
}

GraspPlanningState*
ListPlanner::getState(int index)
{
	std::list<GraspPlanningState*>::iterator it = mInputList.begin();
	int count = 0;
	while (it!=mInputList.end() && count < index) {
		count++; it++;
	}
	if (it==mInputList.end()) {
		DBGA("Requested grasp not in list");
		return NULL;
	}
	return *it;
}

void 
ListPlanner::testState(int index)
{
	GraspPlanningState *state = getState(index);
	if (!state) return;
	bool legal; double energy;
	mEnergyCalculator->analyzeState(legal, energy, state, false);
	DBGA("Energy: " << energy);
}

void
ListPlanner::showState(int index)
{
	GraspPlanningState *state = getState(index);
	if (!state) return;
	state->execute();
}

void
ListPlanner::prepareState(int index)
{
	showState(index);
	mHand->findInitialContact(200);
}

/*! The main planning function. Simply takes the next input grasp from the list,
	tries it and then places is in the output depending on the quality.
*/
void 
ListPlanner::mainLoop()
{
	//check if we are done
	if (mPlanningIterator==mInputList.end()) {
		mCurrentStep = mMaxSteps+1;
		return;
	}

	//analyze the current state
	//we don't allow it to leave the hand in the analysis posture
	//so that after dynamics object gets put back
	bool legal; double energy;
	PRINT_STAT(mOut, mCurrentStep);
	mEnergyCalculator->analyzeState(legal, energy, *mPlanningIterator, true);

	//for rendering purposes; will see later if it's needed
	mCurrentState->copyFrom(*mPlanningIterator);
	mCurrentState->setLegal(legal);

	//put a copy of the result in list if it's legal and there's room or it's 
	//better than the worst solution so far
	//this whole thing could go into a higher level fctn in EGPlanner
	if (legal) {
		double worstEnergy;
		if ((int)mBestList.size() < BEST_LIST_SIZE) worstEnergy = 1.0e5;
		else worstEnergy = mBestList.back()->getEnergy();
		if (energy < worstEnergy) {
			GraspPlanningState *insertState = new GraspPlanningState(*mPlanningIterator);
			insertState->setEnergy(energy);
			insertState->setItNumber(mCurrentStep);
			DBGP("Solution at step " << mCurrentStep);
			mBestList.push_back(insertState);
			mBestList.sort(GraspPlanningState::compareStates);
			while ((int)mBestList.size() > BEST_LIST_SIZE) {
				delete(mBestList.back());
				mBestList.pop_back();
			}
		}
	}

	//advance the planning iterator
	mPlanningIterator++;
	mCurrentStep++;
	emit update();
	PRINT_STAT(mOut, std::endl);
}

void 
ListPlanner::showVisualMarkers(bool show)
{
	std::list<GraspPlanningState*>::iterator it;
	for (it = mInputList.begin(); it!=mInputList.end(); it++) {
		if (show) {
			(*it)->showVisualMarker();
		} else {
			(*it) ->hideVisualMarker();
		}
	}
}
