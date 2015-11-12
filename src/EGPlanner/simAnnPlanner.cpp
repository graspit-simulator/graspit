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
// $Id: simAnnPlanner.cpp,v 1.16 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "simAnnPlanner.h"

#include "searchState.h"
#include "searchEnergy.h"
#include "simAnn.h"
#include "robot.h"
#include "world.h"

//#define GRASPITDBG
#include "debug.h"

//! How many of the best states are buffered. Should be a parameter
#define BEST_LIST_SIZE 20
//! Two states within this distance of each other are considered to be in the same neighborhood
#define DISTANCE_THRESHOLD 0.3

SimAnnPlanner::SimAnnPlanner(Hand *h)
{
	mHand = h;
	init();
	mEnergyCalculator = new SearchEnergy();
	mSimAnn = new SimAnn();
	//mSimAnn->writeResults(true);
}

SimAnnPlanner::~SimAnnPlanner()
{
	if (mSimAnn) delete mSimAnn;
}

void
SimAnnPlanner::setAnnealingParameters(AnnealingType y) {
	if (isActive()) {
		DBGA("Stop planner before setting ann parameters");
		return;
	}
	mSimAnn->setParameters(y);
}

void
SimAnnPlanner::resetParameters()
{
	EGPlanner::resetParameters();
	mSimAnn->reset();
	mCurrentStep = mSimAnn->getCurrentStep();
	mCurrentState->setEnergy(1.0e8);
}

bool
SimAnnPlanner::initialized()
{
	if (!mCurrentState) return false;
	return true;
}

void
SimAnnPlanner::setModelState(const GraspPlanningState *modelState)
{
	if (isActive()) {
		DBGA("Can not change model state while planner is running");
		return;
	}

	if (mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState(modelState);
	mCurrentState->setEnergy(1.0e5);
	//my hand might be a clone
	mCurrentState->changeHand(mHand, true);

	if (mTargetState && (mTargetState->readPosition()->getType() != mCurrentState->readPosition()->getType() ||
						 mTargetState->readPosture()->getType() != mCurrentState->readPosture()->getType() ) ) {
		delete mTargetState; mTargetState = NULL;
    }
	if (!mTargetState) {
		mTargetState = new GraspPlanningState(mCurrentState);
		mTargetState->reset();
		mInputType = INPUT_NONE;
	}
	invalidateReset();
}

void
SimAnnPlanner::mainLoop()
{
	GraspPlanningState *input = NULL;
	if ( processInput() ) {
		input = mTargetState;
	}

	//call sim ann
	//SimAnn::Result result = mSimAnn->iterate(mCurrentState, mEnergyCalculator, input, mHand->getWorld()->getRobot(1));
	/* record hand's current position and fix it there, without the following the hand is always put at the origin of the
	object's system */
	if(mHand->getName() == QString("snake_with_gripper")){
		mCurrentState->setRefTran(mHand->getTran());
		mCurrentState->setBaseFixed(true);
	}
	SimAnn::Result result = mSimAnn->iterate(mCurrentState, mEnergyCalculator, input, NULL);
	if ( result == SimAnn::FAIL) {
		DBGP("Sim ann failed");
		return;
	}
	DBGP("Sim Ann success");

	//put result in list if there's room or it's better than the worst solution so far
	double worstEnergy;
	if ((int)mBestList.size() < BEST_LIST_SIZE) worstEnergy = 1.0e5;
	else worstEnergy = mBestList.back()->getEnergy();
	if (result == SimAnn::JUMP && mCurrentState->getEnergy() < worstEnergy) {
		GraspPlanningState *insertState = new GraspPlanningState(mCurrentState);
		//but check if a similar solution is already in there
		if (!addToListOfUniqueSolutions(insertState,&mBestList,0.2)) {
			delete insertState;
		} else {
			mBestList.sort(GraspPlanningState::compareStates);
			while ((int)mBestList.size() > BEST_LIST_SIZE) {
				delete(mBestList.back());
				mBestList.pop_back();
			}
		}
	}
	render();
	mCurrentStep = mSimAnn->getCurrentStep();
	if (mCurrentStep % 100 == 0 && !mMultiThread) emit update();
	if (mMaxSteps == 200) {DBGP("Child at " << mCurrentStep << " steps");}
}
