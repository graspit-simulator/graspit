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
// $Id: timeTest.cpp,v 1.13 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "timeTest.h"

#include "searchState.h"
#include "searchEnergy.h"
#include "robot.h"

//#define GRASPITDBG
#include "debug.h"

void
TimeTester::startPlanner()
{
	mCount = 0;
	mIllegalCount = 0;
	SimAnnPlanner::startPlanner();
}

void
TimeTester::mainLoop()
{
	GraspPlanningState *sn = new GraspPlanningState(mCurrentState);
	SearchVariable *var;
	double v;
	double r = ((float)rand()) / RAND_MAX - 0.5;
	for (int i=0; i<sn->getNumVariables(); i++) {
		var = sn->getVariable(i);
		if ( var->isFixed() ) continue;
		v = var->getValue() + r * ( var->mMaxJump ) * 0.1;
		var->setValue(v);
	}
	bool legal; double energy;
	mEnergyCalculator->analyzeState(legal, energy, sn);
	if (legal) mCount++;
	else mIllegalCount++;
	delete sn;
}

TimeTester *MTTester::startChild()
{
	TimeTester *child = new TimeTester(mHand);
	child->startThread();
	child->setEnergyType(ENERGY_AUTOGRASP_QUALITY);
	child->setModelState(mCurrentState);
	child->resetPlanner();
	return child;
}

void
MTTester::startPlanner()
{
	int numChildren = 3;

	assert(mCurrentState);
	mCurrentState->setRefTran(mHand->getTran(), false);

	mChildren.clear();
	for (int i=0; i<numChildren; i++) {
		TimeTester *child = startChild();
		mChildren.push_back( child );
	}
	DBGA("Children ready");
	mStartTime = clock();
	for (int i=0; i<(int)mChildren.size(); i++){
		mChildren[i]->startPlanner();
	}
	DBGA("Children started");
	setState(RUNNING);
}

void
MTTester::pausePlanner()
{
	for (int i=0; i<(int)mChildren.size(); i++){
		mChildren[i]->stopPlanner();
	}

	clock_t stopTime = clock();

	int count = 0, illegal = 0;
	for (int i=0; i<(int)mChildren.size(); i++){
		count += mChildren[i]->getCount();
		illegal += mChildren[i]->getIllegal();
		DBGA("Child " << i << ": " << mChildren[i]->getCount() << " grasps.")
	}

	double secs = (float)(stopTime - mStartTime) / CLOCKS_PER_SEC;

	DBGA(count << " grasps in " << secs << " seconds; avg is " << count/secs);
	DBGA("Illegal states: " << illegal);	
	setState(READY);
}
