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
// $Id: timeTest.cpp,v 1.14 2009/12/01 18:54:03 cmatei Exp $
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

	int count = 0, illegal = 0;
	for (int i=0; i<(int)mChildren.size(); i++){
		count += mChildren[i]->getCount();
		illegal += mChildren[i]->getIllegal();
		DBGA("Child " << i << ": " << mChildren[i]->getCount() << " grasps.")
	}

	DBGA(count << " grasps.");
	DBGA("Illegal states: " << illegal);	
	setState(READY);
}
