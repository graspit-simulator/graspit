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
// $Id: loopPlanner.cpp,v 1.7 2009/10/08 16:20:31 cmatei Exp $
//
//######################################################################

#include "loopPlanner.h"

#include "robot.h"
#include "world.h"
#include "simAnn.h"
#include "searchEnergy.h"
#include "searchState.h"

//#define GRASPITDBG
#include "debug.h"

LoopPlanner::LoopPlanner(Hand *h)
{
	mHand = h;
	init();
	mEnergyCalculator = new ClosureSearchEnergy();
	mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
	((ClosureSearchEnergy*)mEnergyCalculator)->setAvoidList( &mAvoidList );

	mSimAnn = new SimAnn();
	mSimAnn->setParameters(ANNEAL_LOOP);
	mRepeat = true;

	mDistanceThreshold = 0.1f;
	((ClosureSearchEnergy*)mEnergyCalculator)->setThreshold(mDistanceThreshold);
}

LoopPlanner::~LoopPlanner()
{
}

/*! The criterion for taking a solution and placing it in the avoid list
	is currently hard-coded in.
*/
void LoopPlanner::resetParameters()
{
	while (!mBestList.empty()) {
		GraspPlanningState *s = mBestList.front();
		mBestList.pop_front();
		if (s->getEnergy() > 10.0) {
			delete s;
		} else {
			mAvoidList.push_back( s );
		}
	}
	SimAnnPlanner::resetParameters();
	emit loopUpdate();
}

const GraspPlanningState* 
LoopPlanner::getGrasp(int i)
{
	DBGP("Loop get grasp");
	assert (i>=0 && i<(int)mAvoidList.size());
	std::list<GraspPlanningState*>::iterator it = mAvoidList.begin();
	for (int k=0; k<i; k++) {
		it++;
	}
	return (*it);
}

void
LoopPlanner::clearSolutions()
{
	SimAnnPlanner::clearSolutions();
	while ( !mAvoidList.empty() ) {
		delete(mAvoidList.back());
		mAvoidList.pop_back();
	}
}
