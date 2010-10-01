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
