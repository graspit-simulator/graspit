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

#include "graspit/EGPlanner/loopPlanner.h"

#include "graspit/robot.h"
#include "graspit/world.h"
#include "graspit/EGPlanner/simAnn.h"
#include "graspit/EGPlanner/energy/searchEnergy.h"
#include "graspit/EGPlanner/searchState.h"

//#define GRASPITDBG
#include "graspit/debug.h"

LoopPlanner::LoopPlanner(Hand *h)
{
  mHand = h;
  init();

  mEnergyCalculator = SearchEnergy::getSearchEnergy("GUIDED_POTENTIAL_QUALITY_ENERGY");
  mEnergyCalculator->setAvoidList(&mAvoidList);

  mSimAnn = new SimAnn();
  mSimAnn->setParameters(SimAnnParams::ANNEAL_LOOP());
  mRepeat = true;

  mDistanceThreshold = 0.1f;
  mEnergyCalculator->setThreshold(mDistanceThreshold);

  mSaveThreshold = 10.0f;
}

void
LoopPlanner::setEnergyType(std::string s)
{
  assert(mEnergyCalculator);
  if (!mEnergyCalculator->isType(s))
  {
    delete mEnergyCalculator;
    mEnergyCalculator = SearchEnergy::getSearchEnergy(s);
    mEnergyCalculator->setThreshold(mDistanceThreshold);
    mEnergyCalculator->setAvoidList(&mAvoidList);
  }
}

void LoopPlanner::setDistanceThreshold(float t)
{
  mDistanceThreshold = t;
  mEnergyCalculator->setThreshold(mDistanceThreshold);
}

LoopPlanner::~LoopPlanner()
{
}

void LoopPlanner::resetParameters()
{
  while (!mBestList.empty()) {
    GraspPlanningState *s = mBestList.front();
    mBestList.pop_front();
    if (s->getEnergy() > mSaveThreshold) {
      delete s;
    } else {
      mAvoidList.push_back(s);
    }
  }
  SimAnnPlanner::resetParameters();
  Q_EMIT loopUpdate();
}

const GraspPlanningState *
LoopPlanner::getGrasp(int i)
{
  DBGP("Loop get grasp");
  assert(i >= 0 && i < (int)mAvoidList.size());
  std::list<GraspPlanningState *>::iterator it = mAvoidList.begin();
  for (int k = 0; k < i; k++) {
    it++;
  }
  return (*it);
}

void
LoopPlanner::clearSolutions()
{
  SimAnnPlanner::clearSolutions();
  while (!mAvoidList.empty()) {
    delete(mAvoidList.back());
    mAvoidList.pop_back();
  }
}
