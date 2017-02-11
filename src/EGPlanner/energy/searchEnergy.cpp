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
// $Id: searchEnergy.cpp,v 1.42 2009/09/13 19:57:38 hao Exp $
//
//######################################################################

#include "searchEnergy.h"

#include <time.h>

#include "robot.h"
#include "barrett.h"
#include "body.h"
#include "grasp.h"
#include "contact/contact.h"
#include "world.h"
#include "quality.h"
#include "searchState.h"
#include "graspitCore.h"
#include "ivmgr.h"
#include "matrix.h"

#include "EGPlanner/energy/contactEnergy.h"
#include "EGPlanner/energy/potentialQualityEnergy.h"
#include "EGPlanner/energy/guidedPotentialQualityEnergy.h"
#include "EGPlanner/energy/autoGraspQualityEnergy.h"
#include "EGPlanner/energy/guidedAutoGraspEnergy.h"
#include "EGPlanner/energy/dynamicAutoGraspEnergy.h"
#include "EGPlanner/energy/compliantEnergy.h"
#include "EGPlanner/energy/strictAutoGraspEnergy.h"

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(QS);

//todo move this out of here
const double unbalancedForceThreshold = 1.0e10;

SearchEnergy::SearchEnergy()
{
  mHand = NULL;
  mObject = NULL;
  mType = "CONTACT_ENERGY"; //default
  mContactType = CONTACT_LIVE; //default
  mVolQual = NULL;
  mEpsQual = NULL;
  mDisableRendering = true;
  mOut = NULL;
  mThreshold = 0;
  mAvoidList = NULL;
}

void
SearchEnergy::createQualityMeasures()
{
  if (mVolQual) { delete mVolQual; }
  if (mEpsQual) { delete mEpsQual; }
  mVolQual = new QualVolume(mHand->getGrasp(), QString("SimAnn_qvol"), "L1 Norm");
  mEpsQual = new QualEpsilon(mHand->getGrasp(), QString("SimAnn_qeps"), "L1 Norm");
  DBGP("Qual measures created");
}

void
SearchEnergy::setHandAndObject(Hand *h, Body *o)
{
  if (mHand != h) {
    mHand = h;
    createQualityMeasures();
  }
  mObject = o;
}

SearchEnergy::~SearchEnergy()
{
  if (mVolQual) { delete mVolQual; }
  if (mEpsQual) { delete mEpsQual; }
}

bool
SearchEnergy::legal() const
{
  //hack for iros09
  //the compliant planners do their own checks
  if (mType == "COMPLIANT_ENERGY" || mType == "DYNAMIC_AUTO_GRASP_ENERGY") { return true; }

  //no check at all
  //return true;

  //full collision detection
  //if the hand is passed as an argument, this should only check for collisions that
  //actually involve the hand
  return mHand->getWorld()->noCollision(mHand);

  /*
  //check only palm
  if ( mHand->getWorld()->getDist( mHand->getPalm(), mObject) <= 0) return false;
  return true;
  */
}

void
SearchEnergy::analyzeCurrentPosture(Hand *h, Body *o, bool &isLegal, double &stateEnergy, bool noChange)
{
  setHandAndObject(h, o);

  if (noChange) {
    h->saveState();
  }

  if (!legal()) {
    isLegal = false;
    stateEnergy = 0;
  } else {
    isLegal = true;
    stateEnergy = energy();
  }

  if (noChange) {
    h->restoreState();
  }
}

void SearchEnergy::analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange)
{
  if (mAvoidList) {
    std::list<GraspPlanningState *>::const_iterator it; int i = 0;
    for (it = mAvoidList->begin(); it != mAvoidList->end(); it++) {
      if ((*it)->distance(state) < mThreshold) {
        isLegal = false;
        stateEnergy = 0.0;
        DBGP("State rejected; close to state " << i);
        return;
      }
      i++;
    }
  }

  Hand *h = state->getHand();
  setHandAndObject(h, state->getObject());
  h->saveState();
  transf objTran = state->getObject()->getTran();

  bool render = h->getRenderGeometry();
  if (mDisableRendering) {
    h->setRenderGeometry(false);
  }

  if (!state->execute() || !legal()) {
    isLegal = false;
    stateEnergy = 0;
  } else {
    isLegal = true;
    stateEnergy = energy();
  }

  if (noChange || !isLegal) {
    h->restoreState();
    state->getObject()->setTran(objTran);
  }

  if (render && mDisableRendering) { h->setRenderGeometry(true); }
  return;
}


double SearchEnergy::getEpsQual() {
  mHand->getWorld()->findAllContacts();
  mHand->getWorld()->updateGrasps();
  return mEpsQual->evaluate();
}

double SearchEnergy::getVolQual() {
  mHand->getWorld()->findAllContacts();
  mHand->getWorld()->updateGrasps();
  return mVolQual->evaluate();
}

SearchEnergy *SearchEnergy::getSearchEnergy(std::string type)
{
  SearchEnergy *se = SearchEnergyFactory::getInstance()->createEnergy(type);
  se->setType(type);
  return se;
}




