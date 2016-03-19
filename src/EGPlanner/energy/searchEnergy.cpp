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
#include "contact.h"
#include "world.h"
#include "quality.h"
#include "searchState.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "matrix.h"

#include "include/EGPlanner/energy/contactEnergy.h"
#include "include/EGPlanner/energy/potentialQualityEnergy.h"
#include "include/EGPlanner/energy/guidedPotentialQualityEnergy.h"
#include "include/EGPlanner/energy/autograspQualityEnergy.h"
#include "include/EGPlanner/energy/guidedAutoGraspEnergy.h"
#include "include/EGPlanner/energy/dynamicAutograspEnergy.h"
#include "include/EGPlanner/energy/compliantEnergy.h"
#include "include/EGPlanner/energy/strictAutoGraspEnergy.h"
#include "include/EGPlanner/energy/closureSearchEnergy.h"

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
    mType = ENERGY_CONTACT; //default
    mContactType = CONTACT_LIVE; //default
    mVolQual = NULL;
    mEpsQual = NULL;
    mDisableRendering = true;
    mOut = NULL;
}

void
SearchEnergy::createQualityMeasures()
{
    if (mVolQual) delete mVolQual;
    if (mEpsQual) delete mEpsQual;
    mVolQual = new QualVolume( mHand->getGrasp(), QString("SimAnn_qvol"),"L1 Norm");
    mEpsQual = new QualEpsilon( mHand->getGrasp(), QString("SimAnn_qeps"), "L1 Norm");
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
    delete mVolQual;
    delete mEpsQual;
}

bool
SearchEnergy::legal() const
{	
    //hack for iros09
    //the compliant planners do their own checks
    if (mType == ENERGY_COMPLIANT || mType == ENERGY_DYNAMIC) return true;

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
    setHandAndObject(h,o);

    if (noChange) {
        h->saveState();
    }

    if ( !legal() ) {
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
    Hand *h = state->getHand();
    setHandAndObject( h, state->getObject() );
    h->saveState();
    transf objTran = state->getObject()->getTran();

    bool render = h->getRenderGeometry();
    if( mDisableRendering) {
        h->setRenderGeometry(false);
    }

    if ( !state->execute() || !legal() ) {
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

    if (render && mDisableRendering) h->setRenderGeometry(true);
    return;
}


double SearchEnergy::getEpsQual(){
    mHand->getWorld()->findAllContacts();
    mHand->getWorld()->updateGrasps();
    return mEpsQual->evaluate();
}

double SearchEnergy::getVolQual(){
    mHand->getWorld()->findAllContacts();
    mHand->getWorld()->updateGrasps();
    return mVolQual->evaluate();
}

SearchEnergy * SearchEnergy::getSearchEnergy(SearchEnergyType type)
{
    switch (type)
    {
    case ENERGY_CONTACT:
        return new ContactEnergy();
    case ENERGY_POTENTIAL_QUALITY:
         return new PotentialQualityEnergy();
    case ENERGY_AUTOGRASP_QUALITY:
        return new AutoGraspQualityEnergy();
    case ENERGY_CONTACT_QUALITY:
        return new GuidedPotentialQualityEnergy();
    case ENERGY_GUIDED_AUTOGRASP:
        return new GuidedAutoGraspQualityEnergy();
    case ENERGY_STRICT_AUTOGRASP:
        return new StrictAutoGraspEnergy();
    case ENERGY_COMPLIANT:
        return new CompliantEnergy();
    case ENERGY_DYNAMIC:
        return new DynamicAutoGraspEnergy();
    case ENERGY_CLOSURE:
        return new ClosureSearchEnergy();
    default:
        std::cout << "INVALID SEARCH ENERGY TYPE" << std::endl;
        return NULL;
    }
}


