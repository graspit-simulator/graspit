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
// $Id: egPlanner.cpp,v 1.40 2009/12/01 18:54:03 cmatei Exp $
//
//######################################################################

#include <Inventor/sensors/SoIdleSensor.h>

#include "egPlanner.h"
#include "searchState.h"
#include "searchEnergy.h"
#include "robot.h"
#include "barrett.h"
#include "robotiq.h"
#include "pr2Gripper.h"
#include "body.h"
#include "simAnn.h"
#include "world.h"
#include "gloveInterface.h" //for glove input
#include "eigenGrasp.h" //for glove input
#include "collisionInterface.h"

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(EG_PLANNER);

#define BEST_LIST_SIZE 20

EGPlanner::EGPlanner(Hand *h)
{
	mHand = h;
	init();
	mEnergyCalculator = new SearchEnergy();
}

/*! Also sets the state of the planner to INIT, which is default
	initialization.
*/
void
EGPlanner::init()
{	
	mProfileInstance = new Profiling::ProfileInstance();

	mIdleSensor = NULL;
	mCurrentState = NULL;
	mTargetState = NULL;
	mInputType = INPUT_NONE;
	mRenderType = RENDER_LEGAL;
	mRepeat = false;
	mCurrentStep = 0;
	mRenderCount = 0;
	mLastRenderState = NULL;
	mMaxSteps = 100000;
	mMaxTime = -1;
	mMultiThread = false;
	mState = INIT;
	mUsesClone = false;
	mOut = NULL;
}

EGPlanner::~EGPlanner()
{
        delete mProfileInstance;
	clearSolutions();
	if (mEnergyCalculator) delete mEnergyCalculator;
	if (mUsesClone) {
		mHand->getWorld()->destroyElement(mHand);
	}
	if (mCurrentState) delete mCurrentState;
	if (mTargetState) delete mTargetState;
	if (mIdleSensor) delete mIdleSensor;
}

/*! Set the current state of the planner. It will accept any state, as
	long as the current state is not DONE or EXITED. Once the planner 
	is DONE it can only go to EXITED when the thread stops spinning. 
	DONE is essentially just a flag that is guaranteed to stop the 
	thread. There's no going back...
*/
void
EGPlanner::setState(PlannerState s)
{
	if (mMultiThread) mControlMutex.lock();
	if (mState == DONE && s != DONE && s != EXITED) {
		DBGA("Planner is DONE; change state no longer possible");
	} else if (mState == EXITED && s != EXITED) {
		DBGA("Planner is EXITED; change state no longer possible");
	} else {
		mState = s;
	}
	if (mMultiThread) mControlMutex.unlock();
}

PlannerState
EGPlanner::getState()
{
	if (mMultiThread) mControlMutex.lock();
	PlannerState s = mState;
	if (mMultiThread) mControlMutex.unlock();
	return s;
}

/*! This is the part of resetPlanner() that can also be called while
	the planner is running to cause it to start from the beginning,
	without affecting the currently computed solutions.
*/
void
EGPlanner::resetParameters()
{
	mCurrentStep = 0;
	mRenderCount = 0;
}

/*! The only function available externally to set the planner to the READY
	state. However, before that can actually happen, the planner must receive
	all needed input, which depends on the type of implementation.
	
	It will clear any solutions saved so far and re-start the timer and the 
	step count as well. It can not be used while the planner is running, you 
	must pause the planner first. This function will also move the planner 
	from INIT to READY as long as all the necessary conditions for initilized() 
	have been met.
*/
bool
EGPlanner::resetPlanner()
{
	if ( getState() == RUNNING || getState() == DONE) {
		DBGA("Planner is either running or dead; cannot reset");
		return false;
	}
	if (!initialized()) {
		DBGA("Planner is not ready; not all necessary information has been set");
		return false;
	}
	clearSolutions();
	resetParameters();

	mProfileInstance->reset();

	setState(READY);
	if (!mMultiThread) emit update();
	return true;
}

bool
EGPlanner::checkTerminationConditions()
{
	if (!isActive()) return true;
	bool termination = false;
	//max steps equal to -1 means run forever
	if (mMaxSteps != -1 && mCurrentStep >= mMaxSteps){ 
		if (!mRepeat) {
			pausePlanner();
			termination = true;
		} else {
			resetParameters();
		}
		if (!mMultiThread) {
			emit update();
		}
	} else if (mMaxTime != -1 ) {
		//check time limit
		//for now exceeding the time limit simply kills it for good
		if (getRunningTime() > mMaxTime) {
			termination = true;
			stopPlanner();
		}
	}
	if (termination) {
		emit complete();
	}
	return termination;
}

void
EGPlanner::createAndUseClone()
{
  if (isActive()) {
    DBGA("Can not change hands while planner is running");
    return;
  }
  if (mMultiThread) {
    //let collision detection know this is a new thread
    mHand->getWorld()->getCollisionInterface()->newThread();
  }
  Hand *clone;
  if (mHand->isA("Barrett")) {
    clone = new Barrett( mHand->getWorld(), "Barrett clone");
  } else if (mHand->isA("Pr2Gripper")) {
    clone = new Pr2Gripper( mHand->getWorld(), "PR2 Gripper clone");
  } else if (mHand->isA("RobotIQ")) {
    clone = new RobotIQ( mHand->getWorld(), "RobotIQ clone");
  } else {
    clone = new Hand(mHand->getWorld(), "Hand clone");
  }
  clone->cloneFrom(mHand);
  clone->setRenderGeometry(false);
  clone->showVirtualContacts(false);
  if (mMultiThread) {
    //we do not want to add the robot to the scene graph from here
    //it is ideal not to touch the scene graph from outside the main thread
    mHand->getWorld()->addRobot(clone,false);
  } else {
    mHand->getWorld()->addRobot(clone,true);
  }
  mHand->getWorld()->toggleCollisions(false, mHand, clone);
  clone->setTran( mHand->getTran() );
  mHand = clone;
  mUsesClone = true;
  if (mCurrentState) mCurrentState->changeHand(mHand);
}

/*! The planner can remove the clone from the scene graph. The clone 
	still exists in the collision detection system and can be used 
	normally, it is just not rendered anymore. HIGHLY RECOMMENDED for 
	multi-threaded operation, since Inventor is not thread-safe. The 
	clone should be rendered just for debug purposes.
*/
void
EGPlanner::showClone(bool s)
{
	if (!mUsesClone) {
		DBGA("Planner is not using a clone");
		return;
	}
	if (!s) {
		mHand->getWorld()->removeElementFromSceneGraph(mHand);
	} else {
		mHand->getWorld()->addElementToSceneGraph(mHand);
	}
}

void
EGPlanner::setEnergyType(SearchEnergyType s)
{
	assert (mEnergyCalculator);
	mEnergyCalculator->setType(s);
}

void
EGPlanner::setContactType(SearchContactType c)
{
	assert (mEnergyCalculator);
	mEnergyCalculator->setContactType(c);
}

void
EGPlanner::sensorCB(void *data, SoSensor *)
{
	EGPlanner *planner = (EGPlanner*)data;
	if (planner->checkTerminationConditions()) {
		//if the planner has stopped we are done
		return;
	}
	planner->mainLoop();
	planner->mIdleSensor->schedule();
}

void EGPlanner::threadLoop()
{
	bool done = false;
	while (!done) {
		PlannerState s = getState();
		switch(s) {
               		case STARTING_THREAD: //do nothing
			  break;
                  	case INIT:
				sleep(0.1);
				break;
			case READY:
				sleep(0.1);
				break;
			case RUNNING:
				mainLoop();
				break;
			case DONE:
				done = true;
				break;
         		case EXITED: //Do nothing
			        break;
		}
		if (!done) checkTerminationConditions();
	}
	setState(EXITED);
	DBGP("Thread is done!");
}


double
EGPlanner::getRunningTime()
{
  return 1.0e-6 * mProfileInstance->getTotalTimeMicroseconds();
}

void
EGPlanner::run()
{
	mMultiThread = true;
	mRenderType = RENDER_NEVER;
	//threaded planners always use cloned hands
	createAndUseClone();
	//signal that initialization is ready
	setState(INIT);
	threadLoop();
}

/*! Automatically calls createAndUseClone(). Does not return until new 
	thread is up and ready. However, it does not also start planning, it
	just spins. Use startPlanner(), like you would for single-threaded
	operation, for that.
*/
void
EGPlanner::startThread()
{
	if (mMultiThread) {
		DBGA("Can not start thread; already multi-threaded");
	}
	if (getState()!=INIT) {
		DBGA("Can not start thread; state is not INIT");
	}
	setState(STARTING_THREAD);
	start();
	mMultiThread = true;
	//wait for thread initialization to finish
	while (getState() == STARTING_THREAD);
	//the new thread will automatically create a clone and add it to the world
	//but not to the scene graph. We add it to the scene graph here so it's
	//done in the main thread
	mHand->getWorld()->addElementToSceneGraph(mHand);	
}

void
EGPlanner::startPlanner()
{
	if ( getState() != READY ) {
		DBGA("Planner not ready to start!");
		return;
	}
	if (!mMultiThread) {
		mHand->showVirtualContacts(false);
		mIdleSensor = new SoIdleSensor(sensorCB, this);
		mIdleSensor->schedule();
	}
	PROF_RESET_ALL;
	PROF_START_TIMER(EG_PLANNER);

	mProfileInstance->startTimer();

	setState( RUNNING );
}

/*! After this is called, the planner can no longer be re-started. 
	This differentiation is needed mainly for the multi-threaded case: 
	this function stops the planner's thread.
*/
void
EGPlanner::stopPlanner()
{
	if (getState()==DONE || getState()==EXITED) return;
	//this will stop the planner REGARDLESS of what state it is in!
	pausePlanner();
	//this also finishes the thread
	setState(DONE);
	if (mMultiThread) {
		DBGP("Waiting for exit");
		//wait for the thread to stop spinning
		while (getState()!=EXITED);
		DBGP("Exited");
	}
}

void
EGPlanner::pausePlanner()
{
	if (getState() != RUNNING) return;
	mProfileInstance->stopTimer();
	if (!mMultiThread) {
		if (mIdleSensor) delete mIdleSensor;
		mIdleSensor = NULL;
		mHand->showVirtualContacts(true);
	} 
	setState(READY);
	PROF_STOP_TIMER(EG_PLANNER);
	PROF_PRINT_ALL;
	if (!mMultiThread) emit complete();
}

void
EGPlanner::render()
{
	if (mMultiThread) {
		//for now, multi-threaded planners are not allowed to render
		//rendering should only be done by the main thread
		return;
	}
	if (mRenderType == RENDER_BEST) {
		if ( mBestList.empty() ) return;
		if ( mLastRenderState == mBestList.front() ) return;		
		mLastRenderState = mBestList.front();
		mBestList.front()->execute();
	} else if (mRenderType == RENDER_LEGAL) {
		if (mRenderCount >= 20) {
			DBGP("Render: geom is " << mHand->getRenderGeometry() );
			mRenderCount = 0;
			if ( mCurrentState && mCurrentState->isLegal() ) mCurrentState->execute();
		} else mRenderCount++;
	} else if (mRenderType==RENDER_ALWAYS) {
		mCurrentState->execute();
	} else if ( mRenderType == RENDER_NEVER ) {
		return;
	}
}

const GraspPlanningState* 
EGPlanner::getGrasp(int i)
{
	assert (i>=0 && i<(int)mBestList.size());
	std::list<GraspPlanningState*>::iterator it = mBestList.begin();
	for (int k=0; k<i; k++) {
		it++;
	}
	return (*it);
}

void 
EGPlanner::showGrasp(int i)
{
	assert (i>=0 && i<getListSize());
	const GraspPlanningState *s = getGrasp(i);
	s->execute();
	bool l; double e;
	mEnergyCalculator->analyzeCurrentPosture(s->getHand(), s->getObject(), l, e, false);
	DBGA("Re-computed energy: " << e);
}

void
EGPlanner::clearSolutions()
{
	while ( !mBestList.empty() ) {
		delete(mBestList.back());
		mBestList.pop_back();
	}
}
bool
EGPlanner::setInput(unsigned char input, bool on)
{
	if (input == INPUT_NONE) {
		mInputType = INPUT_NONE;
	} else if (on) {
		mInputType = mInputType | input;
	} else {
		mInputType = mInputType & (~input);
	}
	return true;	
}

bool
EGPlanner::processInput()
{
	assert(mTargetState);
	if (mInputType & INPUT_GLOVE) {
		//if we are using the glove as input, the mTargetState needs to be 
		//updated all the time. Since the planner has the callback loop, 
		//it's the planner who has to do it, not the dialog
		assert(mHand->getGloveInterface());
		double *gloveDOF = new double [mHand->getNumDOF()];
		for (int d=0; d<mHand->getNumDOF(); d++) {
			if (mHand->getGloveInterface()->isDOFControlled(d)) {
				gloveDOF[d] = mHand->getGloveInterface()->getDOFValue(d);
			} else {
				gloveDOF[d] = mHand->getDOF(d)->getVal();
			}
		}
		mTargetState->getPosture()->storeHandDOF(gloveDOF);
		delete [] gloveDOF;		
	}
	//for input from the Flock of Birds:
	//nothing to do here. The way this works, is that the reference hand 
	//is controlled by the flock of birds. The OnLinePlanner is designed 
	//to handle this, see that class for reference.
	return true;
}

double
EGPlanner::stateDistance(const GraspPlanningState *s1, const GraspPlanningState *s2) {
	return s1->distance(s2);
}

/*!	Attempts to maintain a list of unique solutions. Therefore, whenever 
	a new state is added to the list, we check if any of the states that 
	are already in the list are within a given distance of the new state.
	If so, the best one is kept and the other one is thrown away. This 
	method does not gurantee unique states, but	it comes close and runs 
	in linear time for each addition, rather than square time for 
	maintenance.
*/
bool
EGPlanner::addToListOfUniqueSolutions(GraspPlanningState *s, std::list<GraspPlanningState*> *list, double distance)
{
	std::list<GraspPlanningState*>::iterator it;
	it = list->begin();
	bool add = true;
	while (it!=list->end()) {
		double d = stateDistance(s,*it);
		if ( fabs(d) < distance ) {
			DBGP("Distance: " << fabs(d) );
			//states are close to each other
			if ( s->getEnergy() < (*it)->getEnergy() ) {
				//new state is better; remove old one from list
				delete (*it);
				it = list->erase(it);
				DBGP("Old state removed");
			} else {
				//old state is better; we don't want to add the new one
				add = false;
				break;
			}
		} else {
			//states are not close, proceed through the list
			it++;
		}
	}
	if (add) {
		list->push_back(s);
	}
	return add;
}

void 
EGPlanner::setStatStream(std::ostream *out) const 
{
	mOut = out; 
	assert(mEnergyCalculator);
	mEnergyCalculator->setStatStream(out);
}
