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
// $Id: egPlanner.h,v 1.27 2009/12/01 18:44:56 cmatei Exp $
//
//######################################################################

#ifndef _egplanner_h_
#define _egplanner_h_

#include <vector>
#include <list>
#include <ostream>
#include <time.h>

#include <QThread>
#include <QMutex>
#include <QObject>

#include "search.h"

class Hand;
class Body;
class GraspPlanningState;
class SoSensor;
class SearchEnergy;

namespace Profiling {
  class ProfileInstance;
}

enum PlannerState{INIT, READY, RUNNING, DONE, EXITED, STARTING_THREAD};

/*!	The EGPlanner is one of the main classes that are available in GraspIt! 
	for doing grasp planning. Unlike the name suggests, it is not necessarily 
	restricted to running in EigenGrasp space. It is designed to take care of 
	loops, lists of solutions, etc. and allow sub-classes to just focus on the 
	planning itself. When subclassing from this class, you just have to define 
	the mainLoop() function which here is pure abstract. 

	It can operate either in a single-thread fashion, where the looping 
	functionality is provided using a Coin IdleSensor, or it can start it's own 
	thread which spins forever and calls the mainLoop().

	It can either operate on the GraspIt hand that is passed as an argument, 
	or it can create its own "clone" of the hand to do the planning on. If 
	multi-threaded, the planner will ALWAYS run on a clone.
*/
class EGPlanner : public QThread
{
	Q_OBJECT
protected:
	EGPlanner(){}
	//!Contains initialization that is COMMON between this and all subclasses. 
	/*! Class-specific initialization will go in the constructor. */
	void init();

	//! This is the hand that the planner is using. Might be a real hand from the GraspIt world or a clone of one
	Hand *mHand;
	bool mUsesClone;

	//! This is where the planner (and its SearchEnergy) will spit their output
	mutable std::ostream *mOut;
	//! The current state of the planner (see the HandObjectState class for details)
	GraspPlanningState *mCurrentState;
	//! The instance of the SearchEnergy class that this planner can use to compute the "quality" of a state
	SearchEnergy *mEnergyCalculator;

	//! How many iterations this planner has done since the last reset
	int mCurrentStep;
	//! Maximum number of iterations allowed. mMaxSteps = -1 means run forever.
	int	mMaxSteps;
	//! Tells us what happens when mMaxSteps have been exceeded: the planner will either stop or restart
	bool mRepeat;

	//! These options decide when and what the planner should render.
	int mRenderType, mRenderCount;
	//! The last rendered state
	const GraspPlanningState *mLastRenderState;

	//! A decision is made whether to put in a redraw request to the scene graph.
	/*! WARNING: when multi-threaded, it is best to avoid ALL rendering 
		requests issued from inside the planner. */
	void render();

	//! Maximum time allowed, in seconds. mMaxTime = -1 means no time limit
	double mMaxTime;
	//! The profiling instance used to time this planner
	Profiling::ProfileInstance *mProfileInstance;

	//! If used, this is a target state, or an "input" state provided by the user as a guideline
	GraspPlanningState *mTargetState;
	//! Tells the planner if it should process its own input state, based on Cyberglove, Flock of Birds etc.
	unsigned char mInputType;
	//! Sets the values in the mTargetState based in the mInputType flag
	virtual bool processInput();

	//! Resets all the inner parameters. Can be called while planner is running as well
	virtual void resetParameters();

	//! Checks whether we should terminate the planner, either because maxSteps or maxTime have been exceeded
	virtual bool checkTerminationConditions();

	//! For single-threaded operation, using an idle sensor
	SoSensor *mIdleSensor;
	//! The idle sensor callback, which calls mainLoop() during single-threaded operation
	static void sensorCB(void *data,SoSensor*);

	//! Flag that indicates single- or multi-threaded operation
	bool mMultiThread;
	//! Mutex for synchronizing threads
	QMutex mControlMutex;
	//! The entry point of child threads
	virtual void run();
	//! The main loop that child threads run in
	virtual void threadLoop();

	//! The current state of the planner (ready, running, paused, etc.)
	PlannerState mState;
	//! Sets the current state of the planner
	void setState(PlannerState s);

	//! A list that is normally used to keep track of solutions found so far
	std::list<GraspPlanningState*> mBestList;
	//! Helper function that helps maintain a list of UNIQUE solutions.
	bool addToListOfUniqueSolutions(GraspPlanningState *s, std::list<GraspPlanningState*> *list, double distance);
	//! Helper function that returns a measure of how similar two states are.
	virtual double stateDistance(const GraspPlanningState *s1, const GraspPlanningState *s2);

	//! Pure abstract function to be written in all subclasses. This is where most of the real planning is done.
	virtual void mainLoop()=0;

signals:
	//! To be emitted during the search as a measure of progress
	void update();
	//! To be emitted when planner stops
	/*! To be emitted when planner stops, either through an explicit 
		termination user command or by exceeding mMaxSteps or mMaxTime.*/
	void complete();

public:
	//! The constructor is desigend NOT to be called by sub-classes.
	EGPlanner(Hand *h);
	virtual ~EGPlanner();
	//! The type of this planner, for easier run-time check.
	virtual PlannerType getType()=0;
	
	//! Tells the planner if it has all the information needed to start planning
	virtual bool initialized(){return true;} 

	//! Used to restart the search from the beginning. 
	virtual bool resetPlanner();

	//! Start the loops
	virtual void startPlanner();
	//! Pause the loops; planner can be re-started
	virtual void pausePlanner();
	//! Stops the planner FOR GOOD.
	virtual void stopPlanner();

	//! Tells the planner to create and use a clone of the hand passed to the constructor
	void createAndUseClone();

	//! Add (and thus render) or remove (and thus hide) the clone from the world scene graph.
	virtual void showClone(bool s);

	//! Tells the planner to start and run in its own thread.
	void startThread();

	//! Get the current state of the planner (ready, running, etc.). Thread-safe.
	PlannerState getState();

	//! Convenience function; return whether current state is RUNNING or not
	virtual bool isActive(){return getState()==RUNNING;}
	//! Convenience function; return whether current state is READY or not
	virtual bool isReady(){return getState()==READY;}

	//! Tells the planner that its initialization is no longer valid and new input must be provided.
	void invalidateReset(){if (isReady()) setState(INIT);}

	//! Tells the planner what kind of energy calculator to use
	virtual void setEnergyType(SearchEnergyType s);
	//! Tells the planner what kind of virtual contact computation to use
	void setContactType(SearchContactType c);

	void setRenderType(RenderType r){mRenderType = r;}
	void setMaxSteps(int s){mMaxSteps=s;}
	void setRepeat(bool r){mRepeat = r;}
	void setMaxTime(int t){mMaxTime=t;}
	
	//! Returns the i-th state from the list of solutions mBestList
	virtual const GraspPlanningState* getGrasp(int i);
	//! Renders the i-th state from the list of solutions mBestList
	virtual void showGrasp(int i);
	//! Returns the size of the mBestList list of solutions
	virtual int getListSize(){return mBestList.size();}
	//! Clears the list of solutions mBestList.
	virtual void clearSolutions();

	int getCurrentStep(){return mCurrentStep;}
	Hand *getHand(){return mHand;}

	//! The time elapsed since the last reset
	double getRunningTime();

	//! Can be used to set what kind of information is used from the "target state"
	bool setInput(unsigned char input, bool on = true);
	//! Sets the "target state" that is used as a model during the search
	GraspPlanningState *getTargetState(){return mTargetState;}
	unsigned char getInputType(){return mInputType;}

	//! Set the stream for outputting stats and info
	void setStatStream(std::ostream *out) const;
};
#endif
