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
// $Id: searchEnergy.h,v 1.27 2009/09/13 19:56:51 hao Exp $
//
//######################################################################

#ifndef _searchenergy_h_
#define _searchenergy_h_

#include <ostream>
#include <list>
#include <QObject>

#include "search.h"
#include "matvec3D.h"

class Hand;
class Body;
class Robot;
class QualityMeasure;
class GraspPlanningState;

//! Computes the "quality" of a HandObjectState, which encapsulates the state of the hand 
/*!	This class can compute the "quality" (or in other terms "energy") of a
	HandObjectState. There are many ways of doing this which are all right now 
	hidden in the protected functions and managed by the mType flag. In the 
	near future I plan to re-write this using inheritance instead of 
	switch(...) but I never got around to doing that.

	The various type of energy calculations used are a bit fuzzy too, 
	re-organization will help when it happens. For now, I will not provide 
	very detailed documentation here, as I am really embarassed by this code
	and plan to re-write it as soon as possible.
*/
class SearchEnergy : public QObject
{
	Q_OBJECT
protected:
	static int counter;
	Hand *mHand; Body *mObject;
	QualityMeasure *mVolQual, *mEpsQual;
	SearchEnergyType mType;
	SearchContactType mContactType;
	/*! If this flag is set, the hand is disconnected from the scene graph while 
		the calculator does energy computations */
	bool mDisableRendering;

	//! If not null, it will print its output here
	mutable std::ostream *mOut;

	void createQualityMeasures();
	void setHandAndObject(Hand *h, Body *o);
	double contactEnergy() const;
	double potentialQualityEnergy(bool verbose = false) const;
	double guidedPotentialQualityEnergy() const;
	double autograspQualityEnergy() const;
	double approachAutograspQualityEnergy() const;  //Accepts only contacts on palm, otherwise, retreats to initial position.
	double approachToContactAutograspQualityEnergy() const ;// This accepts any contact.
	double guidedAutograspEnergy() const;
	double strictAutograspEnergy() const;
	double numContactsEnergy() const;

	//! Closes the hand in dynamics mode, then computes grasp quality
	double dynamicAutograspEnergy() const;
	//! Helper function for dynamics energy; returns true if any contacts are slipping
	bool contactSlip() const;
	//! Another helper function for dynamics energy
	bool dynamicAutograspComplete() const;
	//! Used for project with Harvard hand
	double compliantEnergy() const;

	double distanceFunction(double d) const;
	double potentialQualityScalingFunction(double dist, double cosTheta) const;

	//! This is where the decision is made of which type of energy value should be computed and returned
	double energy() const;
	//! Checks if the current state is legal or not (usually legal means no interpenetrations)
	bool legal() const;

	//! Used only by compliant energy to keep track of its internal state
	mutable bool mCompUnbalanced;
	mutable vec3 mMaxUnbalancedForce;
	//! Used by dynamic energy to keep track of the dynamic autograsp
	mutable bool mDynamicsError;
private slots:
	//! Called to compute compliant force balances during autograsp
	void autoGraspStep(int numCols, bool &stopRequest) const;
	//!Called to warn of a dynamics error in dynamic quality
	void dynamicsError(const char*) const;
public:
	SearchEnergy();
	~SearchEnergy();

	void setType(SearchEnergyType t){mType = t;}
	void setContactType(SearchContactType t){mContactType = t;}
	double getEpsQual();
	double getVolQual();
	SearchContactType getContactType() const {return mContactType;}
	void disableRendering(bool dr){mDisableRendering = dr;}

	/*! This is the main interface to this class. It is passed a GraspPlanningState* and returns whether the HandObjectState is 
		legal, and if so, it's energy. If noChange = true, it will re-set the world situation to what it was on entry.
		if not, it will leave the world in the state encapsulated in HandObjectState.
		if the HandObjectState is not legal, it will re-set the world before exiting regardless of the noChange flag.
	*/
	virtual void analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange = true);
	//! Works the same way as analyzeState, but analyzes the hand as it is when the function is called.
	virtual void analyzeCurrentPosture(Hand *h, Body *o, bool &isLegal, double &stateEnergy, bool noChange = true);
	//! Check the whether the parent robot can or not achieve the child robot's position
	virtual bool analyzeAccessibility(Robot *parentRobot, GraspPlanningState *childState);

	//! Sets the stat file where results are to be written
	void setStatStream(std::ostream *out) const {mOut = out;}
};

/*! This class is meant to be used with the GuidedPlanner that looks for force-closure. It adds one main thing:
	- can use a list of "avoid states", so that a new state is deemed illegal if it is in the vicintiy of one of these	
*/
class ClosureSearchEnergy : public SearchEnergy
{
protected:
	const std::list<GraspPlanningState*> *mAvoidList;
	double mThreshold;
public:
	ClosureSearchEnergy() : SearchEnergy(), mAvoidList(NULL), mThreshold(0.3) {}

	void setThreshold(double t){mThreshold=t;}
	void setAvoidList(const std::list<GraspPlanningState*> *l){mAvoidList = l;}
	void analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange = true);
};

#endif
