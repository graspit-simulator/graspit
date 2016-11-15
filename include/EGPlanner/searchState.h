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
// $Id: searchState.h,v 1.36 2009/07/24 13:40:47 cmatei Exp $
//
//######################################################################

#ifndef _HandObjectState_h_
#define _HandObjectState_h_

#include <vector>
#include <list>
#include <QString>

#include "search.h"
#include "matvec3D.h"

class Hand;
class Body;
class GraspableBody;
class QualityMeasure;

class SoSeparator;
class SoTransform;
class SoMaterial;

/*!	This is a general variable that we can perform search over. It can be an 
	eigenGrasp amplitude, wrist DOF, hand position in space etc. It just has
	a value, a legal range defined my a min and a max. It can also store some 
	other information, such as the maximum "jump" allowed for this variable
	when doing simulated annealing, a "confidence" leve we have in the value
	of this variable, etc.
*/
class SearchVariable
{
private:
	double mValue;
	//! The confidence that we have in the stored value
	/*! Mainly used when this variable is used as "input" to guide a search,
		rather than a variable that is being searched. */
	double mConfidence;
	//! The name of this variable; can be used to retrieve the variable
	QString mName;
	//! Flag shows that this variable is fixed at its current position and should not be changed
	bool mFixed;
	//! If the variable is circular it means that max and min actually mean the same thing
	/*! For example, angles: -PI = PI */
	bool mCircular;

public:
	double mMinVal, mMaxVal, mMaxJump;

	SearchVariable(QString name, double min, double max, double maxJump, bool circular = false);
	SearchVariable(const SearchVariable *v);

	void setValue(double v){mValue = v;}
	double getValue() const {return mValue;}
	void setFixed(bool f){mFixed = f;}
	bool isFixed() const {return mFixed;}
	void setCircular(bool c){mCircular = c;}
	bool isCircular() const {return mCircular;}
	void setRange(double min, double max){mMinVal = min; mMaxVal = max;}
	double getRange() const { return fabs(mMaxVal - mMinVal); }
	void setMaxJump(double mj){mMaxJump = mj;}
	void setJump(double j){mMaxJump = getRange() * j;}
	double getConfidence() const {return mConfidence;}
	void setConfidence(double c) {assert(c>=0 && c<=1);mConfidence=c;}
	QString getName() const {return mName;}
};

/*! A parameter is used to change the behavior of a search, but is not actually
	searched on. Is set once, typically at the beginning of the search. 
	Conceptually, it is not all that different from a variable. For example,
	consider we are searching for a position on a known sphere. The latitude and
	longitude would be variables, but the radius of the sphere would be a 
	parameter.
*/
class SearchParameter {
private:
	QString mName;
	double mValue;
public:
	SearchParameter(QString name, double val=0) : mName(name), mValue(val) {}
	SearchParameter(const SearchParameter &p) {
		mName = p.mName;
		mValue = p.mValue;
	}
	void set(double v){mValue = v;}
	double get() const {return mValue;}
	QString name() const {return mName;}
};

/*!	A VariableSet is a collection of multiple variables and parameters. It manages
	the list of variables, can read and write itself to a file, etc.
*/
class VariableSet
{
protected:
	//! The variables in the set
	std::vector<SearchVariable*> mVariables;
	//! The parameters in the set
	std::vector<SearchParameter> mParameters;
	//! The hand pointer shouldn't really be here
	/*! At this level nobody cares about that, but it was easier like this. */
	const Hand *mHand;

	void clearVariables();

	//! Implementations of VariableSet that know what purpose they serve will implement this
	virtual void createVariables() = 0;
public:
	VariableSet(const Hand *h){mHand = h;}
	VariableSet(const VariableSet &vs);
	virtual ~VariableSet();
	virtual StateType getType() const = 0;

	inline void copyValuesFrom(const VariableSet *s);
	inline void reset();
	virtual void changeHand(const Hand *h, bool sticky = false);

	int getNumVariables() const {return mVariables.size();}
	int getNumUsedVariables() const;
	SearchVariable* getVariable(int i){assert(i<(int)mVariables.size()); return mVariables[i];}
	const SearchVariable* getVariable(int i) const {assert(i<(int)mVariables.size()); return mVariables[i];}
	SearchVariable* getVariable (QString name);
	const SearchVariable* getConstVariable (QString name) const;
	double readVariable(int i) const {return mVariables[i]->getValue();}
	double readVariable(QString name) const;
	virtual double distance(const VariableSet *s) const;

	void setParameter(QString name, double value);
	double getParameter (QString name) const;

	void addParameter(QString name, double value);
	void removeParameter(QString name);

	void setAllConfidences(double c);
	void setAllFixed(bool f);

	void writeToFile(FILE *fp) const;
	bool readFromFile(FILE *fp);
	bool readFromArray(std::vector<double> array);
	void print() const;
};

/*! Only stores parameters, no variables. */
class AttributeSet : public VariableSet
{
private:
	//! No variables are stored here.
	virtual void createVariables(){}
public:
	AttributeSet(const Hand *h) : VariableSet(h){}
	AttributeSet(const AttributeSet &as) : VariableSet(as){}
	virtual StateType getType() const {return ATTRIBUTES;}
};

/*!	This class is a set of variables for saving hand posture. In knows how to 
	convert the variables in the set into DOF values for a given hand. It is 
	still pure abstract, as there can be multiple ways of storing the DOF's 
	of the hand - see searchStateImpl.h for implementations.
*/
class PostureState : public VariableSet
{
public:
	PostureState(const Hand *h) : VariableSet(h) {}
	~PostureState(){}
	//! Get the DOF values that are set by the current values of the variables stored here
	virtual void getHandDOF(double *dof) const = 0;
	//! Set the values of the current variables to match the given set of DOF values
	/*! Not all current implementations have this implemented correctly.*/
	virtual void storeHandDOF(const double *dof) = 0;
	static PostureState* createInstance(StateType type, const Hand *h);
};

/*!	This class is a set of variables for saving wrist position and orientation. 
	It knows how to convert its own	variables into a transform that gives the 
	wrist position. Again, there can be many ways of saving a position and
	orientation, so this class is also pure abstract. See searchStateImpl.h 
	for implementations.
*/
class PositionState : public VariableSet
{
public:
	PositionState(const Hand *h) : VariableSet(h){}
	~PositionState(){}
	//! Get the transform set by the current values of the variables stored here
	virtual transf getCoreTran() const = 0;
	//! Set the internal variables to match the given transform
	/*! Not all implementations have this implemented correctly.*/
	virtual void setTran(const transf &t) = 0;
	static PositionState* createInstance(StateType type, const Hand *h);
};

/*!	A HandObjectState is the combination of a PositionState and a PostureState. 
	As such, it completely defines the hand	and can be used to encapsulate the 
	state during a search for a good grasp (grasp planning). It can also have an 
	energy level (or quality function) associated (although it does not know how 
	to compute it itself).

	In general, can be thought of as a "State" as it encapsulates part of the state
	the world - the part that is relevant to grasping. It can contain the position
	of the hand relative to the object, the hand posture and other information. As
	such, can also be thought of as a "grasp" but that term already exists in GraspIt
	and refers to the Grasp class which does not contain state, but can compute
	grasp quality metrics.

	Mainly, the HandObjectState contains two sets of variables: an instance of the 
	PositionState class which is used to save the position of the hand, and an 
	instance of the PostureState class used to save the posture of the hand.

	The HandObjectState can be asked to render itself. It knows about what Hand it is 
	used on.

	It can also store a REFERENCE transform. This means that the inner transform, 
	stored in mPosition, is only relevant relative to the reference transform. 
	Imagine for example that we want the mPosition to just encode the trasnform 
	relative to the target object - then we set mRefTran to the transform of the 
	object. Or, mPosition can be relative to an input transform specified by the 
	user, etc. Alternatively, mPosition can also store the hand transform in 
	absolute values (world coordinates) in which case we just set 
	mRefTran = transf::IDENTITY.

	For debugging purposes, it can also create a visual marker that can be added 
	to the world scene graph (just below the world IVRoot) which can show us where 
	this State would want to place the hand.
*/
class HandObjectState
{
private:
	void init();
	//! The variables that define the hand posture
	PostureState *mPosture;
	//! The variables that define the hand position relative to mRefTran
	PositionState *mPosition;
	//! A set of attributes with numerical values that are used to store information, but not to search on
	/*! The attributes stored here are strictly characteristics of the HandObjectState,
		they have no functional importance to saving or restoring the state. For
		example, here we would store the quality of the grasp.
	*/
	AttributeSet* mAttributes;

	//! The hand that this state refers to
	Hand *mHand;

	//! The object that is the target of the grasp search
	GraspableBody *mTargetObject;

	//! This is the reference transform that this state's transform is relative to
	transf mRefTran;

	//! Can be added to the world scene graph root so we can visualize where this solution is
	SoSeparator *IVRoot;
	//! Internal storage for the visual marker
	SoTransform *IVTran;
	//! Internal storage for the visual marker
	SoMaterial *IVMat;
	
public:
	HandObjectState(Hand *h);
	HandObjectState(const HandObjectState *s){init();copyFrom(s);}
	virtual ~HandObjectState();
	inline void copyFrom(const HandObjectState *s);
	//! Resets both posture and position (currently sets all variables to 0)
	inline void reset();

	//! This HandObjectState becomes a snapshot of the CURRENT world state when the function is called.
	void saveCurrentHandState();

	void changeHand(Hand *h, bool sticky = false);
	Hand *getHand() const {return mHand;}
	void setObject(GraspableBody *b){mTargetObject = b;}
	GraspableBody *getObject() const {return mTargetObject;}

	//! Changes the WAY we store the hand position, not necessarily the position itself
	void setPositionType(StateType type, bool sticky = false);
	//! Changes the WAY we store the hand posture, not necessarily the posture itself
	void setPostureType(StateType type, bool sticky = false);
	//! Changes the reference transform.
	void setRefTran(transf t, bool sticky = false);

	//! Places the given hand in the given state; if no hand is given it places its own hand
	bool execute(Hand *h = NULL) const;

	//! Gives the inner transform (stored in mPosition) multiplied by the reference transform
	transf getTotalTran() const {return mPosition->getCoreTran() * mRefTran;}

	PostureState* getPosture(){return mPosture;}
	const PostureState* readPosture() const {return mPosture;}
	PositionState* getPosition(){return mPosition;}
	const PositionState* readPosition() const {return mPosition;}

	void writeToFile(FILE *fp) const;
	bool readFromFile(FILE *fp);

	//! Prints the variables to stderr
	void printState () const {mPosture->print(); mPosition->print();}

	double distance(const HandObjectState *s) const;

	int getNumVariables() const {return mPosture->getNumVariables() + mPosition->getNumVariables();}
	int getNumUsedVariables() const {return mPosture->getNumUsedVariables() + 
										    mPosition->getNumUsedVariables();}
	
	SearchVariable* getVariable(int i);
	const SearchVariable* getVariable(int i) const;
	SearchVariable* getVariable (QString name);
	const SearchVariable* getConstVariable (QString name) const;
	
	void setAttribute(QString name, double value){mAttributes->setParameter(name, value);}
	double getAttribute(QString name) const {return mAttributes->getParameter(name);}
	void addAttribute(QString name, double value) {mAttributes->addParameter(name, value);}
	void removeAttribute(QString name){mAttributes->removeParameter(name);}

	//! Updates and returns the geometry of the debugging marker, with the correct transform already in
	SoSeparator *getIVRoot();
	void setIVMarkerColor(double r, double g, double b) const;

	//! Places a visual marker in the world to show where its position would be
	void showVisualMarker();
	//! Hides the visual marker
	void hideVisualMarker();
};

//! This is a HandObjectState that stores more information useful for grasp planning
/*! In addition to the state, this class can store more information
	needed for grasp planning. This includes whether the state is
	"legal" or not, the grasp quality, energy, etc. However, it has
	no intelligence, in the sense that it does not compute any of these;
	it just stores whatever is set externally.
*/
class GraspPlanningState : public HandObjectState
{
private:
	//! The energy used by the simulated annealing-based planners
	double mEnergy;
	//! Whether this state is legal or not, usually refers to presence or absence of collisions
	bool mLegal;
	//! The iteration number inside the search that this state was found at
	int mItNumber;
	//! For the online planner, this is the distance between the hand and the object
	double mDistance;
	//! The epsilon quality for this grasp
	double mQualEpsilon;
	//! The volume quality for this grasp
	double mQualVolume;
	//! Used for grasp cross-correlation; shows the index of the original grasp that was used for this one
	int mIndex;

	//! The list of contacts between hand and object in object coordinates
	std::list<position> mContacts;

public:
	GraspPlanningState(Hand* hand) : HandObjectState(hand) {}
	GraspPlanningState(const GraspPlanningState *gps) : HandObjectState(gps){
		copyFrom(gps);
	}

	void copyFrom(const GraspPlanningState *t){
		HandObjectState::copyFrom(t); 

		mEnergy=t->getEnergy();
		mLegal=t->isLegal();
		mDistance = t->getDistance();
		mItNumber = t->getItNumber();

		mQualEpsilon = t->getEpsilonQuality(); 
		mQualVolume = t->getVolume(); 
		mIndex = t->getIndex();
	}

	double getEnergy() const {return mEnergy;} void setEnergy(double e){mEnergy = e;}
	bool isLegal() const {return mLegal;} void setLegal(bool l){mLegal = l;}
	int getItNumber() const {return mItNumber;} void setItNumber(int n){mItNumber=n;}
	double getDistance() const {return mDistance;} void setDistance(double d){mDistance = d;}

	double getEpsilonQuality() const {return mQualEpsilon;} void setEpsilonQuality(double q){mQualEpsilon = q;}
	double getVolume() const {return mQualVolume;} void setVolume(double v){mQualVolume = v;}
	int getIndex() const {return mIndex;} void setIndex(double i){mIndex = i;}
	std::list<position> *getContacts(){return &mContacts;}
	const std::list<position> *getContacts() const {return &mContacts;}

	//! Compares two states by their saved "energy" information
	static bool compareStates(const GraspPlanningState *s1, const GraspPlanningState *s2);
	//! Compares two states by their saved "distance" information
	static bool compareStatesDistances(const GraspPlanningState *s1, const GraspPlanningState *s2);

};

void VariableSet::copyValuesFrom(const VariableSet *s)
{
	for (int i=0; i<(int)s->mVariables.size(); i++) {
		mVariables[i]->setValue( s->mVariables[i]->getValue());
		mVariables[i]->setFixed( s->mVariables[i]->isFixed() );
	}
	for (int i=0; i<(int)s->mParameters.size(); i++) {
		mParameters[i].set( s->mParameters[i].get() );
	}
}

/*! Sets all variables to 0 (regardless of whether 0 is inside the legal
	range of this variable), the fixed flag to false and the confidence 
	to 0.0
*/
void VariableSet::reset()
{
	for (int i=0; i<(int)mVariables.size(); i++) {
		mVariables[i]->setValue(0);
		mVariables[i]->setFixed(false);
		mVariables[i]->setConfidence(0.0);
	}
}

void HandObjectState::copyFrom(const HandObjectState *s)
{
	mHand = s->mHand;
	mTargetObject = s->mTargetObject;
	mRefTran = s->mRefTran;

	if (mPosture) delete mPosture;
	if (mPosition) delete mPosition;
	mPosture = PostureState::createInstance(s->mPosture->getType(), mHand);
	mPosture->copyValuesFrom(s->mPosture);
	mPosition = PositionState::createInstance(s->mPosition->getType(), mHand);
	mPosition->copyValuesFrom(s->mPosition);

	if (mAttributes) delete mAttributes;
	mAttributes = new AttributeSet(*s->mAttributes);
}

void HandObjectState::reset()
{
	mPosture->reset();
	mPosition->reset();
}

/*! Creates a list of search states obtained by sampling uniformly at a given
	resolution the position space variables of the seed state. Variables that are
	set as fixed are not sampled.
*/
void createPositionSpaceSampling(const GraspPlanningState &seed, 
								 std::list<GraspPlanningState*> *sampling, int samples);

#endif
