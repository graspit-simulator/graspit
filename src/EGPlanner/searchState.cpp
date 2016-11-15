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
// $Id: searchState.cpp,v 1.40 2009/11/20 23:03:34 cmatei Exp $
//
//######################################################################

#include "searchState.h"
#include "searchStateImpl.h"

#include <QString>
//for the visual marker
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoGroup.h>
#include "SoArrow.h"

#include "robot.h"
#include "body.h"
#include "world.h"
#include "eigenGrasp.h"

//#define GRASPITDBG
#include "debug.h"

SearchVariable::SearchVariable(QString name, double min, double max, double maxJump, bool circular)
{
	mName = name;
	mMinVal = min; mMaxVal = max;
	mMaxJump = maxJump;
	mValue = 0;
	mFixed = false;
	mCircular = circular;
}

SearchVariable::SearchVariable(const SearchVariable *v)
{
	mName = v->mName;
	mMinVal = v->mMinVal; mMaxVal = v->mMaxVal;
	mMaxJump = v->mMaxJump;
	mValue = v->mValue;
	mFixed = v->mFixed;
	mCircular = v->mCircular;
}

//----------------------------------------------------------------------------------

VariableSet::~VariableSet()
{
	clearVariables();
}

VariableSet::VariableSet(const VariableSet &vs)
{
	for (int var = 0; var<vs.getNumVariables(); var++) {
		mVariables.push_back(new SearchVariable(vs.getVariable(var)));
	}
	for (int par=0; par<(int)vs.mParameters.size(); par++) {
		mParameters.push_back(SearchParameter(vs.mParameters[par]));
	}
	mHand = vs.mHand;
}

void
VariableSet::clearVariables()
{
	for (int i=0; i < (int)mVariables.size(); i++)
		delete mVariables[i];
	mVariables.clear();
}

void
VariableSet::changeHand(const Hand *h, bool sticky)
{
	if (sticky) {
		assert(mHand->getNumDOF() == h->getNumDOF());
		assert(mHand->getEigenGrasps()->getSize() == h->getEigenGrasps()->getSize());
	}
	mHand = h;
	if (!sticky) {
		clearVariables();
		createVariables();
	}
}

void 
VariableSet::setAllConfidences(double c)
{
	for (int i=0; i<(int)mVariables.size(); i++) {
		mVariables[i]->setConfidence(c);
	}
}

void 
VariableSet::setAllFixed(bool f)
{
	for (int i=0; i<(int)mVariables.size(); i++) {
		mVariables[i]->setFixed(f);
	}
}

int
VariableSet::getNumUsedVariables() const
{
	int n=0;
	for (unsigned int i=0; i<mVariables.size(); i++)
		if (!mVariables[i]->isFixed()) n++;
	return n;
}

SearchVariable*
VariableSet::getVariable(QString name)
{
	for (unsigned int i=0; i < mVariables.size(); i++) {
		if (mVariables[i]->getName() == name) return mVariables[i];
	}
	DBGP("Requested variable " << name.latin1() << " not found in search state");
	return NULL;
}

const SearchVariable*
VariableSet::getConstVariable(QString name) const
{
	for (unsigned int i=0; i < mVariables.size(); i++) {
		if (mVariables[i]->getName() == name) return mVariables[i];
	}
	DBGP("Requested variable " << name.latin1() << " not found in search state");
	return NULL;
}

double
VariableSet::readVariable(QString name) const
{
	for (unsigned int i=0; i < mVariables.size(); i++) {
		if (mVariables[i]->getName() == name) return mVariables[i]->getValue();
	}
	DBGP("Requested variable " << name.latin1() << " not found in search state");
	return 0;
}

void 
VariableSet::setParameter(QString name, double value)
{
	std::vector<SearchParameter>::iterator it;
	for(it = mParameters.begin(); it!=mParameters.end(); it++) {
		if ( it->name() == name ) break;
	}
	if (it==mParameters.end()) {
		DBGA("Parameter " << name.latin1() << " not found!");
		assert(0);
		return;
	}
	it->set(value);
}

double 
VariableSet::getParameter (QString name) const
{
	std::vector<SearchParameter>::const_iterator it;
	for(it = mParameters.begin(); it!=mParameters.end(); it++) {
		if ( it->name() == name ) break;
	}
	if (it==mParameters.end()) {
		DBGA("Parameter " << name.latin1() << " not found!");
		assert(0);
		return 0;
	}
	return it->get();
}

void
VariableSet::addParameter(QString name, double value)
{
	std::vector<SearchParameter>::iterator it;
	for(it = mParameters.begin(); it!=mParameters.end(); it++) {
		if ( it->name() == name ) break;
	}
	if (it!=mParameters.end()) {
		DBGA("Parameter " << name.latin1() << " already present!");
		assert(0);
		return;
	}
	mParameters.push_back(SearchParameter(name, value));
}

void
VariableSet::removeParameter(QString name)
{
	std::vector<SearchParameter>::iterator it;
	for(it = mParameters.begin(); it!=mParameters.end(); it++) {
		if ( it->name() == name ) break;
	}
	if (it==mParameters.end()) {
		DBGA("Parameter " << name.latin1() << " does not exist!");
		assert(0);
		return;
	}
	mParameters.erase(it);
}

void 
VariableSet::writeToFile(FILE *fp) const
{
	fprintf(fp,"%d ",getType());
	for (int i=0; i<getNumVariables(); i++)
		fprintf(fp,"%f ",mVariables[i]->getValue() );
	fprintf(fp,"\n");
}

bool 
VariableSet::readFromFile(FILE *fp)
{
	//read the type first and check against my own
	int type;
	if( fscanf(fp,"%d",&type) <= 0) {
	  DBGA("VariableSet::readFromFile - failed to get variable set type");
	  return false;
	}
	if (type != getType()) {
		fprintf(stderr,"Wrong type %d in state file (%d expected)\n",type,getType());
		return false;
	}

	char line[10000];
	int offset;
	float v;
	if (!fgets(line, 10000, fp)) {
		fprintf(stderr,"Failed to read data from file!\n");
		return false;
	}
	// should really learn how to use streams instead of these hacks...
	offset = 0;
	//read all variables
	for (int i=0; i<getNumVariables(); i++) {
		if ( line[offset]=='\0' ) {
			fprintf(stderr,"Line to short to read all state variables\n");
			return false;
		}
		while( isspace(*(line+offset)) ) offset++;
		sscanf(line+offset,"%f",&v);
		mVariables[i]->setValue(v);
		while( !isspace(*(line+offset)) ) offset++;
	}
	return true;
}

bool 
VariableSet::readFromArray(std::vector<double> array)
{
	if(getNumVariables() != (int) array.size())
	{
		DBGA("size does not match" << getNumVariables() << " " << array.size());
		return false;
	}
	for(int i = 0; i < getNumVariables(); i ++)
	{
		mVariables[i]->setValue(array[i]);
	}
	return true;
}


/*!	Right now we define the distance as the max distance between any two 
	variables. Alternatively, the mean distance between all variables could 
	also be used. For position states we could also look at the distance in 
	actual position.
*/
double
VariableSet::distance(const VariableSet *s) const
{
	if (getNumVariables() != s->getNumVariables()) return -1;
	double distance = 0;
	for (int i=0; i<getNumVariables(); i++) {
		double altD;
		double d = readVariable(i) - s->readVariable(i);	

		if ( getVariable(i)->isCircular() ) {
			//if the variable is circular, the shortest distance might be around the end
			//going one direction or the other
			altD = fabs( readVariable(i) - getVariable(i)->mMinVal ) + 
				fabs( s->readVariable(i) - getVariable(i)->mMaxVal );
			d = std::min(d, altD);

			altD = fabs( s->readVariable(i) - getVariable(i)->mMinVal ) + 
				fabs( readVariable(i) - getVariable(i)->mMaxVal );
			d = std::min(d, altD);
		}
		d = fabs(d) / getVariable(i)->getRange();
		distance = std::max(d, distance);
	}
	return distance;
}

void
VariableSet::print() const
{
	fprintf(stderr,"\n");
	fprintf(stderr,"Type: %d\n",getType());
	for (int i=0; i<getNumVariables(); i++) {
		fprintf(stderr,"%s = %.2f; ",mVariables[i]->getName().latin1(),mVariables[i]->getValue());
	}
	fprintf(stderr,"\n");
}
//------------------------------- POSTURE STATES -----------------------------------
PostureState* PostureState::createInstance(StateType type, const Hand *h)
{
	switch(type) {
		case POSE_EIGEN:
			return new PostureStateEigen(h);
		case POSE_DOF:
			return new PostureStateDOF(h);
		default:
			assert(0);
			return NULL;
	}
}

PositionState* PositionState::createInstance(StateType type, const Hand *h)
{
	switch(type) {
		case SPACE_COMPLETE:
			return new PositionStateComplete(h);
		case SPACE_ELLIPSOID:
			return new PositionStateEllipsoid(h);
		case SPACE_APPROACH:
			return new PositionStateApproach(h);
		case SPACE_AXIS_ANGLE:
			return new PositionStateAA(h);
		default:
			assert(0);
			return NULL;
	}
}

//----------------------------------------------------------------------------------
HandObjectState::HandObjectState(Hand *h)
{
	init();
	mPosture = PostureState::createInstance(POSE_EIGEN,h);
	mPosition = PositionState::createInstance(SPACE_COMPLETE,h);
	mAttributes = new AttributeSet(h);
	mHand = h;
}

void
HandObjectState::init()
{
	mRefTran = transf::IDENTITY;
	mTargetObject = NULL;
	mPosture = NULL;
	mPosition = NULL;
	mAttributes = NULL;
	IVRoot = NULL;
	IVMat = NULL;
	IVTran = NULL;
}

HandObjectState::~HandObjectState()
{
	hideVisualMarker();
	if (IVRoot) {
		//this should delete it
		IVRoot->unref();
	}
	delete mPosture;
	delete mPosition;
	delete mAttributes;
}

void HandObjectState::changeHand(Hand *h, bool sticky)
{
	mHand = h;
	mPosition->changeHand(h, sticky);
	mPosture->changeHand(h, sticky);
}

void HandObjectState::saveCurrentHandState()
{
	transf newTran = mHand->getTran() * mRefTran.inverse();
	mPosition->setTran(newTran);

	double *dof = new double[mHand->getNumDOF()];
	//we use the dof vals for saving, not the current dof vals
	mHand->storeDOFVals(dof);

	mPosture->storeHandDOF(dof);
	delete [] dof;
}

/*! If the \a sticky flag is true, it will attempt to maintatin the same 
	state in the new parameterization. Otherwise the new parameterization 
	will just be reset. The \s sticky flag does not work for any conversion
	between two parameterization types.
*/
void HandObjectState::setPositionType(StateType type, bool sticky)
{
	if (type == mPosition->getType()) return;
	PositionState *p = PositionState::createInstance(type, mHand);
	transf t;
	if (sticky) {
		t = mPosition->getCoreTran();
	}
	delete mPosition;
	mPosition = p;
	if (sticky) {
		mPosition->setTran(t);
	} else {
		mPosition->reset();
	}
}

/*! If the \a sticky flag is true, it will attempt to maintatin the same 
	state in the new parameterization. Otherwise the new parameterization 
	will just be reset. The \s sticky flag does not work for any conversion
	between two parameterization types.
*/
void HandObjectState::setPostureType(StateType type, bool sticky)
{
	if (type == mPosture->getType()) return;
	PostureState *p = PostureState::createInstance(type, mHand);
	double *dof;
	if (sticky) {
		dof = new double [mHand->getNumDOF()];
		mPosture->getHandDOF(dof);
	}
	delete mPosture;
	mPosture = p;
	if (sticky) {
		mPosture->storeHandDOF(dof);
		delete [] dof;
	} else {
		mPosture->reset();
	}
}

/*! If the \a sticky flag is true, it will attempt to change the values 
	of the new PositionState to maintain the same overall transform that
	we had with the old parameterization.
*/
void HandObjectState::setRefTran(transf t, bool sticky)
{
	transf totalTran;
	if (sticky) {
		totalTran = mPosition->getCoreTran() * mRefTran;
	}
	mRefTran = t;
	if (sticky) {
		mPosition->setTran( totalTran * mRefTran.inverse() );
	}
}

bool HandObjectState::readFromFile(FILE *fp)
{
	//this whole read-write system is one big hack.
	int type; fpos_t pos;
	fgetpos(fp,&pos);
	if (!fscanf(fp,"%d",&type)) return false;
	DBGP("Pose type: " << type);
	if ( (StateType)type != POSE_DOF && (StateType)type != POSE_EIGEN ) return false;
	if ( type != mPosture->getType() ) {
		setPostureType((StateType)type);
	}
	fsetpos(fp,&pos);
	if ( !mPosture->readFromFile(fp) ) {
		DBGA("Failed");
		return false;
	}
	fgetpos(fp,&pos);
	if (!fscanf(fp,"%d",&type)) return false;
	DBGP("Space type: " << type);
	if ( (StateType)type != SPACE_COMPLETE && (StateType)type != SPACE_APPROACH &&
		 (StateType)type != SPACE_AXIS_ANGLE && (StateType)type != SPACE_ELLIPSOID ) return false;
	if ( type != mPosition->getType() ) {
		setPositionType((StateType)type);
	}
	fsetpos(fp,&pos);
	if ( !mPosition->readFromFile(fp) ) return false;
	return true;
}

void HandObjectState::writeToFile(FILE *fp) const
{
	mPosture->writeToFile(fp); 
	mPosition->writeToFile(fp);
}

bool HandObjectState::execute(Hand *h) const
{
	if (!h) h = mHand;
	else assert( h->getNumDOF() == mHand->getNumDOF());

	if (h->setTran( mPosition->getCoreTran() * mRefTran ) == FAILURE) return false;
	double *dof = new double[ h->getNumDOF() ];
	mPosture->getHandDOF(dof);
	//DBGP("Dof: " << dof[0] << " " << dof[1] << " " << dof[2] << " " << dof[3]);
	h->forceDOFVals( dof );
	delete [] dof;
	return true;
}

SearchVariable* HandObjectState::getVariable(int i)
{
	if ( i < 0 ) assert(0);
	else if ( i < mPosture->getNumVariables() ) return mPosture->getVariable(i);
	else if ( i < getNumVariables() ) return mPosition->getVariable( i - mPosture->getNumVariables() );
	else assert(0);
	return NULL;
}

const SearchVariable* HandObjectState::getVariable(int i) const
{
	if ( i < mPosture->getNumVariables() ) return mPosture->getVariable(i);
	else if ( i < getNumVariables() ) return mPosition->getVariable( i - mPosture->getNumVariables() );
	else assert(0);
	return NULL;
}

SearchVariable* HandObjectState::getVariable (QString name)
{
	SearchVariable *v = mPosture->getVariable(name);
	if (v) return v;
	return mPosition->getVariable(name);
}

const SearchVariable* HandObjectState::getConstVariable (QString name) const
{
	const SearchVariable *v = mPosture->getConstVariable(name);
	if (v) return v;
	return mPosition->getConstVariable(name);
}

double
HandObjectState::distance(const HandObjectState *s) const
{
	/*
	bool usePosture = false;
	double d1 = mPosition->distance( s->mPosition );
	double d2 = mPosture->distance( s->mPosture );
	
	if (d1 < 0 || d2 < 0) {
		fprintf(stderr,"Cannot compute distance between disimillar states\n");
		return -1;
	}
	
	if (usePosture) {
		return std::max(d1,d2);
	} 	
	return d1;
	*/
	//alternative version that only looks at euclidian distance

	transf t1 = mHand->getApproachTran() * getTotalTran();
	transf t2 = mHand->getApproachTran() * s->getTotalTran();

	vec3 dvec = t1.translation() - t2.translation();
	double d = dvec.len();

	if (mTargetObject->isA("GraspableBody")) {
		d = d / ((GraspableBody*)mTargetObject)->getMaxRadius();
	} else {
		d = d / 200;
	}

	Quaternion qvec = t1.rotation() * t2.rotation().inverse();
	vec3 axis; double angle;
	qvec.ToAngleAxis(angle,axis);
	double q = 0.5 * fabs(angle) / M_PI; //0.5 weight out of thin air

	return std::max(d,q);
}

SoSeparator*
HandObjectState::getIVRoot()
{
	if (!IVRoot) {
		IVRoot = new SoSeparator;
		IVTran = new SoTransform;
		IVMat = new SoMaterial;
		IVRoot->addChild(IVTran);
		IVRoot->addChild(IVMat);

		IVRoot->ref();
/*
		//flock geometry
		SoCube *cube = new SoCube();
		cube->width = 30;
		cube->height = 20;
		cube->depth = 4;
		IVRoot->addChild(cube);

		SoCube *smallCube = new SoCube();
		smallCube->width = 10;
		smallCube->height = 20;
		smallCube->depth = 6;
		SoTransform *t2 = new SoTransform();
		t2->translation.setValue(10,0,-5);
		IVRoot->addChild(t2);
		IVRoot->addChild(smallCube);
*/
		//aproach geometry
/*
		SoSphere *sphere1 = new SoSphere();
		sphere1->radius = 3;
		IVRoot->addChild(sphere1);

		SoTransform *t2 = new SoTransform();
		t2->translation.setValue( 0,0,3);
		IVRoot->addChild(t2);
		SoSphere *sphere2 = new SoSphere();
		sphere2->radius = 2;
		IVRoot->addChild(sphere2);

		SoTransform *t3 = new SoTransform();
		t3->translation.setValue( 0,0,2);
		IVRoot->addChild(t3);
		SoSphere *sphere3 = new SoSphere();
		sphere3->radius = 1;
		IVRoot->addChild(sphere3);
*/

		//arrow
		SoArrow *arrow = new SoArrow;
		arrow->height = 18;
		arrow->cylRadius = 1.5;
		arrow->coneRadius = 3.25;
		arrow->coneHeight = 8;
		SoTransform *arrowTran = new SoTransform();
		arrowTran->rotation.setValue(SbVec3f(1,0,0),(float)(M_PI/2.0));
		IVRoot->addChild(arrowTran);
		SoTransform *moveTran = new SoTransform();
		moveTran->translation.setValue(SbVec3f(0, -18, 0));
		IVRoot->addChild(moveTran);
		IVRoot->addChild(arrow);

	}

	//flock transform
	//transf t = mHand->getFlockTran()->getMount() * getTotalTran();
	//approach transform
	transf t = mHand->getApproachTran() * getTotalTran();
	t.toSoTransform(IVTran);
	return IVRoot;
}

void
HandObjectState::setIVMarkerColor(double r, double g, double b) const
{
	if (!IVRoot || !IVMat) {
		DBGA("Attempting to set marker color, but marker not created");
		return;
	}
	IVMat->diffuseColor = SbColor(r,g,b);
	IVMat->ambientColor = SbColor(r,g,b);
}

void
HandObjectState::showVisualMarker()
{
	//check if it'a already there so we don't add it again
	if (IVRoot) {
		int i = mHand->getWorld()->getIVRoot()->findChild( IVRoot );
		if (i>=0) {
			DBGP("Search state marker is already in world root");
			return;
		}
	}
	mHand->getWorld()->getIVRoot()->addChild( getIVRoot() );	
}

void
HandObjectState::hideVisualMarker()
{
	if (!IVRoot) return;
	int i = mHand->getWorld()->getIVRoot()->findChild( IVRoot );
	if ( i < 0 ) {
		DBGP("Could not find search state marker in the world root");
	} else {
		mHand->getWorld()->getIVRoot()->removeChild(i);
	}
}

bool
GraspPlanningState::compareStates(const GraspPlanningState *s1, const GraspPlanningState *s2)
{
	if (s1->getEnergy() <= s2->getEnergy()) return true; 
	return false;

}

bool
GraspPlanningState::compareStatesDistances(const GraspPlanningState *s1, const GraspPlanningState *s2)
{
	if (s1->getDistance() <= s2->getDistance()) return true; 
	return false;
}


