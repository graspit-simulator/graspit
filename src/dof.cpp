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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: dof.cpp,v 1.24 2009/07/27 17:00:17 cmatei Exp $
//
//######################################################################

#include "dof.h"

#include <QTextStream>

#include "robot.h"
#include "joint.h"
#include "dynJoint.h"
#include "body.h"
#include "world.h"

//#define GRASPITDBG
#include "debug.h"

  /*! Initializes everything to zero. */
DOF::DOF() : owner(NULL), q(0.0),desiredq(0.0), desiredVelocity(0.0),defaultVelocity(0.0),
	actualVelocity(0.0), maxAccel(10.0), force(0.0),maxForce(0.0),
	extForce(0.0), Kv(0.0),Kp(0.0),mHistoryMaxSize(10), mDynStartTime(0.0), currTrajPt(0), 
	draggerScale(20.0)
{
}

/*! This copy constructor only copies the paramters of the DOF, but none of 
	its dynamic state or the joints or robot. Use initDOF(), like you would 
	with a regularly contructed DOF, to assign joints to it.
*/
DOF::DOF(const DOF *original)
{
  owner = NULL;
  dofNum = original->dofNum;
  maxq = original->maxq;
  minq = original->minq;
  q = 0.0;
  desiredq = 0.0;

  defaultVelocity = original->defaultVelocity;
  maxAccel = original->maxAccel;
  desiredVelocity = 0.0;
  actualVelocity = 0.0;

  maxForce = original->maxForce;
  force = 0.0;
  extForce = 0.0;

  Kv = original->Kv;
  Kp = original->Kp;

  mHistoryMaxSize = original->mHistoryMaxSize;
  draggerScale = original->draggerScale;
  currTrajPt = 0;
}

/*!
  Initializes a DOF by giving it the owning robot \a myRobot, and the list
  of joints connected to this DOF, \a jList.  Also sets the max and min vals
  of the DOF from the smallest range of the joint limits.
*/
void
DOF::initDOF(Robot *myRobot,const std::list<Joint *>& jList)
{
  owner = myRobot;  
  jointList = jList; // copy jList
  updateMinMax();
}

/*! Sets the max and min vals of the DOF from the smallest range of the joint 
  limits. */
void DOF::updateMinMax()
{
  maxq = (*jointList.begin())->getMax()/getStaticRatio(*jointList.begin());
  minq = (*jointList.begin())->getMin()/getStaticRatio(*jointList.begin());
  DBGP("Joint 0 min "  << minq << " max " << maxq);
  if (maxq < minq) std::swap(maxq, minq);  
  DBGP("maxq " << maxq << " minq " << minq);
  std::list<Joint *>::iterator j;
  double testMin, testMax;
  DBGST(int num = 0;)
  for(j=++jointList.begin();j!=jointList.end();j++) {
    testMax = (*j)->getMax()/getStaticRatio(*j);
    testMin = (*j)->getMin()/getStaticRatio(*j);
    //can happen if ratio is negative
    if (testMax < testMin) std::swap(testMin, testMax);
    DBGP("Joint " << num++ << " min "  << testMin << " max " << testMax);
    maxq = ( testMax < maxq ? testMax : maxq);
    minq = ( testMin > minq ? testMin : minq);
    DBGP("maxq " << maxq << " minq " << minq);
  }
}

/*!
  If a trajectory exists, this increments the \a currTrajPt counter, and
  makes the \a setPoint the next point of the trajectory.  Otherwise, it
  makes the \a setPoint the current desired value.  The \a setPoint is used
  by the PD controller to determine the proper force to apply to this DOF.
*/
void
DOF::updateSetPoint() {
	if (!trajectory.empty()) {    
		setPoint = trajectory[currTrajPt++];
		if (trajectory.begin()+currTrajPt==trajectory.end()) {
			trajectory.clear();
		}
	}else if (setPoint != desiredq) {
		setPoint=desiredq;
	}
	DBGP("Update set point: " << setPoint);
}

/*!
  Given an array of DOF values, and the length of the array, this clears the
  current trajectory and sets the trajectory to the given values.
*/
void
DOF::setTrajectory(double *traj,int numPts)
{
	trajectory.clear();
	currTrajPt = 0;
	trajectory.reserve(numPts);
	for (int i=0;i<numPts;i++) {
		trajectory.push_back(traj[i]);
	}
	desiredq = traj[numPts-1];
	mErrorHistory.clear();
	mPositionHistory.clear();
	mForceHistory.clear();
	//reset the dynamics start time
	mDynStartTime = owner->getWorld()->getWorldTime();
}

/*!	Adds \a numPts values to the current trajectory.*/
void
DOF::addToTrajectory(double *traj,int numPts)
{
	for (int i=0;i<numPts;i++){
		trajectory.push_back(traj[i]);
	}
	desiredq = trajectory.back();
}

/*! This wraps the controller call so we can have different types of
	DOF's call different controllers if we want to.
*/
void
DOF::callController(double timeStep)
{
	//we need to update the current position based on current joint values 
	//we rely on those having already been set from dynamics
	updateFromJointValues();

	//bookkeeping
	mErrorHistory.push_front(setPoint - q);
	while ((int)mErrorHistory.size() > mHistoryMaxSize) {
		mErrorHistory.pop_back();
	}
	//the error wrt the last desired position in the trajectory
	/*
	if (!trajectory.empty()) {
		mPositionHistory.push_front(trajectory.back() - q);
	} else {
		mPositionHistory.push_front(setPoint - q);
	}
	*/
	if (!mPositionHistory.empty()) {
		mVelocityHistory.push_front( (q - mPositionHistory.front() ) / timeStep );
		while ((int)mVelocityHistory.size() > mHistoryMaxSize) {
			mVelocityHistory.pop_back();
		}
	}

	mPositionHistory.push_front(q);
	while ((int)mPositionHistory.size() > mHistoryMaxSize) {
		mPositionHistory.pop_back();
	}
	
	//call the appropriate controller
	double newForce = PDPositionController(timeStep);
	//actually sets the force
	setForce(newForce);

	mForceHistory.push_front(newForce);
	while ((int)mForceHistory.size() > mHistoryMaxSize) {
		mForceHistory.pop_back();
	}
}

/*!
  Updates the error between the current DOF value and its setpoint and uses
  this along with the rate of change of the error (first-order approx) to
  compute a force that should be applied to the DOF to correct the error.  The
  gains for this controller are read from the robot configuration file.
*/
double
DOF::PDPositionController(double timeStep)
{
	double error = mErrorHistory.front(), lastError;
	if (mErrorHistory.size() >= 2) {
		lastError = *(++mErrorHistory.begin());
	} else {
		lastError = error;
	}

	double newForce;

	newForce = Kp * error + Kv * (error-lastError)/timeStep;

	DBGP("DOF " << getDOFNum() );
	DBGP("setPoint ="<<setPoint<<" error ="<<error<<" edot = "<<(error-lastError));
	DBGP("proportional: "<<error<<"*"<<Kp<<"="<<error*Kp);
	DBGP("derivative: "<<Kv<<"*"<<(error-lastError)/timeStep);
	DBGP("cap: "<<getMaxForce()*0.8 << " Force: "<<newForce);
	DBGP("e " << error << " de " << error-lastError << " ts " << timeStep << " f " << newForce);
	return newForce;
}

/*! Not implemented for the general case. This is a more difficult feature than
	originally thought. What is a good test of whether dynamic progress has
	stopped?
*/
bool
DOF::dynamicsProgress()
{
	return true;
}

/*! Reads the parameters (not the value) of this DOF from XML. Used
	when reading robot configuration files, where we want to define a
	DOF, not set a particular value.
*/
bool
DOF::readParametersFromXml(const TiXmlElement* root)
{
	if(!getDouble(root,"defaultVelocity", defaultVelocity))
		return false;
	if(!getDouble(root,"maxEffort", maxForce))
		return false;
	if(!getDouble(root,"Kp", Kp))
		return false;
	if(!getDouble(root,"Kd", Kv))
		return false;
	if(!getDouble(root,"draggerScale", draggerScale))
		return false;
	return true;
}

/*! Write the value, or the overall state, of this DOF to a stream. Used
	for saving the posture of a robot, either to a variable or to a file.
*/
bool
DOF::writeToStream(QTextStream &stream)
{
	stream << q;
	return true;
}

/*! Read in the value (or for more complex dof's the complete state) of 
	this dof from a stream.
*/
bool
DOF::readFromStream(QTextStream &stream)
{
	if (stream.atEnd()) return false;
	stream >> q;
	return true;
}

/*!
  Sets the current force applied to this DOF, keeping it within the maximum
  force limits.  This force is then applied to the joints based on the 
  transmission type implemented by each dof.
*/
void
RigidDOF::setForce(double f)
{
	Joint *activeJoint = *(jointList.begin());
	if (f > maxForce) force = maxForce;
	else if (f < -maxForce) force = -maxForce;
	else force = f;

	//set force for the first joint only, passive joint constraints will take care of the others
	activeJoint->applyInternalWrench(force);
	DBGP("Applied force = " << force);
}

/*! Returns how far this dof is from exceeding one of it's joint limits.
	The return value is the distance between the current value and the
	joint limit, positive if the joint is inside its limit or negative if
	the joint has already exceeded its limit. \a direction is set to +1 if
	the limit is a positive one, or -1 if the limit is a negative one.
*/
double
RigidDOF::getClosestJointLimit(int *direction)
{
	bool firstTime = true;
	double closestLimit = 0.0;
	*direction = 0.0;
	std::list<Joint*>::iterator j;
	for (j=jointList.begin(); j!=jointList.end(); j++) {
		double val = (*j)->getVal();
		double maxError = val - (*j)->getMax();
		double minError = (*j)->getMin() - val;
		double error; int jdir;
		if (maxError > minError) {
			error = maxError;
			jdir = -1;
		} else {
			error = minError;
			jdir = +1;
		}
		error /= getStaticRatio(*j);
		if (firstTime || fabs(error) > fabs(closestLimit)) {
			closestLimit = error;
			*direction = jdir;
			firstTime = false;
		}
	}
	return closestLimit;
}

/*! If any of the joints are too close (or over) their range, we still
	have a single contraint that we will use to correct this
*/
int
RigidDOF::getNumLimitConstraints() 
{
	int d;
	if (getClosestJointLimit(&d) < -0.01) return 0;
	return 1;
}

/*! The regular DOF only imposes a single limit constraint, which will be placed
	on the first joint in this DOF. Assumes the rest of the joints are rigidly
	connected, so coupling constraints will take care of the rest.
*/
void 
RigidDOF::buildDynamicLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
								  double* H, double *g, int &hcn)
{	
	int dir;
	double error = getClosestJointLimit(&dir);
	if ( error < -0.01) return;

	g[hcn] = MIN(0.0, -error - 0.005);
	DBGP("adding dof limit constraint for DOF " << dofNum << " in dir " << dir << "; error: "<<g[hcn]);

	//the first joint in the list is considered the master joint
	Joint *currentJoint = jointList.front();
	DynamicBody *prevLink = currentJoint->dynJoint->getPrevLink();
	DynamicBody *nextLink = currentJoint->dynJoint->getNextLink();
   
	assert( islandIndices[prevLink]>=0 );
	int row = 6*islandIndices[prevLink];
	H[(hcn)*6*numBodies + row+3] -= dir * currentJoint->getWorldAxis()[0];
	H[(hcn)*6*numBodies + row+4] -= dir * currentJoint->getWorldAxis()[1];
	H[(hcn)*6*numBodies + row+5] -= dir * currentJoint->getWorldAxis()[2];

    assert( islandIndices[nextLink]>=0 );
	row = 6*islandIndices[nextLink];
	H[(hcn)*6*numBodies + row+3] +=  dir * currentJoint->getWorldAxis()[0];
	H[(hcn)*6*numBodies + row+4] +=  dir * currentJoint->getWorldAxis()[1];
	H[(hcn)*6*numBodies + row+5] +=  dir * currentJoint->getWorldAxis()[2];

	hcn++;
}

/*! On the rigid dof, all joints are coupled together, enforcing that the movement
	in all joints is identical to the movement in the first joint in the list
	controlled by this DOF.
*/
void 
RigidDOF::buildDynamicCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
									 double* Nu, double*, int &ncn)
{
	//again we consider the first joint as the master joint 
	Joint *masterJoint = jointList.front();
	
	DynamicBody *masterPrevLink = masterJoint->dynJoint->getPrevLink();
	DynamicBody *masterNextLink = masterJoint->dynJoint->getNextLink();
	double masterRatio = getStaticRatio(masterJoint);
	vec3 masterFreeRotAxis = masterJoint->getWorldAxis();

	std::list<Joint*>::iterator it = jointList.begin();
	while(++it != jointList.end()) {
		Joint *currentJoint = (*it);
		//for now this only works on revolute dynamic joints
		//which means made up of a single revolute static joint
		//(unlike universal dynamic joints which are made up of 2
		//revolute static joints)

		if (currentJoint->getType() != REVOLUTE) continue;
		
		DynamicBody *prevLink = currentJoint->dynJoint->getPrevLink();
		DynamicBody *nextLink = currentJoint->dynJoint->getNextLink();
		vec3 freeRotAxis = currentJoint->getWorldAxis();

		//the *rotation* between these joint links...
		double ratio = 1.0/getStaticRatio(currentJoint);
		assert( islandIndices[prevLink] >= 0 );
		int row = 6*islandIndices[prevLink];
		Nu[(ncn)*6*numBodies + row+3] -= ratio * freeRotAxis[0];
		Nu[(ncn)*6*numBodies + row+4] -= ratio * freeRotAxis[1];
		Nu[(ncn)*6*numBodies + row+5] -= ratio * freeRotAxis[2];
		assert( islandIndices[nextLink] >= 0 );
		row = 6*islandIndices[nextLink];
		Nu[(ncn)*6*numBodies + row+3] +=  ratio * freeRotAxis[0];
		Nu[(ncn)*6*numBodies + row+4] +=  ratio * freeRotAxis[1];
		Nu[(ncn)*6*numBodies + row+5] +=  ratio * freeRotAxis[2];

		//... must be the same as the movement between the master joint links
		ratio = 1.0/masterRatio;
		assert( islandIndices[masterPrevLink] >= 0 );
		row = 6*islandIndices[masterPrevLink];
		Nu[(ncn)*6*numBodies + row+3] += ratio * masterFreeRotAxis[0];
		Nu[(ncn)*6*numBodies + row+4] += ratio * masterFreeRotAxis[1];
		Nu[(ncn)*6*numBodies + row+5] += ratio * masterFreeRotAxis[2];
		assert( islandIndices[masterNextLink] >= 0 );
		row = 6*islandIndices[masterNextLink];
		Nu[(ncn)*6*numBodies + row+3] += - ratio * masterFreeRotAxis[0];
		Nu[(ncn)*6*numBodies + row+4] += - ratio * masterFreeRotAxis[1];
		Nu[(ncn)*6*numBodies + row+5] += - ratio * masterFreeRotAxis[2];

		ncn++;		

		//there should be a corrective term added here!!!
	}
}

double 
RigidDOF::getStaticRatio(Joint *j) const 
{
	return j->getCouplingRatio();
}

void
RigidDOF::getJointValues(double *jointVals) const
{
	std::list<Joint*>::const_iterator j;
	for(j=jointList.begin();j!=jointList.end();j++) {
		jointVals[ (*j)->getNum() ] = q * getStaticRatio(*j);
	}
}

/*! The RigidDOF stops altogether if a single joint that it controls is stopped.
	If not, it sets all joints based on their ratio to the DOF value */
bool
RigidDOF::accumulateMove(double q1, double *jointVals, int *stoppedJoints)
{
	if ( fabs(q-q1) < 1.0e-5 ) return false;
	std::list<Joint*>::iterator j;
	if (stoppedJoints) {
		for(j=jointList.begin();j!=jointList.end();j++) {
			if ( stoppedJoints[ (*j)->getNum()] ) return false;
		}
	}
	for(j=jointList.begin();j!=jointList.end();j++) {
		jointVals[ (*j)->getNum() ] = q1 * getStaticRatio(*j);
	}
	return true;
}

/*! In the RigidDOF all joint values behave the same way (either they are 
	all stopped or they all move based on the DOF ratios. Therefore it is 
	enough to look at a single joint in the list (such as the first one) 
	to know what the DOF value should be.
*/
void
RigidDOF::updateFromJointValues(const double *jointVals)
{
	assert(!jointList.empty());
	double val;
	if (jointVals) {
		val = jointVals[ jointList.front()->getNum() ];
	}
	else {
		val = jointList.front()->getVal();
	}
	q = val / getStaticRatio(jointList.front());
	DBGP("DOF " << getDOFNum() << ": set value " << q);
}

BreakAwayDOF::~BreakAwayDOF()
{
	if (mInBreakAway) delete [] mInBreakAway;
	if (mBreakAwayValues) delete [] mBreakAwayValues;
}

void BreakAwayDOF::initDOF(Robot *myRobot, const std::list<Joint*> &jList)
{
	DOF::initDOF(myRobot, jList);
	int size = (int)jList.size();
	assert(size>0);
	mInBreakAway = new int[size];
	mBreakAwayValues = new double[size];
	for (int i=0; i<size; i++) {
		mInBreakAway[i] = 0;
		//maybe useful for debugging purposes; should not matter what we put here
		mBreakAwayValues[i] = -10;
	}
}

/*! Each joint is either in breakaway, or connected to the dof
	value by it's static ratio.
*/
void
BreakAwayDOF::getJointValues(double *jointVals) const
{
	int index = -1;
	std::list<Joint*>::const_iterator j;
	for (j=jointList.begin(); j!=jointList.end(); j++) {	
		index++;
		if (mInBreakAway[index] && q > mBreakAwayValues[index]) {
			jointVals[ (*j)->getNum() ] = mBreakAwayValues[index] * getStaticRatio(*j);
		} else {
			jointVals[ (*j)->getNum() ] = q * getStaticRatio(*j);
		}
	}
}

/*! If proximal joints are stopped or in breakaway, distal joints continue to 
	close when the dof is moving in the positive direction.
*/
bool
BreakAwayDOF::accumulateMove(double q1, double *jointVals, int *stoppedJoints)
{
	if ( fabs(q-q1) < 1.0e-5 ) {
		DBGP("DOF new value same as current value");
		return false;
	}
	std::list<Joint*>::iterator j;
	//check if we are moving in the negative direction of the DOF and some joint is stopped
	for (j=jointList.begin(); j!=jointList.end(); j++) {	
		if (stoppedJoints && stoppedJoints[ (*j)->getNum()] ) {
			if (q1 < q){
				DBGP("Joint stopped in the negative direction; stopping all movement");
				return false;
			}
		}
	}

	//we are moving in the positive direction of the DOF
	bool movement = false;
	int index = -1;
	for (j=jointList.begin(); j!=jointList.end(); j++) {	
		index++;
		if (mInBreakAway[index] && q1 > mBreakAwayValues[index]) {
			DBGP("Joint " << index << " is in break away");
			continue;
		}
		double newVal = q1 * getStaticRatio(*j);
		DBGP("q1 " << q1 << " * ratio " << getStaticRatio(*j) << " = " << newVal);
		double oldVal = (*j)->getVal();
		if (stoppedJoints) {
			if (newVal > oldVal && (stoppedJoints[ (*j)->getNum()] & 1) ) {
				DBGP("Joint " << index << " is stopped in the positive direction");
				continue;
			}
			if (newVal < oldVal && (stoppedJoints[ (*j)->getNum()] & 2) ) {
				DBGP("Joint " << index << " is stopped in the negative direction");
				continue;
			}
		}
		jointVals[ (*j)->getNum() ] = newVal;
		movement = true;
	}
	return movement;
}

void
BreakAwayDOF::updateVal(double q1)
{
	std::list<Joint*>::iterator j;
	int index;
	for(j=jointList.begin(), index=0; j!=jointList.end(); j++,index++) {
		double jVal = (*j)->getVal() / getStaticRatio(*j);
		if (mInBreakAway[index]) {
			if (q1 < mBreakAwayValues[index] - 1.0e-5) {
				mInBreakAway[index] = false;
				DBGP("DOF " << dofNum << " joint " << index << " re-engaged");
			}
		} else {
			if (jVal < q1 - 1.0e-5) {
				DBGP("DOF " << dofNum << " joint " << index << " disengaged at " << jVal);
				mInBreakAway[index] = true;
				mBreakAwayValues[index] = jVal;
			}
		}
	}
	q = q1;
}

/*! For this type of dof, the value of any joint that is not in breakaway
	can give us the value of the dof.
*/
void
BreakAwayDOF::updateFromJointValues(const double *jointVals)
{
	std::list<Joint*>::iterator j;
	int index;
	double val;
	for(j=jointList.begin(),index=0; j!=jointList.end(); j++,index++) {
		if (mInBreakAway[index]) continue;
		if (!jointVals) {
			val = (*j)->getVal() / getStaticRatio(*j);
		} else {
			val = jointVals[ (*j)->getNum() ] / getStaticRatio(*j);
		}
		break;
	}
	//all joints in breakaway?
	assert( j!= jointList.end() );
	q = val;
	DBGP("DOF " << getDOFNum() << ": set value " << q);
}

/*! Clears all the breakaway flags, effectively re-engaging all joints. */
void
BreakAwayDOF::reset()
{
	if (!mInBreakAway) return;
	for (int j=0; j<(int)jointList.size(); j++) {
		mInBreakAway[j] = false;
	}
}

bool
BreakAwayDOF::writeToStream(QTextStream &stream)
{
	stream << q;
	for (int j=0; j<(int)jointList.size(); j++) {
		if (!mInBreakAway || !mInBreakAway[j]) {
			stream << " " << 0;
		} else {
			stream << " " << 1 << " " << mBreakAwayValues[j];
		}
	}
	return true;
}

/*! Returns the smallest breakaway value of all the joints
	of this DOF.
*/
double
BreakAwayDOF::getSaveVal() const
{
	if (!mInBreakAway) return q;
	double val = q;
	for (int j=0; j<(int)jointList.size(); j++) {
		if ( mInBreakAway[j] ) {
			if ( mBreakAwayValues[j] < val ) {
				val = mBreakAwayValues[j];
			}
		}
	}
	return val;
}

bool
BreakAwayDOF::readFromStream(QTextStream &stream)
{
	if (stream.atEnd()) return false;
	stream >> q;
	DBGP("Value: " << q);
	for (int j=0; j<(int)jointList.size(); j++) {
		assert(mInBreakAway);
		stream >> mInBreakAway[j];
		DBGP("  bway: " << mInBreakAway[j]);
		if (mInBreakAway[j] == 1) {
			stream >> mBreakAwayValues[j];
		} else if (mInBreakAway[j]==0) {
			mBreakAwayValues[j] = -10;
		} else {
			return false;
		}
	}
	return true;
}

bool
BreakAwayDOF::readParametersFromXml(const TiXmlElement* root)
{
	if (!DOF::readParametersFromXml(root)) return false;
	if(!getDouble(root,"breakAwayTorque", mBreakAwayTorque)||mBreakAwayTorque < 0){
		//the break away torque is missing; warn and use default value
		DBGA("BreakAway torque missing or negative, using default value 0.0");
		mBreakAwayTorque = 0.0f;
	} else {
		//convert to N mm
		mBreakAwayTorque *= 1.0e3;
	}
	return true;
}

/*! We assume that any joint that is in breakaway is applying exactly the 
	breakaway torque. This is probably not accurate, as after breakaway occurs, 
	the joint is disengaged. But it is accurate in the sense that no joint can 
	apply more than the breakaway torque. 
*/
bool
BreakAwayDOF::computeStaticJointTorques(double *jointTorques, double)
{
	std::list<Joint*>::iterator j;
	int index;
	for(j=jointList.begin(), index=0; j!=jointList.end(); j++,index++) {
		if (mInBreakAway[index]) {
			jointTorques[ (*j)->getNum() ] += mBreakAwayTorque;
		}
	}
	return true;
}

void
CompliantDOF::initDOF(Robot *myRobot, const std::list<Joint*> &jList)
{
	DOF::initDOF(myRobot, jList);
	std::list<Joint*>::iterator j;
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		if ( (*j)->getSpringStiffness() == 0.0 ) {
			DBGA("ERROR: Compliant joint has no stiffness! DEFAULT VALUE will be used!");
		}
	}
}

double 
CompliantDOF::getStaticRatio(Joint *j) const {
	double ref = jointList.front()->getSpringStiffness();
	assert(ref!=0.0);
	double k = j->getSpringStiffness();
	if (k == 0.0) k = ref;
	/* this is actually incorrect. I will hard-code in the right relationship
		for the HH, but in the future, todo: generalize this */
	//return j->getCouplingRatio() * ( ref / k );
	int jn = j->getNum();
	if (jn==0 || jn==2 || jn== 4 || jn==6) {
		return j->getCouplingRatio();
	} else {
		double tau2 = j->getCouplingRatio();
		double tau1 = jointList.front()->getCouplingRatio();
		return (ref / k) * ( tau1 * tau2) / (tau1 + tau2);
	} 
}

void 
CompliantDOF::getJointValues(double* jointVals) const 
{
	std::list<Joint*>::const_iterator j;
	for (j=jointList.begin(); j!=jointList.end(); j++) {	
		jointVals[ (*j)->getNum() ] = q * getStaticRatio(*j);
	}
}

/*! Make the assumption that the joint that's most flexed gives the value of the 
	DOF.	This doesn't always work. In general, due to the nature of this DOF 
	and the fact that it is implemented as stateless, link positions can be
	ambiguous, so this function only provides a best effort but no guarantee. 
*/
void 
CompliantDOF::updateFromJointValues(const double* jointVals)
{
	double max = -1.0e5;
	std::list<Joint*>::const_iterator j;
	for (j=jointList.begin(); j!=jointList.end(); j++) {
		double v;
		if (jointVals) {
			v = jointVals[ (*j)->getNum() ] / getStaticRatio(*j);
		} else {
			v = (*j)->getVal() / getStaticRatio(*j);
		}
		max = std::max(max,v);
	}
	q = max;
}

/*! Proximal joints do not affect distal joints, in any direction of movement. */
bool 
CompliantDOF::accumulateMove(double q1, double *jointVals, int *stoppedJoints)
{
	if ( fabs(q-q1) < 1.0e-5 ) {
		DBGP("DOF is already at desired value");
		return false;
	}

	bool movement = false;
	std::list<Joint*>::const_iterator j; int index;
	for (j=jointList.begin(), index=0; j!=jointList.end(); j++, index++) {	
		double newVal = q1 * getStaticRatio(*j);
		double oldVal = (*j)->getVal();
		if (stoppedJoints) {
			if (newVal > oldVal && (stoppedJoints[ (*j)->getNum()] & 1) ) {
				DBGP("Joint " << index << " is stopped in the positive direction");
				continue;
			}
			if (newVal < oldVal && (stoppedJoints[ (*j)->getNum()] & 2) ) {
				DBGP("Joint " << index << " is stopped in the negative direction");
				continue;
			}
		}
		jointVals[ (*j)->getNum() ] = newVal;
		movement = true;
	}
	return movement;
}


/*! We use a linear spring model w. stiffness k for any joint. First we 
	apply the spring force to each joint. Then we say that the dof force 
	must be enough to balance the largest spring force we have found. 
	Therefore, we then apply that dof force to all joints.
*/
bool 
CompliantDOF::computeStaticJointTorques(double *jointTorques, double dofForce)
{
	//this version assumes that dof force is applied as pure joint
	//torque. This is probably not physically accurate for most actual hands
	//dof force ends up as a force on the link, and then joint forces will have
	//to be computed as JT * link_forces

	bool retVal = true;
	//add the spring forces for all joints
	double maxDofTorque = 0.0;
	std::list<Joint*>::iterator j;
	Joint *pj = NULL;
	int count = 0;
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		double springTorque = (*j)->getSpringForce();
		DBGP("Inner spring: " << springTorque);
		jointTorques[ (*j)->getNum() ] -= springTorque;
		//also propagate this to previous joint
		if (count==1 || count==3 || count==5 || count==7) {
			assert(pj);
			vec3 axis1 = (*j)->dynJoint->getPrevLink()->getTran().affine().row(2);
			vec3 axis2 =   pj->dynJoint->getPrevLink()->getTran().affine().row(2);
			double t = fabs(axis1 % axis2);
			//todo what about non-revolute joints, complex kinematic chains, etc...
			jointTorques[pj->getNum()] += springTorque * t;
		}
		pj = (*j);
		count++;
	}
        /*
        std::cerr << "before max torque:\n";
	for(j=jointList.begin(); j!=jointList.end(); j++) {
          std::cerr << jointTorques[ (*j)->getNum() ] << " ";
        }
        std::cerr << "\n";
        */
	//what is the max torque that the dof must balance at any joint
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		double springTorque = - jointTorques[(*j)->getNum()];
		double dofTorque = springTorque / (*j)->getCouplingRatio();
		if (fabs(dofTorque) > fabs(maxDofTorque)) {
			maxDofTorque = dofTorque;
		}
	}
	DBGP("Max dof torque: " << maxDofTorque);

	double dofTorque;
	if (dofForce < 0) {
		//dof force has not been passed in. We thus compute the case where all we
		//want is to keep the joints in their current position. Therefore, the dof
		//force has to balance the max spring force found at any joint
		dofTorque = maxDofTorque;
	} else {
		//we have some level of dof force applied. Use that one.
		//hard-coded 4cm moment arm, same as in setForce. At some point we'll have
		//to use actual insertion points
		dofTorque = force * 40.0;
		if (dofTorque < maxDofTorque) {
			DBGA("For now, dof torque must at least balance spring forces!");
			//give it a little bit of extra woomph
			dofTorque = maxDofTorque + 3.0e7;
		}
	}

	//now apply the dof torque to all joints
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		double jointTorque = dofTorque * (*j)->getCouplingRatio();
		jointTorques[ (*j)->getNum() ] += jointTorque;
		if ( jointTorques[ (*j)->getNum() ] < -1.0e-5) {
			DBGP("Error: negative torque on joint " << (*j)->getNum());
			//now that spring torques are also propagated back, this is no longer an error
			//retVal = false;
		}
                //try to fix numerical errors at least for 0, since it is a special case
                if ( fabs(jointTorques[(*j)->getNum()]) < 1.0e-5) {
                  jointTorques[ (*j)->getNum() ] = 0.0;
                }
		DBGP(jointTorques[ (*j)->getNum() ]);
	}
        /*
        std::cerr << "after max torque:\n";
	for(j=jointList.begin(); j!=jointList.end(); j++) {
          std::cerr << jointTorques[ (*j)->getNum() ] << " ";
        }
        std::cerr << "\n";
        */
	return retVal;
}

/*! We will use one limit constraint for each joint that exceeds is very close to
	or outside its legal range. */
int 
CompliantDOF::getNumLimitConstraints()
{
	int numCon = 0;
	std::list<Joint*>::iterator j;
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		if ( (*j)->getVal() >= (*j)->getMax() - 0.01) numCon ++;
		else if ( (*j)->getVal() <= (*j)->getMin() + 0.01) numCon ++;
	}
	return numCon;
}

/*! One constraint for each joint that has exceeded its legal range. For now this 
	duplicates code from the regular DOF, so everything should probably move someplace
	else.
*/
void 
CompliantDOF::buildDynamicLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
										   double* H, double *g, int &hcn)
{
	std::list<Joint*>::iterator j;
	int count = 0;
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		Joint *currentJoint = *j;
		DynamicBody *prevLink = currentJoint->dynJoint->getPrevLink();
		DynamicBody *nextLink = currentJoint->dynJoint->getNextLink();
		int row, dir;
		double error;
		if (currentJoint->getVal() >= currentJoint->getMax()-0.01) {
			error = currentJoint->getMax() - currentJoint->getVal();
			dir = -1;
		} else if (currentJoint->getVal() <= currentJoint->getMin()+0.01) {
			error = currentJoint->getVal() - currentJoint->getMin();
			dir = +1;
		} else continue;
		    
		g[hcn] = MIN(0.0, error - 0.005);
		DBGP("adding dof limit constraint for DOF " << dofNum << "  error: "<<g[hcn]);

		assert( islandIndices[prevLink]>=0 );
		row = 6*islandIndices[prevLink];
		H[(hcn)*6*numBodies + row+3] -= dir * currentJoint->getWorldAxis()[0];
		H[(hcn)*6*numBodies + row+4] -= dir * currentJoint->getWorldAxis()[1];
		H[(hcn)*6*numBodies + row+5] -= dir * currentJoint->getWorldAxis()[2];

     	assert( islandIndices[nextLink]>=0 );
		row = 6*islandIndices[nextLink];
		H[(hcn)*6*numBodies + row+3] +=  dir * currentJoint->getWorldAxis()[0];
		H[(hcn)*6*numBodies + row+4] +=  dir * currentJoint->getWorldAxis()[1];
		H[(hcn)*6*numBodies + row+5] +=  dir * currentJoint->getWorldAxis()[2];

		hcn++;
		count++;
	}
}

void 
CompliantDOF::setForce(double f)
{
	//cap the force
	if (f > maxForce) force = maxForce;
	if (force < 0) force = 0;
	else if (f < -maxForce) force = -maxForce;
	else force = f;

	//apply it to links. compute moment arms in order to obtain required torque ratios
	//reference is a moment arm of 40mm
	std::list<Joint*>::iterator j;
	for(j=jointList.begin(); j!=jointList.end(); j++) {
		double torqueRatio = (*j)->getCouplingRatio();
		DynamicBody* body = (*j)->dynJoint->getNextLink();
		//assume a pure torque is applied to the joint
		//easier to match in the statics computation
		//assume a moment arm of 40mm for the conversion
		//could keep dof effort as torques in the files, but this is how we have it now
		double jointTorque = force * 40.0 * torqueRatio;
		//since a pure torque is applied, maybe we just need to apply to the next link
		//and let joint constraints take care of the rest?
		body->addTorque(jointTorque * (*j)->getWorldAxis());
		
		/*
		//assume force is applied to the link - probably more correct
		//joint location in body coordinates
		vec3 bodyJoint = (*j)->dynJoint->getNextTrans().translation();
		//vector from joint location to body cog in body coordinates
		vec3 cog = (body->getCoG()  - position::ORIGIN) - bodyJoint;
		//joint axis in body coordinates
		vec3 axis = (*j)->getWorldAxis() * body->getTran().inverse();
		//force is cross product of the two, scaled to desired force magnitude
		vec3 forceVector = force * normalise( axis * cog );
		//scale the arm based on how much torque we want around the joint
		//40mm arm is reference
		vec3 arm = 40.0 * torqueRatio * normalise(cog);
		//position on body
		vec3 bodyPosition = bodyJoint + arm;
		position bodyPos(bodyPosition.x(), bodyPosition.y(), bodyPosition.z());
		body->addForceAtRelPos(forceVector, bodyPos);
		*/
	}
}

/*! A stub for now. Always applies max force no matter what. For now, the simulation of
	the Harvard Hand behaves reasonably like this, but in the future this should be
	fixed.
*/
double
CompliantDOF::PDPositionController(double timeStep)
{
	//todo find a better home for the PD controller (at the DOF level)
	return smoothProfileController(timeStep);
	//return DOF::PDPositionController(timeStep);
	//return getMaxForce();
}

/*! Simply looks at the change in position of this dof. Since it is hard-coded
	to always apply the max force, no motion always means no progress.
*/
bool
CompliantDOF::dynamicsProgress()
{
	double eps = 1.0e-5;
	if ((int)mPositionHistory.size() < mHistoryMaxSize) return true;
	std::list<double>::iterator it = mPositionHistory.begin();
	double ref = *it; it++;
	//fprintf(stderr,"%f: ", ref);
	while (it!=mPositionHistory.end()) {
		//fprintf(stderr,"%f ", *it);
		if ( fabs(*it-ref) > eps) {
			//fprintf(stderr,"move.\n");
			return true;
		}
		it++;
	}
	//fprintf(stderr,"no move.\n");
	return false;
}

/*! Tries to build up the force slowly looking at the time elapsed since the
	last reset. This is something of a hack, but it's the best I could come
	up with to avoid jerking the hand around.
*/
double 
CompliantDOF::smoothProfileController(double)
{
	double time = owner->getWorld()->getWorldTime() - mDynStartTime;
	time = std::min(time, 1.0);
	if (time < 0) {
		DBGA("Zero elapsed time in CD controller");
	}
	time = std::max(time, 0.0);
	return time * getMaxForce();
}
