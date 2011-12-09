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
// $Id: joint.cpp,v 1.31 2009/08/19 23:17:07 cmatei Exp $
//
//######################################################################

/*! \file 
\brief Implements the classes: DHTransform, DOF, RevoluteJoint, and PrismaticJoint.
*/

/* standard C includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/draggers/SoRotateDiscDragger.h>

#include "joint.h"
#include "robot.h"
#include "mytools.h"
#include "body.h"
#include "dynJoint.h"
#include "math/matrix.h"
#include "tinyxml.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

/*! Initializes the DHTransform with the 4 DH parameters. */
DHTransform::DHTransform(double thval,double dval,double aval,double alval) :
theta(thval),d(dval),a(aval),alpha(alval)
{
  computeTran();
}

void DHTransform::computeTran()
{
  transf tr3,tr4;
  
  dtrans[0] = 0.0;
  dtrans[1] = 0.0;
  dtrans[2] = d;
  
  atrans[0] = a;
  atrans[1] = 0.0;
  atrans[2] = 0.0;
  
  tr1 = rotate_transf(theta,vec3(0,0,1));
  tr2 = translate_transf(dtrans);
  tr3 = translate_transf(atrans);
  tr4 = rotate_transf(alpha,vec3(1,0,0));
  tr4TimesTr3 = tr4 * tr3;
  
  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
Sets a new d value for prismatic joints and recomputes the current value
of the transform.
*/
void DHTransform::setD(double q)
{
  d = q;
  dtrans[2] = d;
  tr2 = translate_transf(dtrans);
  
  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
Sets a new theta value for revolute joints and recomputes the current value
of the transform.
*/
void DHTransform::setTheta(double q)
{
  theta = q;
  tr1 = rotate_transf(theta,vec3(0,0,1));
  
  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
Unreferences the associated Inventor transform, and deletes the DHTransform
associated with this joint.
*/
Joint::~Joint()
{
	IVTran->unref(); delete DH;
}

void
Joint::cloneFrom(const Joint *original)
{
	DOFnum  = original->DOFnum;
	jointNum = original->jointNum;
	minVal = original->minVal;
	maxVal = original->maxVal;
	mCouplingRatio = original->mCouplingRatio;
	c = original->c;
	f1 = original->f1;
	f0 = original->f0;
	mK = original->mK;
	mRestVal = original->mRestVal;
	worldAxis = original->worldAxis;
	DH = new DHTransform(original->DH);
	DH->getTran().toSoTransform(IVTran);
}

int
Joint::getChainNum() const
{
	return owner->getNum();
}

void
Joint::applyPassiveInternalWrenches()
{
	double f = getFriction();
	DBGP("Friction coeffs: " << f0 << " " << f1);
	DBGP("Friction force: " << f);
	if (f != 0.0) applyInternalWrench(f);

	f = getSpringForce();
	DBGP("Spring force: " << f);
	applyInternalWrench(-f);
}

/*! Assumes a linear spring with the rest value specified*/
double
Joint::getSpringForce() const 
{
	return mK * getDisplacement();
}

/*! The joint Jacobian is a 6x1 matrix that relates movement of a joint
(1 DOF) to movement (translation and rotation) of a point in 3D space 
placed at world coordinates	\a toTarget. If worldCoords is true, the 
jacobian is expressed in world coordinate system; otherwise, it is
expressed in the local coordinate system of the target point.
*/
Matrix 
Joint::jacobian(const Joint *joint, const transf &jointTran, 
				const transf &toTarget, bool worldCoords)
{
	transf T;
	if (worldCoords) {
		// the translation from joint coordinate system to world coordinate system
		T = transf(Quaternion::IDENTITY, toTarget.translation()) * jointTran.inverse();
	} else {
		T = toTarget * jointTran.inverse();	
	}
	double M[36];
	T.jacobian(M);
	Matrix fullJ(M,6,6,true);
	Matrix J(Matrix::ZEROES<Matrix>(6,1));
	if (joint->getType() == REVOLUTE) {
		//keep rotation against z
		J.copySubBlock(0, 0, 6, 1, fullJ, 0, 5);
	} else if (joint->getType() == PRISMATIC) {
		//keep translation along z
		J.copySubBlock(0, 0, 6, 1, fullJ, 0, 2);
	} else {
		assert(0);
	}
	return J;
}

/*! Initializes a prismatic joint from an XML DOM read from the robot 
	configuration file.  Returns FAILURE if it could not read all the 
	necessary values from the provided XML, otherwise it returns SUCESS.
*/
int
PrismaticJoint::initJointFromXml(const TiXmlElement* root, int jnum)
{
	char dStr[40],num[40],*tmp;
	QString dQStr;
	double theta,d,a,alpha;
	jointNum = jnum;
	const TiXmlElement* element = findXmlElement(root,"d");
	if(element){
		dQStr = element->GetText();
		dQStr = dQStr.stripWhiteSpace();
		strcpy(dStr,dQStr.toStdString().c_str());
	} else {
		return FAILURE;
	}
	if(!getDouble(root,"theta", theta)) return FAILURE;
	if(!getDouble(root,"a", a)) return FAILURE;
	if(!getDouble(root,"alpha", alpha)) return FAILURE;
	if(!getDouble(root,"minValue", minVal)) return FAILURE;
	if(!getDouble(root,"maxValue", maxVal))	return FAILURE;
	if (!getDouble(root,"viscousFriction", f1)) f1 = 0.0;
	if (!getDouble(root,"CoulombFriction", f0)) f0 = 0.0;
	if (!getDouble(root,"springStiffness", mK)) mK = 0.0;
	if (!getDouble(root,"restValue", mRestVal)) mRestVal = 0.0;

	DBGP("thStr: " << theta << " d: " << dStr << " a: " << a << " alpha: " 
		<< alpha << " minVal: " << minVal << " maxVal: " << maxVal << " f1: " 
		<< f1 << " f0:" << f0 << " mK: " << mK << " mRestVal: " << mRestVal);

	//convert to graspit force units which for now seem to be the
	//rather strange N * 1.0e6
	mK *= 1.0e6; 

	theta *= M_PI/180.0;
	alpha *= M_PI/180.0;

	d = 0.0;
	tmp = dStr+1;
	sscanf(tmp,"%[0-9]",num);
	DOFnum = atoi(num);
	tmp += strlen(num);
	if (DOFnum > owner->getOwner()->getNumDOF()) {
		pr_error("DOF number is out of range\n");
		return FAILURE;
	}
	if (*tmp=='*') {
		tmp++;
		sscanf(tmp,"%[0-9.-]",num);
		tmp += strlen(num);
		mCouplingRatio = atof(num);
	}
	if (*tmp=='+') {
		tmp++;
		sscanf(tmp,"%lf",&c);
	}

	DH = new DHTransform(theta,d+c,a,alpha);  
	DH->getTran().toSoTransform(IVTran);

	return SUCCESS;
}

/*!
Sets the current joint value to \a q.  The \a d value of the DHTransform
is then set to \a q + the joint offset \a c. 
*/
int
PrismaticJoint::setVal(double q)
{
	DH->setD(q+c);
	DH->getTran().toSoTransform(IVTran);
	return SUCCESS;  
}

/*!
Applies equal and opposite forces of magnitude \a f along the axis
\a worldAxis to the two links connected to this joint.
*/
void
PrismaticJoint::applyInternalWrench(double magnitude)
{
	dynJoint->getPrevLink()->addForce(-magnitude * worldAxis);
	dynJoint->getNextLink()->addForce(magnitude * worldAxis);
}

/*! Initializes a revolute joint from an XML DOM read from the robot 
	configuration file. This returns FAILURE if it could not read all 
	the necessary values from the provided string, otherwise it 
	returns SUCESS.
*/
int
RevoluteJoint::initJointFromXml(const TiXmlElement* root, int jnum)
{

  QString thQStr;
  char thStr[40],num[40],*tmp;
  double theta,d,a,alpha;
  jointNum = jnum;
  const TiXmlElement* element = findXmlElement(root,"theta");
  if(element){
    thQStr = element->GetText();
    thQStr = thQStr.stripWhiteSpace();
    strcpy(thStr,thQStr.toStdString().c_str());
  }
  else
    return FAILURE;
  if(!getDouble(root,"d", d)) return FAILURE;
  if(!getDouble(root,"a", a)) return FAILURE;
  if(!getDouble(root,"alpha", alpha)) return FAILURE;
  if(!getDouble(root,"minValue", minVal)) return FAILURE;
  if(!getDouble(root,"maxValue", maxVal)) return FAILURE;
  if(!getDouble(root,"viscousFriction", f1)) f1 = 0.0;
  if(!getDouble(root,"CoulombFriction", f0)) f0 = 0.0;
  if(!getDouble(root,"springStiffness", mK)) mK = 0.0;
  if(!getDouble(root,"restValue", mRestVal)) mRestVal = 0.0;
  
  DBGP("thStr: " << thStr << " d: " << d << " a: " << a << " alpha: " 
       << alpha << " minVal: " << minVal << " maxVal: " << maxVal << " f1: " 
       << f1 << " f0:" << f0 << " mK: " << mK << " mRestVal: " << mRestVal);
  
  if (mK < 0) {
    DBGA("Negative spring stiffness");
    return FAILURE;
  }
  
  //convert to graspit units which for now seem to be the
  //rather strange Nmm * 1.0e6
  mK *= 1.0e6; 
  
  alpha *= M_PI/180.0;
  minVal *= M_PI/180.0;
  maxVal *= M_PI/180.0;
  
  theta = 0.0;
  tmp = thStr+1;
  sscanf(tmp,"%[0-9]",num);
  DOFnum = atoi(num);
  tmp += strlen(num);
  
  if (DOFnum > owner->getOwner()->getNumDOF()) {
    pr_error("DOF number is out of range\n");
    return FAILURE;
  }
  
  if (*tmp=='*') {
    tmp++;
    sscanf(tmp,"%[0-9.-]",num);
    tmp += strlen(num);
    mCouplingRatio = atof(num);
  }
  if (*tmp=='+') {
    tmp++;
    sscanf(tmp,"%lf",&c);
    c *= M_PI/180.0;
  }
  
  DH = new DHTransform(theta+c,d,a,alpha);  
  DH->getTran().toSoTransform(IVTran);
  return SUCCESS;
}

/*!
Sets the current joint value to \a q.  The \a theta value of the DHTransform
is then set to \a q + the joint offset \a c.  
*/
int
RevoluteJoint::setVal(double q)
{
  DH->setTheta(q+c);
  DH->getTran().toSoTransform(IVTran);
  return SUCCESS;  
}

/*!
Applies equal and opposite torques of magnitude \a f about the axis
\a worldAxis to the two links connected to this joint.
*/
void
RevoluteJoint::applyInternalWrench(double magnitude)
{
  dynJoint->getPrevLink()->addTorque(-magnitude * worldAxis);
  dynJoint->getNextLink()->addTorque(magnitude * worldAxis);
}
