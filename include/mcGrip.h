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
// Author(s):  Matei Ciocarlie 
//
// $Id: mcGrip.h,v 1.6 2009/09/12 00:14:55 cmatei Exp $
//
//######################################################################

#ifndef _mcgrip_h_
#define _mcgrip_h_

#include <vector>

#include "humanHand.h"
#include "grasp.h"

class Matrix;
class Joint;

//! A type of gripper that we define some particular optimizations for
/*! This is a simple two-fingered gripper that we use as a testbed for
	some optimization methods. We assume some very specific tendon actuation
	methods and parameters that we optimize for. This class can compute some
	of the relationships involved in this optimization, such as the 
	relationship between tendon forces and joint torques.

	For example, we assume a relationship of the form
	tau = f * (B*l + a)
	where f is tendon force, l is a vector of tendon routing parameters and
	B and a depend on the current values of the joints. This class can 
	compute and return B and a.

	For any joint i, the torque tau_i will depend on the tendon routing and 
	joint angles at both joint i and joint i+1. The relationship is of the
	form

	tau_i = a_i + b_i * l_i + c_i * l_i+1

	where a_i, b_i, and c_i depend on the joint values (at joint i and i+1) 
	and l_i and l_i+1 are the tendon routing parameters at the same joints.
	See notes for math details.

	Some of this maybe should (and will) end up in a DOF subclass.

	For now, this inherits from the HumanHand class so that we can use the
	explicit tendon framework that is encapsulated in that class for 
	comparisons. That tendon framework probably need to make it into its
	own DOF subclass at some point as well.
*/
class McGrip : public HumanHand
{
	Q_OBJECT
private:

	//! Computes the contribution of insertion points from a given link on a given joint
	void assembleTorqueMatrices(int refJointNum, int thisJointNum, int nextJointNum,
								double tx, double ty,
								double theta, double theta_n,
								Matrix &B, Matrix &a);

	//! Construction paramter, for now fixed and hard-coded in the constructor
	double mLinkLength;
	//! Construction paramter, for now fixed and hard-coded in the constructor
	double mJointRadius;

public:
	//! Sets some fixed construction parameters and initializes grasp to the specialized class
	McGrip(World *w,const char *name);

	//! Computes the full routing matrices that relate construction parameters to joint torques
	void getRoutingMatrices(Matrix **B, Matrix **a);

	//! Gets a diagonal matrix containing joint displacements in this pose
	void getJointDisplacementMatrix(Matrix **J);

	//! Returns the construction paramters link length
	double getLinkLength() const {return mLinkLength;}

	//! Returns the construction paramters joint radius
	double getJointRadius() const {return mJointRadius;}

	//! Disables passive tendon force application
	virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

	//! Checks for joint equilibrium in the current pose
	int jointTorqueEquilibrium();
};

//! The McGrip also comes with a specialized Grasp class to perform some optimizations
class McGripGrasp : public Grasp
{
	Q_OBJECT
public:
	//! Also check that the hand is indeed a McGrip
	McGripGrasp(Hand *h) : Grasp(h) {
		assert(h->isA("McGrip"));
	}

	//! Optimizes contact forces tendon routes and construction parameters for one particular grasp
	int tendonAndHandOptimization(Matrix *parameters, double &objValRet);

	//! Computes joint torques for this hand, then calls the super for the usual quasistatic equilibrium
	int computeQuasistaticForces(double tendonForce);
};

#endif
