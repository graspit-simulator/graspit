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
// $Id: dynJoint.h,v 1.11 2009/05/26 20:05:52 cmatei Exp $
//
//######################################################################

/*! \file
\brief Defines the DynJoint classes. 
*/
#ifndef _DYNJOINT_H_

#include <vector>
#include <map>
#include "matvec3D.h"

class Joint;
class DynamicBody;
class Body;
class Matrix;

//! The parent class for each of the joint modules used in the dynamics.
/*! A dynamic joint is different from a regular Joint in that it is not
	restricted to a single degree of freedom of motion. A regular Joint
	defines a single rotation, around 1 axis, or a single translation.
	In contrast, a dynamic joint can alow motion along anything between
	0 and 6 of the degrees of freedom that are possible between two
	bodies in general. The 6 degrees of freedom that are possible between
	two bodies are rotations around the 3 axes and translations around the
	3 axes. Note that here we use the term "degree of freedom" with a 
	different meaning that in the context of a robot DOF. Think of the 
	DynJoint as a "generalized Joint".

	The simplest dynamic joint is the fixed one: is prohibits motion along 
	all 6 possible dof's. A revolute joints only allows one rotation around
	one axis, and fixes the other 5 dofs. A universall joint allows two
	rotations, a ball joint three, etc.

	In GraspIt, dynamic joints are usually made up of the regular joints
	that they correspond to. For example, a revolute dynamic joint will 
	correspond to a single regular Joint in the robot. A universall dynamic
	joint correspdonds to two regular joints, one for each axis of
	rotation, etc. In the future, it would be nice to remove this dual
	notion of regular joint / dynamic joint, but that would also mean
	getting away from defining robots in the D-H notation.

	The DynJoint has two main responsabilities: to assemble its own 
	constraints for the dynamic engine, and to update the values of the 
	regular joints it corresponds to after a dynamic step has been taken.
*/
class DynJoint {
public:
	enum DynamicJointT{FIXED, REVOLUTE, UNIVERSAL, BALL, PRISMATIC};
 protected:
  
  //! The link preceding this joint in the kinematic chain, could be NULL if joint is connected to the world space.
  DynamicBody *prevLink;

  //! The link succeeding this joint in the kinematic chain
  DynamicBody *nextLink;

  //! The position and orientation of the end frame of the preceding link
  transf prevFrame;
    
  //! The position and orientation of the end frame of the succeeding link
  transf nextFrame;

public:

  /*! 
     Initializes the class with pointers and transfroms of the two links
     connected to this joint.
  */
  DynJoint(DynamicBody *pLink,DynamicBody *nLink,const transf &pFrame,
	const transf &nFrame):
    prevLink(pLink),nextLink(nLink),prevFrame(pFrame),nextFrame(nFrame) {}

  /*! Creates the dynamic constraints for this joint.  \a Nu is the joint
      constraint matrix, \a eps is the constraint error.
  */
  virtual void buildConstraints(double *Nu,double *eps,int numBodies,
								std::map<Body*,int> &islandIndices,int &ncn);

  //! Fills in a array of binary chars showing what constraints are enforced by this joint
  virtual void getConstraints(char *c) = 0;

  /*! Update the joint values from the current position of the connected links. */
  virtual void updateValues() = 0;

  /*! Returns the transform from the next link to the coordinate system of this joint */
  virtual transf getNextTrans() = 0;

  /*! Returns the transform from the previous link to the coordinate system of this joint */
  virtual transf getPrevTrans() = 0;

  /*! Returns the number of constrained DOF's for this joint. */
  virtual int getNumConstraints()=0;

  /*! Returns a pointer to the preceding link */
  DynamicBody *getPrevLink() const {return prevLink;}

  /*! Returns a pointer to the succeeding link */
  DynamicBody *getNextLink() const {return nextLink;}

  //! Returns the type of this dynamic joint
  virtual DynamicJointT getType() = 0;

  //! Computes the 6x6 Jacobian of this joint wrt to a point 
  void jacobian(transf toTarget, Matrix *J, bool worldCoords);
};

//! A fixed joint constrains all translations and rotations.
/*!
  A fixed joint completely constrains the relative motion between the two
  connected links.  Or if the prevLink is NULL, it fixes the position of
  the nextLink to its current location in the world.
*/
class FixedDynJoint : public DynJoint {
 public:
  /*! 
     Initializes the class with pointers and transforms of the two links
     connected to this joint.
  */
  FixedDynJoint(DynamicBody *pLink,DynamicBody *nLink,
		const transf &pFrame=transf::IDENTITY,
		const transf &nFrame=transf::IDENTITY) :
    DynJoint(pLink,nLink,pFrame,nFrame) {}
  
  virtual int getNumConstraints() {return 6;}

	/*! This joint enforces all 6 constraints */
   virtual void getConstraints(char *c) {
	   c[0] = c[1] = c[2] = c[3] = c[4] = c[5] = 1;
   }

   virtual void buildConstraints(double *Nu,double *eps,int numBodies,
								std::map<Body*,int> &islandIndices,int &ncn);

  virtual void updateValues();

  virtual transf getNextTrans(){return transf::IDENTITY;}

  virtual transf getPrevTrans(){return prevFrame;}

  virtual DynamicJointT getType(){return FIXED;}
};

//! A revolute joint constrains all 3 translations and 2 rotations.
/*!
  It only allows relative rotation about a single axis.
*/
class RevoluteDynJoint : public DynJoint {
 protected:  
  //! A pointer to the associated joint in the kinematic chain
  Joint *joint;

 public:

  /*! 
     Initializes the class with pointers and transfroms of the two links
     connected to this joint.  It also requires a pointer to the joint
     in the kinematic chain that this dynJoint is associated with.
  */
  RevoluteDynJoint(Joint *j,DynamicBody *pLink,DynamicBody *nLink,
		  const transf &pFrame=transf::IDENTITY,
		  const transf &nFrame=transf::IDENTITY) :
    DynJoint(pLink,nLink,pFrame,nFrame),joint(j) {}
  
  virtual int getNumConstraints() {return 5;}

  //! Enforces all constraints except one axis of rotation
  virtual void getConstraints(char *c) {
    c[0] = c[1] = c[2] = c[3] = c[4] = 1;
	c[5] = 0;
  }
  virtual transf getNextTrans();

  virtual transf getPrevTrans();

  virtual void updateValues();

  virtual DynamicJointT getType(){return REVOLUTE;}
};


//! A universal joint constrains all 3 translations and 1 rotation.
/*!
  A universal joint consits of two co-located revolute joints with
  perpendicular axes.  It constrains the relative translational motion
  between the two connected links as well as relative rotations about an axis
  perpendicular to the two joint axes. 
*/
class UniversalDynJoint : public DynJoint {

  //! Revolute joint connected to the end of the prevLink
  Joint *joint1;

  //! Revolute joint connected to the start of the nextLink
  Joint *joint2;

 public:

  /*! 
     Initializes the class with pointers and transfroms of the two links
     connected to this joint.  It also requires a pointer to the 2 joints
     in the kinematic chain that this dynJoint is associated with.
  */
  UniversalDynJoint(Joint *j1,Joint *j2,DynamicBody *pLink,DynamicBody *nLink,
		    const transf &pFrame=transf::IDENTITY,
		    const transf &nFrame=transf::IDENTITY) :
    DynJoint(pLink,nLink,pFrame,nFrame),joint1(j1),joint2(j2) {}

  virtual int getNumConstraints() {return 4;}

  //! Allows rotation around two axes
  virtual void getConstraints(char *c) {
	c[0] = c[1] = c[2] = c[3] = 1;
	c[4] = c[5] = 0;
  }

  virtual void updateValues();

  virtual transf getNextTrans();

  virtual transf getPrevTrans();

  virtual DynamicJointT getType(){return UNIVERSAL;}
};

//! A ball joint constrains all 3 translations.
/*!
  A ball joint consits of three co-located revolute joints with
  perpendicular axes.  It constrains the relative translational motion
  between the two connected links but not of the rotational motions.
*/
class BallDynJoint : public DynJoint {
  //! Revolute joint connected to the end of the prevLink
  Joint *joint1;

  //! Revolute joint connected to joint1 and joint2
  Joint *joint2;

  //! Revolute joint connected to the start of the nextLink
  Joint *joint3;

 public:
  /*! 
     Initializes the class with pointers and transforms of the two links
     connected to this joint.  It also requires a pointer to the 3 joints
     in the kinematic chain that this dynJoint is associated with.
  */
  BallDynJoint(Joint *j1,Joint *j2,Joint *j3,DynamicBody *pLink,DynamicBody *nLink,
		    const transf &pFrame=transf::IDENTITY,
		    const transf &nFrame=transf::IDENTITY) :
    DynJoint(pLink,nLink,pFrame,nFrame),joint1(j1),joint2(j2),joint3(j3) {}

  virtual int getNumConstraints() {return 3;}

  //! Only enforces translation; allows all rotation axes
  virtual void getConstraints(char *c) {
    c[0] = c[1] = c[2] = 1;
    c[3] = c[4] = c[5] = 0;
  }

  virtual void updateValues();

  virtual transf getNextTrans();

  virtual transf getPrevTrans();

  virtual DynamicJointT getType(){return BALL;}
};

#define _DYNJOINT_H_
#endif
