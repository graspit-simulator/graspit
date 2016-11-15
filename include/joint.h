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
// $Id: joint.h,v 1.20 2009/08/19 23:17:19 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the classes: DHTransform, Joint, RevoluteJoint, and PrismaticJoint.
 */
#ifndef JOINT_H

#include <list>
#include <vector>

#include "mytools.h"
#include "matvec3D.h"

class KinematicChain;
class DynJoint;
class Robot;
class Matrix;

class SoTransform;
class TiXmlElement;

enum JointT {REVOLUTE,PRISMATIC};

//! Converts 4 parameter Denavit-Hartenberg notation to a 4x4 transform.
/*! 
  Stores the 4 Denavit-Hartenberg parameters that define a transform between
  the joints of a robot.  Each joint has its axis aligned with the z-axis of
  the joint frame.  The four values are theta, d, a, and alpha.  Theta is
  the rotation about the z-axis of the current joint frame (this value is
  variable for revolute joints).  D is the translation along the z-axis of
  the joint frame (this value is variable for prismatic joints).  A is the
  transaltion along the x-axis of the joint frame, and alpha is the rotation
  about the x-axis.  
 */
class DHTransform {

  //! Translation vector along the z-axis
  vec3 dtrans;

  //! Translation vector along the x-axis
  vec3 atrans;

  //! Precomputed value of transform 4 times transform 3
  transf tr4TimesTr3;

  //! Translation transform of d along z-axis
  transf tr2;

  //! Rotation transform of theta about z-axis
  transf tr1;

  //! Final transform
  transf tran;

  //! Rotation about z-axis in radians
  double theta;

  //! Translation along z-axis in mm
  double d;

  //! Translation along x-axis in mm
  double a;

  //! Rotation about x-axis in radians
  double alpha;

  //! Re-computes the transform from scratch based on theta, d, a and alpha
  void computeTran();

public:
  DHTransform(double thval=0.0,double dval=0.0,double aval=0.0,
	      double alval=0.0);
  DHTransform( const DHTransform *d ){
	  memcpy( this, d, sizeof(DHTransform));
  }

  /*! Returns the current d value. */
  double getD() const {return d;}

  /*! Returns the current theta value. */
  double getTheta() const {return theta;}

  /*! Returns the current a value. */
  double getA() const {return a;}

  /*! Sets the d value. */
  void setD(double q);

  /*! Sets the theta value. */
  void setTheta(double q);

  /*! Sets the a value and re-computes the transform (from scratch). */
  void setA(double q){a=q; computeTran();}

  /*! Returns the current value of this transform. */
  const transf& getTran() const {return tran;}
  
  /*!
    Returns the value of this transform if \a thetaVal is substituted for
    theta and \a dval is substituted for d. It does not change any values
    within the DHTransform.
  */
  transf getTran(double thetaVal,double dVal) const
    {
      return tr4TimesTr3 * translate_transf(vec3(0,0,dVal)) * 
	rotate_transf(thetaVal,vec3(0,0,1));
    }

};

//! Abstract base class for a single axis robot joint.
/*!
  A robot joint allows either translation or rotation on a single axis.
  It is part of a kinematic chain, which is a serial set of links and joints.
  Multiple joints can be defined between two links.  The value of the joint is
  linearly related to a robot DOF value, and has its own minimum and maximum
  values.  The joint also computes its friction value during dynamic simulation
  using viscous and Coulomb friction values defined in the robot configuration
  file. The joint can also have a spring behavior, and it computes its own 
  spring forces based on a linear stiffness coefficient.
*/
class Joint {
protected:
  //! Index of the robot DOF this joint is connected to
  int DOFnum;

  //! A pointer to the kinematic chain that this joint is a part of
  KinematicChain *owner;

  //! The number of this joint in the robot's list
  int jointNum;

  //! \c TRUE if an Inventor dragger is currently attached to this joint
  bool draggerAttached;

  //! The current value of the joint computed by inverse kinematic during dynamics
  double dynamicsVal;

  //! The current velocity of the joint (first order approximation)
  double velocity;

  //! Joint limit
  double minVal,maxVal;

  //! Linear multiplier of DOF value.  JointVal = mCouplingRatio * (DOFVal) + c
  double mCouplingRatio;  

  //! Joint offset 
  double c;  

  //! The coefficient of viscous friction
  double f1;

  //! Coulomb friction value (constant offset)
  double f0;

  //! Joint spring stiffness, in N * 1.0e6 mm / rad for torque or N * 1.0e6 / mm for force. 
  //! 0 if this joint has no spring.
  double mK;

  //! The rest value of the joint spring
  double mRestVal;

  //! Current joint axis expressed in world coordinates
  vec3 worldAxis;

  //! The DHTransform from this joint frame to the next
  DHTransform *DH;

  //! A pointer to the associated Inventor transform used for joint draggers
  SoTransform *IVTran;

public:
  //! [Temporary] this points to the DynJoint that contains this joint
  DynJoint *dynJoint;

  /*! Initializes all values and pointers to zero.  Joint should set up
  with initJoint.  */
 Joint(KinematicChain *k) : DOFnum(0), owner(k), jointNum(-1), draggerAttached(false), dynamicsVal(0.0),
    velocity(0.0),minVal(0.0),maxVal(0.0),mCouplingRatio(1.0),c(0.0),f1(0.0),f0(0.0),mK(0.0),
	mRestVal(0.0), DH(NULL),
    IVTran(NULL), dynJoint(NULL) {IVTran = new SoTransform(); IVTran->ref();}

  virtual ~Joint();

  /*! Set up the joint using values from an XML DOM read from the robot
	  configuration file.
  */
  virtual int initJointFromXml(const TiXmlElement* root, int jnum) =0;

  /*! Clones this joint from another joint */
  void cloneFrom(const Joint *original);

  /*! This sets the joint value */
  virtual int setVal(double q)=0;

  /*! Sets the minimum joint limit */
  void setMin(double min) {minVal = min;}

  /*! Set the maximum joint limit. */
  void setMax(double max) {maxVal = max;}

  /*! This applies an internal wrench to this joint*/
  virtual void applyInternalWrench(double magnitude)=0;

  /*! Applies internal passive joint wrenches (such as friction or springs) */
  virtual void applyPassiveInternalWrenches();
 
  /*! Sets the current joint velocity (computed during dynamics). */
  void setVelocity(double v) {velocity = v;}

  /*! Sets the current value of the joint (computed during dynamics). */
  void setDynamicsVal(double v) {dynamicsVal = v;}

  /*! Sets the current orientation of the joint axis in world coordinates. 
	  (computed during dynamics) */
  void setWorldAxis(const vec3 &wa) {worldAxis = normalise(wa);}

  /*! Updates \a draggerAttached when a dragger is added or removed from this joint.*/
  void setDraggerAttached(bool b) {draggerAttached = b;}

  //! Sets the linear stiffness coefficient of this joint spring
  void setSpringStiffness(double k) {mK = k;}

  //! Sets the rest value of the attached joint spring
  void setRestValue(double r){mRestVal = r;}

  //Accessors:

  /*! Returns the number of this joint in its kinematic chain */
  int getNum() const {return jointNum;}

  //! Returns the index of the chain this belongs to in the robot scheme
  int getChainNum() const;

  /*! Returns the current joint value. */
  virtual double getVal() const =0;

  //! Returns the current displacement of the joint compared to the rest value
  double getDisplacement() const {return getVal() - mRestVal;}

  /*! Return type of this joint, either REVOLUE or PRISMATIC. */
  virtual JointT getType() const =0;

  /*! Returns the transform to the next joint frame the results from
      substituting \a jointVal for the current joint value. */
  virtual transf getTran(double jointVal) const =0;

  /*! Returns the current joint transform as computed from IK during dynamic
      simulation. */
  virtual transf getDynamicsTran() const =0;

  /*! Returns the index of the robot DOF this joint is connected to. */
  int getDOFNum() const {return DOFnum;}

  /*! Returns the linear multiplier relating this joint value to the DOF value.*/
  double getCouplingRatio() const {return mCouplingRatio;}

  //! Returns the linear stiffness coefficient of this joint spring
  double getSpringStiffness() {return mK;}

  /*! Returns the constant joint offset value.  */
  double getOffset() const {return c;}

  /*! Returns the minimum joint limit */
  double getMin() const {return minVal;}

  /*! Returns the maximum joint limit. */
  double getMax() const {return maxVal;}

  /*! Returns the current velocity of this joint (computed during dynamics).*/
  double getVelocity() const {return velocity;}

  /*! Returns the magnitude of friction acting on this joint.  This uses
     both viscous friction, which is proportional to the joint velocity,
     and Coulomb friction, which is constant. */
  double getFriction() const {
    return -f1 * velocity + (velocity<0 ? f0 : (velocity>0 ? -f0 : 0.0));
  }

  /*! Returns the spring force acting on this joint. This assumes a linear
    spring model, with constant stifness \a mK. Units are N*1.0e6 mm for torque
    or N*1.0e6 for force. */
  double getSpringForce() const;

  /*! Returns the current joint value as computed from the IK during dyanmic
    simulation. */
  double getDynamicsVal() const {return dynamicsVal;}

  /*! Returns the current value of the DHTransform associated with this joint.*/
  transf const& getTran() const {return DH->getTran();}

  /*! Returns the current direction of the joint axis in world coordinates.*/
  vec3 const& getWorldAxis() const {return worldAxis;}

  /*! Returns a pointer to the DHTransform associated with this joint. */
  DHTransform *getDH() {return DH;}

  /*! Returns a pointer to the Inventor transform associated with this joint
    that is used in the joint dragger sub graph.*/
  SoTransform *getIVTran() const {return IVTran;}

  //! The Jacobian relating movement of this joint to movement of a point in the world
  static Matrix jacobian(const Joint *joint, const transf &jointTran, 
						 const transf &toTarget, bool worldCoords);
};


//! A type of joint that translates along the z-axis of the joint frame.
/*!
  A specific type of joint used for linear motion.
*/
class PrismaticJoint : public Joint {

public:

  /*! Stub */
  PrismaticJoint(KinematicChain *k) : Joint(k) {}

  virtual int initJointFromXml(const TiXmlElement* root, int jnum);
  
  virtual void applyInternalWrench(double magnitude);

  virtual int setVal(double q);

  /*! Returns the current value of this joint. */
  virtual double getVal() const {return DH->getD() - c;}

  /*! Returns the type of this joint: PRISMATIC. */
  virtual JointT getType() const {return PRISMATIC;}

  transf getTran(double jointVal) const {
    return DH->getTran(DH->getTheta(),jointVal + c);
  }

  transf getDynamicsTran() const {
    return DH->getTran(DH->getTheta(),dynamicsVal);
  }

};

//! A type of joint that rotates about the z-axis of the joint frame.
/*!
  A specific type of joint used for rotary motion.
*/
class RevoluteJoint : public Joint {

public:

  /*! Stub */
  RevoluteJoint(KinematicChain *k) : Joint(k) {}

  virtual int initJointFromXml(const TiXmlElement* root, int jnum);

  /*! Returns the current value of this joint. */
  virtual double getVal() const {return DH->getTheta() - c;}

  virtual void applyInternalWrench(double magnitude);

  virtual int setVal(double q);

  /*! Returns the type of this joint: REVOLUTE. */
  virtual JointT getType() const {return REVOLUTE;}


  transf getTran(double jointVal) const {
    return DH->getTran(jointVal + c,DH->getD());
  }
  transf getDynamicsTran() const {
    return DH->getTran(dynamicsVal,DH->getD());
  }

};

#define JOINT_H
#endif
