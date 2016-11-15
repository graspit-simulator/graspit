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
// $Id: dof.h,v 1.16 2009/07/27 17:00:27 cmatei Exp $
//
//######################################################################

#ifndef _dof_h_
#define _dof_h_

#include <vector>
#include <list>
#include <map>

#include "mytools.h"

class Robot;
class Joint;
class Body;
class Link;

class QTextStream;

//! A single degree of freedom in a robot
/*! The best way to think about a DOF is as a motor, connected to one or
	multiple joints by some transmission type. It is the only way that
	an external user has at his disposal to change the joint positions of a 
	robot. A single DOF must be connected to at least one joint, but can be 
	connected to multiple joints as well. Currently though each joint can 
	only be connected to a single DOF.

	Each DOF has a value associated with it (which usually translates directly
	to a joint angle), limits on that value, a velocity, and a generalized force 
	acting on it. The simplest type of DOF has a 1-to-1 relationship with a joint
	and can be considered as a motor that only affects that joint. In this case, 
	the DOF value will be equal to the joint value, and the dof limits equal to 
	the joint limits.

	More complex DOF's are connected to multiple joints. In this case, depending
	on how the connection is done, different DOF's behave differently. This is the
	reason for having a hierarchy of different DOF types. The main difference is
	what happens when a joint from one DOF is stopped due to contact: how are the
	other joints of the same DOF affected? See implementations of the DOF interface
	for details.

	In static mode, the desired value of the DOF can be set directly, and then the
	DOF itself will tell you what joint values result from that value. In dynamics,
	the only way to go is to apply a force to a DOF and let the dynamic engine 
	compute	body motions in response. The DOF can also simulate controllers to 
	create desired joint motion, although simulating better DOF controllers would 
	be needed.
*/
class DOF {
  friend class Robot;

public:
  enum Type{RIGID, BREAKAWAY, COMPLIANT};

protected:
  //! The robot that this DOF is a part of
  Robot *owner;

  //! The number of this DOF within the robot definition
  int dofNum;

  //! The current value of this DOF
  double q;
  
  //! Tha maximum allowable value
  double maxq;

  //! The minimum allowable value
  double minq;

  //! The desired value
  double desiredq;

  //! The current trajectory set point
  double setPoint;

  double desiredVelocity;
  double defaultVelocity;
  double actualVelocity;
  double maxAccel;

  //! The current force acting on this DOF
  double force;

  //! The maximum force this DOF can exert
  double maxForce;

  //! The sum of external forces acting on the this DOF
  double extForce;

  //! Derivative gain used for PD controller
  double Kv;

  //! Proportional gain used for PD controller
  double Kp;

  //! The dynamics error over the last dynamics steps; most recent is first
  std::list<double> mErrorHistory;

  //! The position of this dof over the last steps
  std::list<double> mPositionHistory;

  //! The velocity of the dof over the last steps
  std::list<double> mVelocityHistory;

  //! The force applied to this dof over the last step
  std::list<double> mForceHistory;

  //! The max number of error values remembered
  int mHistoryMaxSize;

  //! The world time when the current desired trajectory was set
  double mDynStartTime;

  //! A vector of set points
  std::vector<double> trajectory;

  //! Index of the current set point within the trajectory vector
  int currTrajPt;

  //! List of robot joints that are connected to this DOF
  std::list<Joint *> jointList;

  //! How large we want the UI dragger for this DOF to be
  double draggerScale;

public:

  //! Initializes everything to zero.
  DOF();
  //! This copy constructor only copies some of the fields; use with care.
  DOF(const DOF *original);
  //! Stub destructor
  virtual ~DOF() {}
  //! Initializes a DOF based on a list of joints it controls
  virtual void initDOF(Robot *myRobot,const std::list<Joint *>& jList);
  //! Sets the max and min vals of the DOF from the smallest range of the joint limits.
  void updateMinMax();

  //! Returns the type of this DOF
  virtual Type getType() const = 0;
  //! Resets the DOF. This is a fairly abstract concept, see implementations for details.
  virtual void reset() = 0;

  //------------------- STATICS ----------------------
  /*! Computes the values of the joints controlled by this DOF for the case where the DOF 
      value was set to \a q1. The vector \a jointVals lists all the joints of the owning robot, 
	  but this function only affects the values for those joints controlled by this DOF. 
	  \a stoppedJoints is a binary vector that has information about what joints are stopped 
	  (presumably due to some contact). The DOF can take that into account. This vector
	  also has an entry for each joint in the owning robot. */
  virtual bool accumulateMove(double q1, double *jointVals, int *stoppedJoints) = 0;
  /*! Attempts to compute the value of the DOF based on the values of the controlled joints. 
	  \a jointVals lists the joint values to be used. If NULL, the current joint values as 
	  reported by the joints themselves are used */
  virtual void updateFromJointValues(const double *jointVals = NULL) = 0;
  /*! Informs the dof that the robot has set joint values according to the given DOF value. 
	  This typically happens due to some interpolation in joint value space */
  virtual void updateVal(double q1) = 0;
  /*! Gets the values of the joints for the current position of the DOF. 
      Does no checking whatsoever */
  virtual void getJointValues(double *jointVals) const = 0;
  /*! Computes the static forces that are applied at each joint for a given level of DOF force
	  If dofForce is negative, it computes instead the joint forces that have to be applied 
	  just to keep the joints in the given position. This is relevant in the case of coupled 
	  and compliant dof's. */
  virtual bool computeStaticJointTorques(double *jointTorques, double dofForce = -1) = 0;
  /*! Returns the ratio of joint \a j to this dof when static computations are performed */
  virtual double getStaticRatio(Joint *j) const = 0;

  //------------------- DYNAMICS ---------------------
  //! Returns the number of limit constraints that this DOF needs
  virtual int getNumLimitConstraints() = 0;
  //! Computes the dynamic constraints that prevent this DOF from exceeding its range 
  virtual void buildDynamicLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
                                            double* H, double *g, int &hcn) = 0;
  //! Returns the number of dynamic coupling constraints this DOF needs
  virtual int getNumCouplingConstraints() = 0;
  //! Computes the dynamic constraints that ensure that the coupling of this DOF is respected
  virtual void buildDynamicCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
                                               double* Nu, double *eps, int &ncn) = 0;
  /*! Sets the motor force applied on this DOF */
  virtual void setForce(double f) = 0;

  //! Does the bookkeeping and calls an appropriate controller to set the force on the DOF
  virtual void callController(double timeStep);

  //! Calls the PD controller which will set the appropriate force on this DOF
  virtual double PDPositionController(double timeStep);

  //! Returns true if this DOF is still progrssing towards its dynamic target
  virtual bool dynamicsProgress();

  /*! Writes the state of this DOF to a stream in the format that \a readFromStream() can read in*/
  virtual bool writeToStream(QTextStream &stream);
  /*! Reads the state of this DOF from a stream */
  virtual bool readFromStream(QTextStream &stream);
  /*! Reads in the force parameters from XML*/
	virtual bool readParametersFromXml(const TiXmlElement* root);
  /*! Returns the current value. */
  double getVal() const {return q;}
  //! Get a value of this DOF that is later used to restore the state.
  /*! See Robot::storeDOFVals(...) for more details on why this was needed. */
  virtual double getSaveVal() const {return q;}
  /*! Returns the number of this DOF within the robot definition. */
  int getDOFNum() const {return dofNum;}
  /*! Returns the maximum allowable value. */
  double getMax() const { return maxq;}
  /*! Returns the minimum allowable value. */
  double getMin() const { return minq;}

  /*! Returns the desired velocity. */
  double getDesiredVelocity() const {return desiredVelocity;}
  /*! Returns the default velocity. */
  double getDefaultVelocity() const {return defaultVelocity;}
  /*! Returns the actual velocity. */
  double getActualVelocity() const {return actualVelocity;}
  /*! Returns the max. acceleration */
  double getMaxAccel() const {return maxAccel;}
  /*! Returns the current force acting on the DOF. */
  double getForce() const {return force;}
  /*! Returns the desired (target) level of force to be applied to this DOF */
  double getDesiredForce() const {return force;}
  /*! Returns the maximum force that can be applied by this DOF (think torque limit). */
  double getMaxForce() const {return maxForce;}
  /*! Returns the sum of all external forces applied to this DOF */
  double getExtForce() const {return extForce;}
  /*! Returns the dragger scale for this DOF */
  float getDraggerScale() const {return draggerScale;}

  /*! Returns the current trajectory set point. */
  double getSetPoint() const {return setPoint;}
  double getDesiredPos() const {return desiredq;}
  void updateSetPoint();
  void setDesiredPos(double p) {desiredq = MIN(maxq,MAX(minq,p));}
  void setDesiredVelocity(double v) {desiredVelocity = v;}
  void setDefaultVelocity(double v) {defaultVelocity = v;}
  void setTrajectory(double *traj,int numPts);
  void addToTrajectory(double *traj,int numPts);
};

/*! The RigidDOF is the simplest form of DOF. All of its joints are rigidly 
	connected, so if one stops, all stop.
*/
class RigidDOF : public DOF 
{
private:
	//! Checks the most that any of the joints is outside of its legal range by
	double getClosestJointLimit(int *direction);
public:
	RigidDOF() : DOF() {}
	RigidDOF(RigidDOF *original) : DOF(original) {}
	Type getType() const {return RIGID;}

    //------------------- STATICS ---------------------
	//! Any stopped joint stops all joints of this DOF
	virtual bool accumulateMove(double q1, double *jointVals, int *stoppedJoints);
	//! Sets based on first joint in the list
	virtual void updateFromJointValues(const double *jointVals = NULL);
	//! Nothing special to do here
	virtual void updateVal(double q1){q = q1;}
	//! This is empty for the rigid DOF which is entirely state-less, nothing to reset
	virtual void reset(){}
	//! For a rigid DOF, the coupling ratio decides the ratio of movement, since the coupling is rigid
	virtual double getStaticRatio(Joint *j) const;
	//! Each joint only depepends on dof value
	virtual void getJointValues(double *jointVals) const;
	//! Zero everywhere, no static torques
	/*! In the RigidDOF there is never any excess force applied at any joint. All joints are 
		rigidly connected to the DOF, so once a joint is stopped the DOF can not advance any 
		more so no more force is applied. */
	virtual bool computeStaticJointTorques(double*,double){return true;}

    //------------------- DYNAMICS ---------------------
	//! At most one, if any of the joints is close or outside limit
	virtual int getNumLimitConstraints();
	//! Only one, applied to first joint in the DOF
	virtual void buildDynamicLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
                                                  double* H, double *g, int &hcn);
	//! Rigid coupling constraints ensure all joints move together
	virtual int getNumCouplingConstraints(){return jointList.size() - 1;}
	//! Rigid coupling constraints ensure all joints move together
	virtual void buildDynamicCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
                                                     double* Nu, double *eps, int &ncn);
	//! Only sets force to first joint, coupling constraints should take care of rest
	virtual void setForce(double f);
};

/*! The BreakAwayDOF is a model of the Barrett Hand finger DOFs. If a 
	proximal joint is stopped by contact, the transmission breaks away 
	from that joint and distal joints continue to close. However, a joint 
	that has been disengaged is only re-engaged when the DOF goes back to 
	the value that is had when the joint was disengaged.

	Dynamically, breakaway happens when a torque limit on breakaway joints 
	is exceeded. We take that into account in the static joint torque 
	computation. However, for the moment, the dynamics engine does not take 
	that into account.

	This class assumes that breakaway only function in one direction, towards 
	the max value of the dof. In real life this is true, as the barrett hand 
	disengages finger joints only when the finger is closing. In simulation, this 
	also helps because it removes any ambiguity when you have to decide what the 
	dof value is based only on joint values.
*/
class BreakAwayDOF : public RigidDOF
{
private:
	//! Marks which joints controlled by this DOF are disengaged
	int *mInBreakAway;
	//! Saves the breakaway values for disengaged joints
	double *mBreakAwayValues;
	//! The level of torque that a breakaway joint applies before disengaging
	double mBreakAwayTorque;

public:
	Type getType()const {return BREAKAWAY;}
	BreakAwayDOF() : RigidDOF(), mInBreakAway(NULL), mBreakAwayValues(NULL),mBreakAwayTorque(0.0) {}
	BreakAwayDOF(BreakAwayDOF *original) : RigidDOF(original), mInBreakAway(NULL), 
                                               mBreakAwayValues(NULL), 
                                               mBreakAwayTorque(original->mBreakAwayTorque) {}
	~BreakAwayDOF();

	//! Initializes breakaway flags
	void initDOF(Robot *myRobot,const std::list<Joint *>& jList);
	//! Takes breakaway into account
	virtual bool accumulateMove(double q1, double *jointVals, int *stoppedJoints);
	//! Looks at a joint that's not in breakaway
	virtual void updateFromJointValues(const double *jointVals = NULL);
	//! Sets breakaway values if breakaway has occured
	virtual void updateVal(double q1);
	//! Takes breakaway into account
	virtual void getJointValues(double *jointVals) const;
	//! Clears all breakaway flags, re-enages all joints
	virtual void reset();
	//! Applies breakaway torques
	virtual bool computeStaticJointTorques(double *jointTorques, double dofForce = -1);
	//! Returns the breakaway value rather than the current value
	virtual double getSaveVal() const;

	//! Also writes the breakaway status
	virtual bool writeToStream(QTextStream &stream);
	//! Also reads in the breakaway status
	virtual bool readFromStream(QTextStream &stream);
	//!Also reads in the breakaway torque level
	virtual bool readParametersFromXml(const TiXmlElement* root);
};

/*! The compliant DOF is modeled after the Harvard Hand. It assumes compliant 
	joints coupled through tendon actuation. As in the Breakaway DOF, if a 
	proximal joint is stopped, distal joints continue to close. However, there 
	is no breakaway mechanism, to joints that are stopped due to contact are 
	always connected to the DOF.
*/
class CompliantDOF : public DOF
{
private:
	//! Attempts to create a smooth profile directly from current and target positions
	double smoothProfileController(double timeStep);
public:
	CompliantDOF() : DOF() {}
	CompliantDOF(CompliantDOF *original) : DOF(original) {}
	Type getType() const {return COMPLIANT;}

    //------------------- STATICS ---------------------
	//! Makes sure all the joints have spring stiffness defined 
	virtual void initDOF(Robot *myRobot,const std::list<Joint *>& jList);
	//! Static ratio depends on both coupling ratio and joint spring ratio.
	virtual double getStaticRatio(Joint *j) const;
	//! Returns the values as if no contact were present and links were free
	virtual void getJointValues(double* jointVals) const;
	//! Nothing to keep track of here
	virtual void updateVal(double q1){q = q1;}
	//! This can not always work, as states can be ambigous
	virtual void updateFromJointValues(const double* jointVals= NULL);
	//! Distal joints are unaffected by proximal joints
	virtual bool accumulateMove(double q1, double *jointVals, int *stoppedJoints);
	//! Takes joint springs and coupling ratios into account
	virtual bool computeStaticJointTorques(double* jointTorques, double dofForce = -1);
	//! This DOF is stateless, nothing to reset
	virtual void reset(){}

	//------------------- DYNAMICS ---------------------
	//! No dynamic coupling constraints, all joint forces are set explicitly
	virtual int getNumCouplingConstraints(){return 0;}
	//! No dynamic coupling constraints, all joint forces are set explicitly
	virtual void buildDynamicCouplingConstraints(std::map<Body*,int>&, int, 
		double*, double*, int&){}
	//! One constraint for each joint that exceeds it range
	virtual int getNumLimitConstraints();
	//! Computes one constraint for each joint that exceeds it range
	virtual void buildDynamicLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
                                                  double* H, double *g, int &hcn);
	//! Explicitly sets force to all joints in the DOF
	virtual void setForce(double f);
	//! A simple stub for now, always applying max force
	virtual double PDPositionController(double timeStep);
	//! Returns true if this DOF is still progrssing towards its dynamic target
	virtual bool dynamicsProgress();
};

#endif
