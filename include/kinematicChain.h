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
// $Id: kinematicChain.h,v 1.20 2009/06/17 21:10:38 saint Exp $
//
//######################################################################

#ifndef _kinematicchain_h_
#define _kinematicchain_h_

#include <QTextStream>
#include <vector>

#include "collisionStructures.h"
#include "matvec3D.h"

class SoSeparator;
class SoTransform;

class Body;
class Link;
class Robot;
class Joint;
class DynJoint;
class Contact;
class Matrix;
class TiXmlElement;

//! A serial chain of links connected by joints
/*! A kinematicChain is a serial chain of links connected by joints.  In a
    robot hand each finger is a kinematic chain.  A chain can also have any
    number of robots attached to its end.  For example, a robot hand can be
    connected to the end of an arm's kinematic chain.  Each chain has a base
    transform that relates the start of the chain to the base frame of the
    robot.  Given values for the joint positions, this class solves the
    forward kinematics for each link in the chain.

	In general, the only way that robot links should ever be moved is 
	through the kinematic chains: the robot asks the dof's for joint values, 
	tells the chains to set those values then tells the chains to update link 
	poses.
*/
class KinematicChain {
  //! Pointer to the robot that this chain is a part of
  Robot *owner;
  
  //! Indicates which chain this is within the robot definition
  int chainNum;
  
  //! The number of the first joint in this chain in the robot's list of joints
  /*! This allows us to compute correspondances between joints in this chain's
	  list and joints in the robot's overall list, which spans all chains.*/
  int firstJointNum;

  //! The number of degrees of freedom in the chain
  int numDOF;

  //! The number of joints in the chain
  int numJoints;

  //! The number of links in the chain
  int numLinks;
  
  //! A vector of pointers to the joints in the chain
  std::vector<Joint *> jointVec;

  //! A vector of pointers to the links in the chain
  std::vector<Link *> linkVec;

  //! For any link, gives us the number of the last joint in the chain that affects its pose
  /*! An array where each element corresponds to a link in the chain and the 
	  value is the index of the last joint affecting the pose of that link. */
  int *lastJoint;

  //! The base transform of the chain relative to the robot base
  transf tran;

  //! The current end transform of the chain relative to the robot base
  transf endTran;

  //! A pointer to the root of the chain's Inventor scene graph
  SoSeparator *IVRoot;

  //! A pointer to the Inventor transform corresponding to the base transform of the chain
  SoTransform *IVTran;

  //! Indicates whether any of the joints in the chain have moved since the last update of the kinematics
  bool jointsMoved;

  //! A vector of child robots connected to the last link of the chain
  std::vector<Robot *> children;

  //! The offset transforms relating the base frame of each child robot to the end transform of the chain
  std::vector<transf> childOffsetTran;

  //! The number of child robots connected to the chain
  int numChildren;

  //! Creates the dynamic joints for each link, based on the regular joints which must already exist
  int createDynamicJoints(const std::vector<int> &dynJointTypes);

  //! Creates a list of the dynamic joints that make up this chain
  void getDynamicJoints(std::vector<DynJoint*> *dj) const;
 
public:
  /*! The constructor is called from a robot's load method, and requires a
      a pointer to the robot owning this chain, and the index of which chain
      this is within the robot.*/
  KinematicChain(Robot *r,int chainNumber, int jointNum);

  ~KinematicChain();

  //! Builds the complete link jacobian wrt to dynamic joints of this chain.
  Matrix linkJacobian(bool worldCoords) const;
  //! Builds the link jacobian but removes the rows corresponding to links that have no contacts
  Matrix activeLinkJacobian(bool worldCoords);
  //! Returns a matrix that only contains the columns of the Jacobian that correspond to actuated joint dofs
  Matrix actuatedJacobian(const Matrix &fullColumnJ) const;

  Matrix jointTorquesVector(Matrix fullRobotTorques);

  //! Returns the number of contacts between links of this chain and a given object
  int getNumContacts(Body *body);

  //! Returns all the contacts between links of this chain and a given object
  std::list<Contact*> getContacts(Body *body);

  /*! Loads the chain information from XML, expected in the standard format
      of GraspIt! .xml robot files */
  int initChainFromXml(const TiXmlElement* root,QString &linkDir);
  
  /*! Creates an indentical copy of another chain, sharing things such as geometry */
  void cloneFrom(const KinematicChain *original);

  //! Sets joint values (without updating poses) based on a vector with values for all robot joints
  void setJointValues(const double *jointVals);
  //! Gets joint values in a vector with values for all robot joints
  void getJointValues(double *jointVals) const;
  //! Compute the link transforms that correspond to a given set of joint values
  void fwdKinematics(const double *jointVals, std::vector<transf> &newLinkTranVec) const;
  //! Compute the locations and orientations of all the joints in this chain for some set of joint values
  void getJointLocations(const double *jointVals, std::vector<transf> &jointTranVec) const;
  //! Compute the link transforms for an infinitesimal joint motion in specified direction
  void infinitesimalMotion(const double *jointVals, std::vector<transf> &newLinkTranVec) const;
  //! Set the transforms of the links to match the current joint values
  void updateLinkPoses();
  //! Keep in the report only those collisions which involve at least a link from this chain
  void filterCollisionReport(CollisionReport &colReport);
  //! Returns the number of the first joint in this chain in the robot numbering scheme
  int getFirstJointNum(){return firstJointNum;}

  //! Returns the index of this chain in the robot's list of chains
  int getNum() const {return chainNum;}

  void attachRobot(Robot *r,const transf &offsetTr);
  void detachRobot(Robot *r);

  /*! Return the robot that owns this chain */
  const Robot* getOwner() const {return owner;}
  /*! Returns the number of joints in the chain */
  int getNumJoints() const {return numJoints;}
  /*! Returns the number of links in the chain */
  int getNumLinks() const {return numLinks;}
  /*! Returns a pointer to the i-th joint in the chain */
  Joint *getJoint(int i) const {return jointVec[i];}
  //! Returns a list of all joints in this chain
  std::list<Joint*> getJoints();
  /*! Returns a pointer to the i-th link in the chain */
  Link *getLink(int i) const {return linkVec[i];}
  /*! Returns the index of the last joint in the chain that affects the pose of link i*/
  int getLastJoint(int i) const {return lastJoint[i];}
  /*! Returns a pointer to the root of the chain's Inventor scene graph */
  SoSeparator *getIVRoot() const {return IVRoot;}
  /*! Returns a pointer to the Inventor transform corresponding to the base transform of the chain */
  SoTransform *getIVTran() const {return IVTran;}
  /*! Returns the base transform of the chain relative to the robot base */
  transf const &getTran() const {return tran;}
  /*! Sets the base transform of the chain relative to the robot base. Does NOT update poses.*/
  void setTran(transf tr){tran = tr;tran.toSoTransform(IVTran);}
  /*! Returns the value of the flag indicating whether the joints have been
    moved since the last time the link poses have been updated */
  bool jointsHaveMoved() const {return jointsMoved;}
  /*! Returns the number of robots attached to the end of the chain */
  int getNumAttachedRobots() const {return numChildren;}
  /*! Returns the i-th robot attached to the end of the chain */
  Robot *getAttachedRobot(int i) const {return children[i];}

  /*! Returns the offset transform between the end of the chain and the base
      of the i-th attached robot*/
  transf const &getAttachedRobotOffset(int i) const{return childOffsetTran[i];}
};

#endif
