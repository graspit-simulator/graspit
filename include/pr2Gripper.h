//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2008  Columbia University in the City of New York.
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
// Author(s):  Matei Ciocarlie (cmatei@cs.columbia.edu)
//
// $Id: 
//
//######################################################################

/*! \file 
  \brief Defines the special %Pr2Gripper class
 */

#ifndef PR2GRIPPER_H

#include "robot.h"

//! A special hand because collisions must be turned off between the first links of the two chains
/*! A special hand because collisions must be turned off between the
    first links of the two chains.  This is done by overriding the
    load method.
 */
class Pr2Gripper : public Hand {
	Q_OBJECT

 public:

  /*! Empty constructor (placeholder) */
  Pr2Gripper(World *w,const char *name) : Hand(w,name) {}
  
   /*! Performs the normal robot load routine from xml then turns off collisions 
       between the first links of the two chains
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
     the first links of the two chains
 */
  virtual void cloneFrom(Hand *original);

};

//! Special class only because of the hack that sets eigengrasp limits manually
/*! In searchStateImpl.cpp you will find eigengrasp limits hard-coded in for
  different hands. This is the reason why this hand needs its own class.
*/
class Pr2Gripper2010 : public Hand {
  Q_OBJECT
public:
  enum ComplianceType{NONE = -1, FINGER0 = 0, FINGER1 = 1};
private:
  const static std::string l_gripper_tip_name;
  const static std::string r_gripper_tip_name;

  ComplianceType mCompliance;

 protected:
  virtual void setJointValuesAndUpdate(const double* jointVals);

 public:
  /*! Empty constructor (placeholder) */
 Pr2Gripper2010(World *w,const char *name) : Hand(w,name), mCompliance(NONE) {}

  /*!
   * Assuming that there is an object in range of the hand, runs a reactive
   * grasping routine which closes the hand, finds out which finger hit first,
   * and then incrementally moves the hand and closes the gripper until both fingers
   * make contact (in practice it stops when autograsp indicates that no movement is
   * possible after moving the hand).
   *
   * This will fail if the hand is in collision (interpenetration) with an object, or
   * if there are multiple contacts on the hand (it is not clear what to do in that
   * situation).
   */
  void compliantClose();

  void setCompliance(ComplianceType type){mCompliance = type;}
  ComplianceType getCompliance() const {return mCompliance;}
};

#define PR2GRIPPER_H
#endif
