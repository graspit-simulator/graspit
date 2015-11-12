//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2008  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Hao Dang (hd2181@columbia.edu)
//
// $Id: 
//
//######################################################################

/*! \file 
  \brief Defines the special %Pr2Gripper class
 */

#ifndef _M7_H_
#define _M7_H_

#include "robot.h"
/*! A special class who loads normally but turns off collisions between
	 all the links in it
*/
class M7 : public Hand {
	Q_OBJECT
public:
  /*! Empty constructor (placeholder) */
  M7(World *w,const char *name) : Hand(w,name) {}
  
 /*! Performs the normal robot load routine then turns off collisions between
	 all the links in the robot
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
	 all the links in the robot
 */
  virtual void cloneFrom(Hand *original);
};

#endif