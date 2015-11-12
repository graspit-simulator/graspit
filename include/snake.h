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
// Author(s):  Hao Dang
//
// $Id: 
//
//######################################################################

/*! \file 
  \brief Defines the special %Pr2Gripper class
 */

#ifndef _SNAKE_H_

#include "robot.h"

//! A special hand because collisions must be turned off between the first links of the two chains
/*! A special hand because collisions must be turned off between the
    first links of the two chains.  This is done by overriding the
    load method.
 */
class Snake : public Hand {
	Q_OBJECT

 public:

  /*! Empty constructor (placeholder) */
  Snake(World *w,const char *name) : Hand(w,name) {}
  
   /*! Performs the normal robot load routine from xml then turns off collisions 
       between the first links of the two chains
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
     the first links of the two chains
 */
  virtual void cloneFrom(Hand *original);

};

#define _SNAKE_H_
#endif
