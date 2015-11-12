//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Andrew T. Miller 
//
// $Id: robonaut.h,v 1.4 2009/06/17 21:10:38 saint Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %Robonaut robot class
 */

#ifndef ROBONAUT_H

#include "robot.h"

//! A special hand because collisions must be turned off between the palm and the second link of the thumb
/*! A special hand because collisions must be turned off between the palm and the second link of the thumb.  This is done by overriding the load method.
 */
class Robonaut : public Hand {
	Q_OBJECT

 public:

  /*! Empty constructor (placeholder) */
  Robonaut(World *w,const char *name) : Hand(w,name) {}
  
   /*! Performs the normal robot load routine from xml then turns off collisions 
       between the palm and the second link of the thumb.
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
     the palm and the second link of the thumb.
 */
  virtual void cloneFrom(Hand *original);
};

#define ROBONAUT_H
#endif
