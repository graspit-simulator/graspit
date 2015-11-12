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
// $Id: shadow.h,v 1.3 2009/06/17 21:10:39 saint Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %Shadow robot class
 */

#ifndef SHADOW_H

#include "robot.h"

//! A special hand because collisions must be turned off between the first links of the fourth and fifth fingers
/*! A special hand because collisions must be turned off between the first links of the fourth and fifth fingers
    This is done by overriding the load method.
 */
class Shadow : public Hand {

 public:

  /*! Empty constructor (placeholder) */
  Shadow(World *w,const char *name) : Hand(w,name) {}
  
 /*! Performs the normal robot load routine then turns off collisions between
     the palm and the second link of the thumb.
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

};

#define SHADOW_H
#endif
