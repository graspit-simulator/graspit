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
// Author(s):  Aaron Blasdel
//
//
// $Id: robotiq.h,v 1.5 2012/03/21 21:06:53 saint Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %RobotIQ robot class
 */

#ifndef ROBOTIQ_H

#include "robot.h"

//! A special hand because collisions must be turned off between the palm and the second link of the thumb. This is done by overriding the load method.

class RobotIQ : public Hand {
	Q_OBJECT

 public:

  /*! Empty constructor (placeholder) */
  RobotIQ(World *w,const char *name) : Hand(w,name) {}
  
   /*! Performs the normal robot load routine from xml then turns off collisions 
       between the palm and the second link of the thumb.
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
     the palm and the second link of the thumb.
 */
  virtual void cloneFrom(Hand *original);

  //! Performs RobotIQ-specific autograsp where distal links stay parallel unless proximal
  //  links hit an object.
  virtual bool autoGrasp(bool renderIt, double speedFactor = 1.0, bool stopAtContact = false);

};

#define ROBOTIQ_H
#endif
