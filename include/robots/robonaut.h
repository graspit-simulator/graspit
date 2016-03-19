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
