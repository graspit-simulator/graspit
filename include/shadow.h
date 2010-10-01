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
