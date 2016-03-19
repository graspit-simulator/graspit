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
// Author(s):  Hao Dang (hd2181@columbia.edu)
//
// $Id: 
//
//######################################################################

/*! \file 
  \brief Defines the special %M7Tool class
 */

#ifndef _M7_TOOL_H_
#define _M7_TOOL_H_

#include "robot.h"
/*! A special class who loads normally but turns off collisions between
	 all the links in it
*/
class M7Tool : public Hand {
	Q_OBJECT
public:
  /*! Empty constructor (placeholder) */
  M7Tool(World *w,const char *name) : Hand(w,name) {}
  
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