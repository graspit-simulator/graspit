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
  \brief Defines the special %M7 class
 */

#include "m7.h"
#include "world.h"

int
M7::loadFromXml(const TiXmlElement* root,QString rootPath)
{
  int result = Robot::loadFromXml(root, rootPath);
  if (result != SUCCESS) return result;
  //toggle off all the collision detection between base and other links
  for(int i = 0; i < 2; ++i){
	  for(int j = 0; j < 6; ++j){
		  myWorld->toggleCollisions(false, base, chainVec[i]->getLink(j));
		  for(int k = j + 1; k < 6; ++k){
			  myWorld->toggleCollisions(false, chainVec[i]->getLink(j), chainVec[i]->getLink(k));
		  }
	  }
  }

  return result;
}

void M7::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
  //toggle off all the collision detection between base and other links
  for(int i = 0; i < 2; ++i){
	  for(int j = 0; j < 6; ++j){
		  myWorld->toggleCollisions(false, base, chainVec[i]->getLink(j));
		  for(int k = j + 1; k < 6; ++k){
			  myWorld->toggleCollisions(false, chainVec[i]->getLink(j), chainVec[i]->getLink(k));
		  }
	  }
  }
} 
