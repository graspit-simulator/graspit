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
// Author(s): Andrew T. Miller 
//
// $Id: shadow.cpp,v 1.5 2009/06/17 21:06:53 saint Exp $
//
//######################################################################

/*! \file 
  \brief Implements the special %Shadow robot class
 */

#include "shadow.h"
#include "world.h"


int
Shadow::loadFromXml(const TiXmlElement* root,QString rootPath)
{
  int result = Robot::loadFromXml(root, rootPath);
  if (result != SUCCESS) return result;
  myWorld->toggleCollisions(false, base,chainVec[4]->getLink(1));
  myWorld->toggleCollisions(false, chainVec[1]->getLink(0),chainVec[0]->getLink(0));

  return result;
}
