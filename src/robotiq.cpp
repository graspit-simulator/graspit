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
// Author(s): Aaron Blasdel 
//
// $Id: robotiq.cpp,v 1.5 2012/03/21 21:06:53 saint Exp $
//
//######################################################################

/*! \file 
\brief Implements the special %RobotIQ robot class
*/

#include "robotiq.h"
#include "world.h"

int RobotIQ::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	myWorld->toggleCollisions(false, base,chainVec[0]->getLink(1));
	myWorld->toggleCollisions(false, base,chainVec[1]->getLink(1));
	myWorld->toggleCollisions(false, base,chainVec[2]->getLink(1));

	return result;
}

void RobotIQ::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	myWorld->toggleCollisions(false, base,chainVec[0]->getLink(1));
	myWorld->toggleCollisions(false, base,chainVec[1]->getLink(1));
	myWorld->toggleCollisions(false, base,chainVec[2]->getLink(1));
} 
