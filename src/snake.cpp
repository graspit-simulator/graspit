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
// Author(s): Hao Dang
//
// $Id: 
//
//######################################################################

/*! \file 
\brief Implements the special %Pr2Gripper robot class
*/

#include "snake.h"
#include "world.h"

int 
Snake::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	myWorld->toggleCollisions(false, base, chainVec[0]->getLink(0));
	myWorld->toggleCollisions(false, base, chainVec[0]->getLink(1));
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[0]->getLink(1));

	return result;
}

void 
Snake::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	myWorld->toggleCollisions(false, base, chainVec[0]->getLink(0));
	myWorld->toggleCollisions(false, base, chainVec[0]->getLink(1));
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[0]->getLink(1));
}
