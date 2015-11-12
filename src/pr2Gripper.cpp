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
// Author(s): Matei Ciocarlie (cmatei@cs.columbia.edu)
//
// $Id: 
//
//######################################################################

/*! \file 
\brief Implements the special %Pr2Gripper robot class
*/

#include "pr2Gripper.h"
#include "world.h"

int 
Pr2Gripper::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));

	return result;
}

void 
Pr2Gripper::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));
}
