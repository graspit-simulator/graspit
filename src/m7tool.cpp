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
// Author(s):  Hao Dang (hd2181@columbia.edu)
//
// $Id: 
//
//######################################################################

/*! \file 
  \brief Defines the special %M7tool class
 */

#include "m7tool.h"
#include "world.h"

int
M7Tool::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	//toggle off collision detections
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));
	return result;
}

void M7Tool::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	//toggle off collision detections
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));
} 
