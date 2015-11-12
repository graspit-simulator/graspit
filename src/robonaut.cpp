//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Andrew T. Miller 
//
// $Id: robonaut.cpp,v 1.5 2009/06/17 21:06:53 saint Exp $
//
//######################################################################

/*! \file 
\brief Implements the special %Robonaut robot class
*/

#include "robonaut.h"
#include "world.h"

int
Robonaut::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	myWorld->toggleCollisions(false, base,chainVec[0]->getLink(1));

	return result;
}

void Robonaut::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	myWorld->toggleCollisions(false, base,chainVec[0]->getLink(1));
} 
