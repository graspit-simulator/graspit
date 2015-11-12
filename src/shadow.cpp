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
