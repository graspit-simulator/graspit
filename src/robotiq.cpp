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
#include "debug.h"

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

bool
RobotIQ::autoGrasp(bool renderIt, double speedFactor, bool stopAtContact)
{
  //not implemented for the RobotIQ yet
  if (myWorld->dynamicsAreOn()) return Hand::autoGrasp(renderIt, speedFactor, stopAtContact);

  if (numDOF != 11) {DBGA("Hard-coded autograsp does not match RobotIQ hand"); return false;}

  std::vector<double> desiredVals(11, 0.0);

  if (speedFactor < 0) 
  {
    DBGA("Hand opening not yet implemented for RobotIQ hand; forcing it to open pose");
    desiredVals[3] = getDOF(3)->getVal();
    desiredVals[7] = getDOF(7)->getVal();
    forceDOFVals(&desiredVals[0]);
    return false;
  }

  std::vector<double> desiredSteps(11, 0.0);

  desiredVals[0] = desiredVals[4] = desiredVals[8] = dofVec[0]->getMax();
  desiredVals[2] = desiredVals[6] = desiredVals[10] = dofVec[2]->getMin();

  desiredSteps[0] = desiredSteps[4] = desiredSteps[8] = speedFactor*AUTO_GRASP_TIME_STEP;
  desiredSteps[2] = desiredSteps[6] = desiredSteps[10] = -speedFactor*AUTO_GRASP_TIME_STEP;

  int steps=0;
  while(1)
  {
    bool moved = moveDOFToContacts(&desiredVals[0], &desiredSteps[0], true, renderIt);
    std::vector<double> ref;
    ref.push_back(0); ref.push_back(4); ref.push_back(8);
    std::vector<double> links;
    links.push_back(0); links.push_back(1); links.push_back(1);
    for(size_t i=0; i<ref.size(); i++)
    {
      if (getChain(i)->getLink(links[i]+0)->getNumContacts() || 
	  getChain(i)->getLink(links[i]+1)->getNumContacts() || 
	  getDOF(ref[i]+0)->getVal() == getDOF(ref[i]+0)->getMax() )
      {
	desiredVals[ref[i]+0] = getDOF(ref[i]+0)->getVal();
	desiredSteps[ref[i]+0] = 0;
	if (!getChain(i)->getLink(links[i]+1)->getNumContacts())
	{
	  desiredVals[ref[i]+1] = getDOF(ref[i]+1)->getMax();
	  desiredSteps[ref[i]+1] = speedFactor*AUTO_GRASP_TIME_STEP;
	}
	else
	{
	  desiredVals[ref[i]+1] = getDOF(ref[i]+1)->getVal();
	  desiredSteps[ref[i]+1] = 0;
	}
	desiredVals[ref[i]+2] = getDOF(ref[i]+2)->getMax();
	desiredSteps[ref[i]+2] = speedFactor*AUTO_GRASP_TIME_STEP;      
      }
    }
    if (!moved) break;
    if (++steps > 1000)
    {
      DBGA("RobotIQ hard-coded autograsp seems to be looping forever; escaping...");
      break;
    }
  }

  return true;
}
