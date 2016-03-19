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
#include "collisionInterface.h"
#include "graspitGUI.h"
#include "mainWindow.h"
#include "contact.h"

//#define GRASPITDBG
#include "debug.h"

#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(REACTIVE_GRASP_TIMER);

using std::string;

const string Pr2Gripper2010::l_gripper_tip_name = "_chain0_link1";
const string Pr2Gripper2010::r_gripper_tip_name = "_chain1_link1";

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


void Pr2Gripper2010::setJointValuesAndUpdate(const double* jointVals)
{
  if (mCompliance == NONE) {
    Robot::setJointValuesAndUpdate(jointVals);
    return;
  }

  transf initial_fingertip_tran = chainVec[mCompliance]->getLink(1)->getTran();
  Robot::setJointValuesAndUpdate(jointVals);
  transf final_fingertip_tran = chainVec[mCompliance]->getLink(1)->getTran();
  transf relative_tran = initial_fingertip_tran * final_fingertip_tran.inverse();
  setTran( relative_tran * getTran() );
}

void Pr2Gripper2010::compliantClose()
{

	DBGP("Reactive grasp started.");
	PROF_TIMER_FUNC(REACTIVE_GRASP_TIMER);

	//Check for collisions in the hand first first. If there are collisions, bail out
	if (!myWorld->noCollision(this)) {
		DBGA("Compliant Close error: the hand currently has collisions");
		return;
	}

	//Close hand
	autoGrasp(false, 1.0);

	//we will close compliantly around the only fingertip that is in contact
	ComplianceType compliance;
	if ( chainVec[0]->getLink(1)->getNumContacts()) {
	  if (chainVec[1]->getLink(1)->getNumContacts()) {
	    DBGA("Reactive grasp: both fingertips are in contact; aborting");
	    return;
	  } else {
	    DBGA("Setting compliance around FINGER0.");
	    compliance = FINGER0;
	  }
	} else {
	  if (!chainVec[1]->getLink(1)->getNumContacts()) {
	    DBGA("Reactive grasp: no fingertips are in contact; aborting");
	    return;
	  } else {
	    DBGA("Setting compliance around FINGER1.");
	    compliance = FINGER1;
	  }
	}
	setCompliance(compliance);

	//disable contacts on fingertip that we are being compliant around
	//as they interfere with the somewhat hacked way in which we do compliance
	chainVec[compliance]->getLink(1)->breakContacts();
	myWorld->toggleCollisions(false, chainVec[compliance]->getLink(1) );

	//Close hand again, this time compliantly
	autoGrasp(false, 1.0);
	
	DBGA("Autograsp complete; re-enabling collisions");
	//re-enable collisions and re-detect contacts
	myWorld->toggleCollisions(true, chainVec[compliance]->getLink(1) );
	myWorld->findContacts(chainVec[compliance]->getLink(1));

	//disable compliance
	setCompliance(NONE);

}

