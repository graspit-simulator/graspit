//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2004  Columbia University in the City of New York.
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
// Author: Andrew T. Miller (amiller@cs.columbia.edu)
//
// $Id: pincer.cpp,v 1.2 2009/07/23 21:32:18 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the special %Pincer robot class
 */

#include "pincer.h"
#include "world.h"
#include "grasp.h"

/*! This grasp controller uses the generic robot DOF controller until contacts
    occur on the fingers.  Then if there is a valid grip axis that lies within
    the friction cones of contacts on opposite fingers it will apply the
    approriate torques to prevent the grasped object from slipping out of the
    grasp.
*/
void
Pincer::DOFController(double timeStep)
{
  int d,c,j,l;
  Link *prevLink,*nextLink;
  position jointPos;
  vec3 jointAxis;
  transf jointTran;
  bool dofDone[numDOF];
  int numContacts;

#ifdef GRASPITDBG
  std::cout << " in pincer controller"<<std::endl;
#endif
  Robot::DOFController(timeStep);

  //  base->SetFixedTran(setPoint);

  grasp->CollectContacts();
  numContacts = grasp->getNumContacts();
  
  if (numContacts > 0) {
    grasp->setGripForce(20.0);
    grasp->BuildGraspMap();
    grasp->BuildJacobian();    
    //      if (grasp->FindOptimalGraspForce())
    if (grasp->FindGripAxis()==SUCCESS) {
      for (d=0;d<numDOF;d++) {
#ifdef GRASPITDBG
	printf("setting dof effort %d to: %lf\n",d,grasp->getOptDOFEfforts()[d]);
#endif
	dofVec[d]->SetForce(1.0e+9*grasp->getOptDOFEfforts()[d]);
      }
    }
  }

}
