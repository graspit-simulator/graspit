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
// Authors: Steffen Knoop
//          Andrew T. Miller 
//
// $Id: grasp_presenter.cpp,v 1.3 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_presenter.cc                                       */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the grasp_presenter (part of grasp planner)
*/

/* standard c,c++ includes */
#include <stdio.h>
#include <iostream>


#include <qstring.h>

/* inventor includes */
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SoLists.h>
#include "SoArrow.h"
#include "SoTorquePointer.h"
#include "SoComplexShape.h"
#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoDirectionalLight.h>

#ifdef Q_WS_X11
  #include <unistd.h>
#endif

/* graspit includes */
#include "graspitGUI.h"
#include "contact.h"
#include "ivmgr.h"
#include "world.h"
#include "body.h"
#include "robot.h"
#include "matvec3D.h"
#include "grasp.h"

/* STL includes */
#include <list>

/* include grasp planner class */
#include "grasp_coordinates.h"
#include "grasp_directions.h"
#include "grasp_preshape.h"
#include "grasp_visualization.h"
#include "grasp_grasps.h"
#include "grasp_planner.h"


#include "grasp_presenter.h"

/* externs; defined in main.cc */
extern IVmgr *ivmgr;

/*! 
  Initializes a few variables.
 */
grasp_presenter::grasp_presenter(){
    processing = -1;
    my_hand = NULL;
#ifdef GRASPITDBG
    std::cout << "PL_OUT: Presenter created." << std::endl;
#endif
}

/*!
  Stub destructor.
*/
grasp_presenter::~grasp_presenter(){
#ifdef GRASPITDBG
    std::cout << "PL_OUT: Presenter destroyed." << std::endl;
#endif
}


/*!
  Copies some global pointers for easy local use.
*/
void
grasp_presenter::updateGlobals(){
    ivmgr    = graspItGUI->getIVmgr();
    myViewer = ivmgr->getViewer();
    my_world = ivmgr->getWorld();
    my_hand  = my_world->getCurrentHand();
}

/*!
  Sets the list of grasps to be presented to be \a graspList_in .
*/
void
grasp_presenter::takeList(std::list<plannedGrasp*> graspList_in){
    graspList = graspList_in;
    processing = -1;
}

/*!
  Presents the next planned and tested grasp to the user.  Currently
  this is done in the main window, but it might be better to create
  a separate window for the presentation of the grasp.  The sorted list of
  grasps to present is passed to this class with takeList .  \a next has
  no effect right now.  The idea was to be able to show previous grasps as
  well.  \a render is passed to the putHand and determines whether the
  movement of the hand as it grasps the object is shown or not.
*/
void
grasp_presenter::showGrasp(int next, bool render){

    if (!graspList.empty()){
	updateGlobals();

	/* Here, a separate window should be opened with copies of
	   the hand, the robot, the object and all obstacles */


	/* Show all grasps beginning with the best */
	if (processing == -1){
	    it_gr = graspList.begin();
	    processing = 0;
	}

	if (next >=0) {
#ifdef GRASPITDBG
	  std::cout << "PL_OUT: Showing next grasp." << std::endl;
	}
	else {
	  std::cout<<"PL_OUT: Previous not implemented. Showing next instead." << std::endl;
#endif
	}

	if ((*it_gr)->get_quality() > 0.0){
	    putHand((*it_gr)->get_finalGraspPosition(), render);
#ifdef GRASPITDBG
	    std::cout << "PL_OUT: Grasp Nr " << processing << std::endl;
	    std::cout << "PL_OUT: Quality: " << (*it_gr)->get_quality() << std::endl;
#endif
	    myViewer->render();
	}
	it_gr++;
	processing++;
	if (it_gr == graspList.end()){
	    it_gr = graspList.begin();
	    processing = 0;
	}
    }
#ifdef GRASPITDBG
    else std::cout << "PL_OUT: No grasps planned yet. There's nothin to show." << std::endl;
#endif
}

/*!
  This is intended to allow the user to select which grasp they want to use
  for some operation.  It is not currently used.
*/
void
grasp_presenter::chooseGrasp(){
#ifdef GRASPITDBG
    std::cout << "PL_OUT: YEAH, actual grasp chosen. Congratulations!" << std::endl;
#endif
}

/*!
  Moves the hand to the final grasp position \a fgp , and \a render controls
  whether the motion is rendered or just the final position of the grasp.
*/
void 
grasp_presenter::putHand(finalGraspPosition fgp, bool render){

    std::list<double> tmp = fgp.get_dof();
    std::list<double>::iterator it = tmp.begin();

    my_hand->setTran(fgp.get_finalTran());

    for (int i=0; i<my_hand->getNumDOF(); i++){
	my_hand->forceDOFVal(i,*it);
	if ((my_hand->getName() == "Barrett") && i>0)
	    my_hand->forceDOFVal(i,my_hand->getDOF(i)->getMin());
	if (it != tmp.end())
	    it++;
    }
    if (my_hand->getName() == "Barrett") {
	my_hand->autoGrasp(render,50);
    } 

    my_world->findAllContacts();
    my_world->updateGrasps();
	
}









