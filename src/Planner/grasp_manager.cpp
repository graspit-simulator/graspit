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
// $Id: grasp_manager.cpp,v 1.8 2009/06/25 20:26:39 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_manager.cc                                         */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the grasp_manager (part of grasp planner)
*/

/* standard c,c++ includes */
#include <stdio.h>
#include <iostream>

#include <QFile>
#include <QTextStream>

/* inventor includes */
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
#include <Inventor/nodes/SoSelection.h>

#ifdef Q_WS_X11
  #include <unistd.h>
#endif

/* graspit includes */
#include "graspitGUI.h"
#include "contact.h"
#include "world.h"
#include "ivmgr.h"
#include "robot.h"
#include "body.h"
#include "grasp.h"

/* STL includes */
#include <list>

/* include grasp planner class */
#include "matvecIO.h"
#include "grasp_coordinates.h"
#include "grasp_directions.h"
#include "grasp_preshape.h"
#include "grasp_visualization.h"
#include "grasp_grasps.h"
#include "grasp_planner.h"
#include "grasp_tester.h"
#include "grasp_presenter.h"
 
#include "grasp_manager.h"

//#define GRASPITDBG


/* globals */
grasp_tester *myTester;

/*!
  Initializes the planning system.  Creates a new planner, a new tester, and
  a new presenter.
*/
grasp_manager::grasp_manager(){
    graspsNotShown = 0;
    renderIt = false;
    doIteration = false;
    itQualThresh = ITERATION_QUALITY_THRESH;
    maxdist = MAX_GRASP_DISTANCE_IN_ITERATION;

    myPlanner = new grasp_planner();
    myTester = new grasp_tester();
    myPresent = new grasp_presenter();

#ifdef GRASPITDBG
    std::cout << "PL_OUT: Grasp Manager built." << std::endl;
#endif
}

/*!
  Deletes any grasps in the graspList , and deletes the planner, tester, and
  presenter.
*/
grasp_manager::~grasp_manager(){
  std::list<plannedGrasp *>::iterator gptr;

  	if (!graspList.empty())
	  for (gptr = graspList.begin();gptr!=graspList.end();gptr++)
		delete *gptr;

    delete myPlanner;
    delete myTester;
    delete myPresent;

#ifdef GRASPITDBG
    std::cout << "PL_OUT: Grasp manager destroyed." << std::endl;
#endif
}


/*!
  Tries to load the shape primitives file for the grasped object.  It uses
  the same file name as the current object, but looks in the primitives folder
  within the objects directory. If the primitives are not found, it uses
  the original geometry of the object, the \a IVGeomRoot.  
*/
void
grasp_manager::loadPrimitives()
{
 
  SoInput myInput;
  char prDir[256];
  QString directory = QString(getenv("GRASPIT"))+
    QString("/models/objects/primitives/");
  QString filename = my_body->getFilename().section('/',-1);
  //make sure the extension is iv, as this is how primitives
  //are stored for now
  filename = filename.section('.',-2,-2) + ".iv";
  QString path = directory + filename;

  printf("Loading primitive %s.\n",path.latin1());

  if (!(myInput.openFile(path.latin1()))) {
    pr_error("could not open primitives file!");
    primitives = my_body->getIVGeomRoot();
    printf ("%s\n",prDir);
    printf("Setting primitive root node to original object.\n");
  }
  else {
      primitives = SoDB::readAll(&myInput);
	  myInput.closeFile();
      if (primitives == NULL) {
	  printf("Load Primitive didnt work, although file seems to exist.\n");
	  printf("Setting primitive root node to original object.\n");
	  primitives = my_body->getIVGeomRoot();
      }
      else {
	primitives->ref();
      }
  }

}

/*!
  If the grasps are not being automatically generated, and only the tester
  is being used, then this method is used to read in the list of candidate
  grasps that should be tested.  It first clears the current graspList , then
  it reads one grasp per line in the file.  Each line should be formatted in
  the following way (including brackets):\n
  
  <tt>[0 0 0][0 0 1][0 1 0]60</tt> \par

  The first vector is the position of the palm.  The second is the palm
  approach direction.  The third is the thumb direction, and the last number
  is the spread angle of the fingers in degrees.
  
  Each grasp is added to the graspList and the primitives are set to the
  geometry of the object itself (IVGeomRoot), since the planner won't be
  used.  However, a window that shows the object and the set of grasps to
  be tested is created in the same way that happens after the planner completes
  the automatic grasp generation.
  
  This function returns SUCCESS or FAILURE depending on whether it was able
  to read the grasps from the file.
*/
int
grasp_manager::readCandidateGraspsFile(const QString& filename)
{
  cartesian_coordinates pt,dir,thumbdir;
  cartesianGraspDirection gd;
  plannedGrasp *pg;
  double spreadAngle;
  std::list<plannedGrasp *>::iterator gptr;


  QFile file(filename);

  if (!file.open(QIODevice::ReadOnly))
    return 1;

  QTextStream stream( &file );

  my_body = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getObject();

  if (my_body == NULL){
    int whichObject = 0;
    if (graspItGUI->getIVmgr()->getWorld()->getNumGB())
      my_body = graspItGUI->getIVmgr()->getWorld()->getGB(whichObject);
    else{
#ifdef GRASPITDBG
      std::cout << "PL_OUT: Nothing selected and no graspable bodies exist. Stop." << std::endl;
#endif
      return 1;
    }
  }
    
  preshape compPreshape;
  if (!graspList.empty())
    for (gptr = graspList.begin();gptr!=graspList.end();gptr++)
      delete *gptr;
  
  graspList.clear();
  
  while (!stream.atEnd()) {
    stream >> pt >> dir >> thumbdir>>spreadAngle; stream.readLine();
#ifdef GRASPITDBG
    std::cout << "read " << pt << dir << thumbdir<<spreadAngle<<std::endl;
#endif
    gd.set_point(pt);
    gd.set_dir(dir);
    pg = new plannedGrasp(gd);
    pg->set_fixedFingerDirection(thumbdir);
    pg->set_graspableBody(my_body);
    compPreshape.set_preshape(spreadAngle,0,0,0);
    pg->set_preshape(compPreshape);
 
    graspList.push_back(pg);
  }
  
#ifdef GRASPITDBG
  std::cout << "PL_OUT: "<<graspList.size()<<" grasps read."<<std::endl;
#endif

  nrOfPlannedGrasps = graspList.size();

  //use original body as primitive since grasps were pregenerated
  primitives = my_body->getIVGeomRoot();
  
  myTester->setupGraspVisWindow( my_body,primitives);
  myTester->visualizePlannedGrasps(graspList);
  
  return 0;
}

/*!
  This method is used to automatically generate grasps for a given object.
  It loads the shape primitives for the grasped object, then calls the planner
  to return a set of candidate grasps for the given object.  Then it creates
  a window that shows the shape primitives and the candidate grasps.
*/
void
grasp_manager::generateGrasps(){
    
    std::list<plannedGrasp*> tmpList;

    my_body = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getObject();

    if (my_body == NULL){
	int whichObject = 0;
	if (graspItGUI->getIVmgr()->getWorld()->getNumGB())
	    my_body = graspItGUI->getIVmgr()->getWorld()->getGB(whichObject);
	else{
#ifdef GRASPITDBG
	    std::cout << "PL_OUT: Nothing selected and no graspable bodies exist. Stop." << std::endl;
#endif
	    return;
	}
    }
    
    loadPrimitives();

    /* change accuracy of planner */
    //changePlanningParameters(nrOfPlanTestIteration);

    tmpList = std::list<plannedGrasp*>(graspList);

    /* get directions and positions */
    graspList = myPlanner->planIt(my_body,primitives);

    /* only keep grasps that are near the good ones from last step, if doIteration is checked */
    if (doIteration)
      compareGraspListsByDist(graspList, tmpList);
    nrOfPlannedGrasps = graspList.size();

    myTester->setupGraspVisWindow( my_body,primitives);
    myTester->visualizePlannedGrasps(graspList);
}

/*!
  Calls the grasp tester with the current set of grasps in the graspList .
*/
void
grasp_manager::testGrasps(){
    /* test all generated directions */
    myTester->callTestIt(graspList, renderIt);
    graspsNotShown=1;
}

/*!
  This takes a list of grasps \a newList, and a set of grasp to compare
  them to, \a criteriaList.  For each grasp in \a newList, it checks the
  distance of that grasp configuration to each of the grasps in the
  \a criteriaList that have a quality greater than the current \a itQualThresh.
  If the distance is less than the current \a maxdist, it keeps the grasp in
  \a newList, otherwise it is erased.  The idea is to keep only those grasps
  that are near other high quality grasps.  However, this is not currently
  part of the user interface and does not get used.
*/
void
grasp_manager::compareGraspListsByDist(std::list<plannedGrasp*>& newList, std::list<plannedGrasp*> criteriaList){
    std::list<plannedGrasp*>::iterator it = newList.begin();
    std::list<plannedGrasp*>::iterator it_tmp;
    std::list<plannedGrasp*>::iterator cr_l_end = criteriaList.end();
    
    bool doErase;
    int size = newList.size();

    if (criteriaList.empty() || newList.empty())
      return;

    for(int i=0; i<size; i++){
      
      doErase = true;
      
      for (std::list<plannedGrasp*>::iterator cr_it = criteriaList.begin(); cr_it != cr_l_end; cr_it++){
	if((*cr_it)->get_quality() >= itQualThresh){
	  double dist = double((*cr_it)->distanceTo(**it));
#ifdef GRASPITDBG
	  std::cout << "PL_OUT: Dist is " << dist << std::endl;
#endif
	  if (dist <= maxdist){
	    doErase = false;
	    break;
	  }
	}
      }
      
      if (doErase){
	it_tmp = it;
	if (it!=newList.end())
	  it++;
	delete (*it_tmp);
	newList.erase(it_tmp);// delete grasp from list
      }
      else if (it!=newList.end())
	it++;
      else break;
    }
    return;
}

//  void 
//  grasp_manager::changePlanningParameters(int step){
    
//      if(step == 0)
//  	return;

//      int nr_of_360_deg_steps_in, 
//  	nr_of_parallel_planes_in,
//  	nr_of_180_deg_grasps_in,
//  	nr_of_grasp_rotations_in;
//      myPlanner->get_planningParameters(nr_of_360_deg_steps_in, 
//  				      nr_of_parallel_planes_in,
//  				      nr_of_180_deg_grasps_in,
//  				      nr_of_grasp_rotations_in);
//      myPlanner->set_planningParameters(nr_of_360_deg_steps_in + DEG_STEPS_STEP, 
//  				      nr_of_parallel_planes_in + PAR_PLANES_STEP,
//  				      DEG_180_STEP,
//  				      nr_of_grasp_rotations_in + ROTATIONS_STEP);
//      if(maxdist > 0.0)
//  	if (maxdist > 5.0)
//  	    maxdist-=2.0;
//  	else if (maxdist > 3.0)
//  	    maxdist-=1.0;
//  	else if (maxdist > 1.0)
//  	    maxdist-=0.5;
//      std::cout << "PL_OUT: MaxDist " << maxdist << std::endl;
//  }

/*!
  After the tester has completed testing all grasps, the \a graspList contains
  all the form closure grasps of the object sorted in quality order (best to
  worst).  This method presents the grasp number \a next from that list to the
  user.  If no grasps have been shown yet, it passes the list to the grasp
  presenter.  Then it calls the presenter's showGrasp for the \a next grasp.
*/
void 
grasp_manager::showGrasps(int next)
{

  if (!graspList.empty()){
    
    if (graspsNotShown){
      
      /* hand the list over to the presenter */
      myPresent->takeList(graspList);
      graspsNotShown=0;
      nrOfStableGrasps = graspList.size();
      //	    computingTime = myTester->get_computingTime();
#ifdef GRASPITDBG
      std::cout << "PL_OUT: " << std::endl << "PL_OUT: " << std::endl;
      std::cout << "PL_OUT: Result grasp generation:" << std::endl;
      std::cout << "PL_OUT: Nr of tested grasps " << nrOfPlannedGrasps << std::endl;
      std::cout << "PL_OUT: Nr of found stable grasps " << nrOfStableGrasps << std::endl;
      
      /* std::cout << "PL_OUT: Testing time: " 
	 << (double)computingTime.tv_sec+(double)computingTime.tv_usec/1000000.0 
	 << " seconds." << std::endl;
	 if (nrOfPlannedGrasps)
	 std::cout << "PL_OUT: That is " 
	 << ((double)computingTime.tv_sec+(double)computingTime.tv_usec/1000000.0)/(double)nrOfPlannedGrasps 
	 << " seconds per planned grasp." << std::endl;
	 if (nrOfStableGrasps)
	 std::cout << "PL_OUT: And " 
	 << ((double)computingTime.tv_sec+(double)computingTime.tv_usec/1000000.0)/(double)nrOfStableGrasps 
	 << " seconds per stable grasp." << std::endl;
      */
      std::cout << "PL_OUT: " << std::endl;
      std::cout << "PL_OUT: " << std::endl;
#endif
	}

    /* now present the best ones to the user */
    myPresent->showGrasp(next, renderIt);
  }

#ifdef GRASPITDBG
  else std::cout << "PL_OUT: No grasps planned yet. Nothing to show." << std::endl;
#endif
}

/*!
  Not used.  Would be for the user to select one of the grasps to use in 
  some way.
*/
void 
grasp_manager::chooseGrasp(){
    myPresent->chooseGrasp();
}

/*!
  Sets the \a renderIt flag to \a in.  Each step of the grasp testing will be
  rendered if \a in is TRUE.
*/
void 
grasp_manager::set_render(bool in){
    renderIt = in;
}

/*!
  Returns the value of the render it flag.
*/
bool 
grasp_manager::get_render()const{
    return renderIt;
}

/*!
  Not used.  Sets the \a doIteration flag to \a in.  If a set of grasps
  has previously been tested and the user wants to compare a new set of
  grasps to the old ones to keep only those close to the old ones, \a in should
  be TRUE.
*/
void 
grasp_manager::set_doIteration(bool in){
    doIteration = in;
}

/*!
  Returns the value of the \a doIteration flag.
*/
bool 
grasp_manager::get_doIteration()const{
    return doIteration;
}

/*!
  Not used because there is no UI for this.  It sets the quality and distance
  thresholds, \a a and \a b, for further iterations of the grasp testing
  process. Returns FALSE and does not perform the operation if either
  parameter is not between 0.0 and 1.0.
*/
bool 
grasp_manager::set_iterationParameters(double a,double b){
    if(a>1.0 || a<0.0 || b>1.0 || b<0.0)
	return false;
    itQualThresh = a;
    maxdist = b;
    return true;
}

/*!
  Returns the iteration parameters, \a itQualThresh and \a maxdist, as \a a
  and \a b.
*/
void 
grasp_manager::get_iterationParameters(double& a,double& b)const{
    a = itQualThresh;
    b = maxdist;
    return;
}

/*!
  Returns a pointer to the graspPlanner.
*/
grasp_planner* 
grasp_manager::get_graspPlanner()const{
    return myPlanner;
}

/*!
  Returns a pointer to the graspTester.
*/
grasp_tester*  
grasp_manager::get_graspTester()const{
    return myTester;
}
