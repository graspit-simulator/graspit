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
// $Id: grasp_tester.cpp,v 1.7 2009/06/25 20:26:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_tester.cc                                          */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the grasp_tester class (part of grasp planner)
*/

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
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/SbColor.h>
#include <Inventor/nodes/SoTransform.h>

#ifdef Q_WS_X11
  #include <unistd.h>
  #include <sys/time.h>
#endif

#include <list>

#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "matvec3D.h"
#include "grasp.h"
#include "quality.h"
#include "graspitGUI.h"
#include "grasp_visualization.h"
#include "grasp_tester.h"

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"
PROF_DECLARE(TOTAL_PLANNER);

/* externs; defined in main.cc */
extern grasp_tester *myTester;

/*!
  Initializes variables to default values.
*/
grasp_tester::grasp_tester() : QObject() {

	ivmgr = graspItGUI->getIVmgr();

	projectionViewer = NULL;
    idleSensor = NULL;
    my_hand = NULL;
    my_grasp = NULL;
	dofs = NULL;
    whichQM = -1;
    saveToFile = false;
    maxItStepNr = MAX_ITERATION_STEPS_PER_GRASP;
    backStepSize = BACK_ITERATION_STEP_SIZE;
#ifdef GRASPITDBG
    std::cout << "PL_OUT: Tester created." << std::endl;
#endif
}

/*!
  Puts the hand back where it was originally before testing began.
  If we used a separate world for testing this would not be necessary.
*/
grasp_tester::~grasp_tester(){
    if (my_hand != NULL){
		if (dofs) {
			// put the hand back to starting position
			my_hand->setTran(origTran);
			my_hand->forceDOFVals(dofs);
		}
    }

	if (projectionViewer) delete projectionViewer;
	if (dofs) delete [] dofs;
#ifdef GRASPITDBG
    std::cout << "PL_OUT: Tester destroyed."<< std::endl;
#endif
}

/*!
  Makes local copies of several global values. 
*/
void
grasp_tester::updateGlobals(){
    myViewer = ivmgr->getViewer();
    my_world = ivmgr->getWorld();
    my_hand  = my_world->getCurrentHand();
    my_grasp = my_hand->getGrasp();
    
}

/*!
  Opens a text result file with the name \a filename to which the results
  of the evaluations will be saved.  The results are saved such that each
  line represents one tested grasps.  Each line has the following format:
   - first 3 numbers are the world position of the palm
   - the next 4 are the components of a quaternion representing the hand orientation, 
   - the next 1 is the spread angle of the fingers in radians
   - the last 1 is the value of the quality metric used for evaluation

If \a append is TRUE, the results will be appended to an existing file.
In this case a line of 0's with a quality result of -2 is added at the end
of the file before the new results to mark a new set of grasps.
*/
void
grasp_tester::saveGraspsToFile(const QString& filename,bool append)
{
  if (saveToFile) graspFile.close();

  graspFile.setName(filename);
  if (append) {
	if (graspFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
	  graspOut.setDevice(&graspFile);
	  saveToFile = true;
	  graspOut << "0 0 0 0 0 0 0 0 -2" << endl;
      // the -2 quality marks the start of a new object
	}
  } else {
   if (graspFile.open(QIODevice::WriteOnly)) {
	  graspOut.setDevice(&graspFile);
	  saveToFile = true;
	}
  }
}

/*!
  Deletes the idleSensor so that the testing process will stop, but can be
  resumed later.
*/
void
grasp_tester::pauseTests()
{
  if (idleSensor!=NULL)
	delete idleSensor;
}

/*!
  Creates a new idleSensor to call the testing callback.  This will resume
  testing from wherever it left off.
*/
void
grasp_tester::continueTests()
{
    idleSensor = new SoIdleSensor(testItCB,NULL);
    idleSensor->schedule();
}

/*!
  Begins the testing process.  The list of grasps to test is provided in
  \a graspList_in , and \a render_in controls whether the movement of the hand
  will be shown during the testing process.  It helps to understand what is
  going on, but slows down the process.

  This saves the current hand configuration, so that after testing is complete
  it can be returned to it's original state.  However, we may want to create
  a separate copy of the main world in which to perform the testing, so we
  don't make any changes to the main world.

  An Inventor idleSensor is created to call the testing callback whenever the
  user is idle.  This means that the testing won't interfere with user
  interaction like changing the camera viewpoint.
*/
bool
grasp_tester::callTestIt(std::list<plannedGrasp*>& graspList_in,bool render_in)
{
  /* check if another testing process is running */
  if (idleSensor) return false;
  
  /* get global stuff from ivmgr */
  updateGlobals();
  
  /* check if at least one quality measure exists */
  if (whichQM<0){
#ifdef GRASPITDBG
    std::cout << "PL_OUT: No quality measure specified. Do that first!" << std::endl;
#endif
    return false;
  }
  
  /* check if any planned grasps in list */
  if (graspList_in.empty()){
#ifdef GRASPITDBG
    std::cout << "PL_OUT: Tester received empty grasp list. Nothing happened." << std::endl;
#endif
    return false;
  }
  
  nrOfGrasps = graspList_in.size();
  actualGraspNr = 0;
  
  graspList = &graspList_in;
  render = render_in;
  
  /* Save old hand transformation */
  origTran = my_hand->getTran();
  dofs = new double[my_hand->getNumDOF()];
  for (int i=0; i<my_hand->getNumDOF(); i++){
    dofs[i] = my_hand->getDOF(i)->getVal();
  }
  
  /* set starting iterator for thread */
  it_gr = (*graspList).begin();

  PROF_RESET_ALL;
  PROF_START_TIMER(TOTAL_PLANNER);

  /* start thread */
  idleSensor = new SoIdleSensor(testItCB,NULL);
  idleSensor->schedule();
  
  return true;
}


/*!
  Static callback function called by the idle sensor.  This calls the
  private testIt function to test one grasp.
*/
void 
grasp_tester::testItCB(void *,SoSensor *){
    myTester->testIt();
}


/*!
  This is the main testing function.  It is called whenever the user is idle,
  and it tests the next grasp in the graspList.  It follows the testing
  process defined above.  When the last grasp is reached, the hand is
  returned to its position before the testing began, the graspList is sorted
  in quality order, and the testingComplete signal is emitted.
*/
void 
grasp_tester::testIt()
{
  bool do_iteration;
  bool do_save;
  
  /* Show status bar */
#ifdef GRASPITDBG
  std::cout<<"PL_OUT: Testing grasp no "<< actualGraspNr++<<" out of "<<
    nrOfGrasps<<std::endl;
#endif

  /* Loop over all planned grasps to test them */
  if (it_gr != (*graspList).end()){
    
    do_iteration = false;
    do_save = false;
    
    /* Update visualization */
#ifdef GRASPITDBG
    std::cout << "PL_OUT: vor change color" << std::endl;
#endif

    (*it_gr)->get_graspRepresentation()->changeColor(1.,0.,0.);
    if (render){
      myViewer->render();
      projectionViewer->render(); // this doesn't work!!!!!!
    }

#ifdef GRASPITDBG
    std::cout << "PL_OUT: Put hand in position" << std::endl;
#endif

    /* First, put hand to starting point outside the object */
    if (putIt(*it_gr, render) == SUCCESS){
      
#ifdef GRASPITDBG
      std::cout << "PL_OUT: set preshape" << std::endl;
#endif

      /* Use given preshape */
      preshapeIt((*it_gr)->get_preshape(), render);
      
#ifdef GRASPITDBG
      std::cout << "PL_OUT: test for collisions" << std::endl;
#endif

      /* check if hand collides already with any obstacle (like the table) */
      if (!handCollision()){
	
#ifdef GRASPITDBG
	std::cout << "PL_OUT: move hand towards object" << std::endl;
#endif

	/* Now, move the hand in the specified direction */
	if (moveIt((*it_gr)->get_graspDirection(), render)){
	  
#ifdef GRASPITDBG
	  std::cout << "PL_OUT: check contacts" << std::endl;
#endif
	  /* Check if contact exists between hand and the right object */
	  if (checkContactToHand((*it_gr)->get_graspableBody())){
	    
	    /* Then close the fingers */
	    my_hand->autoGrasp(render,1);
	    //	my_world->findAllContacts();
	    my_world->updateGrasps();
	    //	if (render) myViewer->render();
	    
	    
	    /* Evaluate grasp */
	    (*it_gr)->set_quality(my_grasp->getQM(whichQM)->evaluate());
#ifdef GRASPITDBG
	    std::cout << "PL_OUT: !!!! whichQM: "<<whichQM<<" quality: "<<(*it_gr)->get_quality()<<std::endl;
#endif
	    if (saveToFile) saveGrasp((*it_gr)->get_quality());			
	    /* save final position to grasp class */
	    if ((*it_gr)->get_quality() > QUALITY_MIN_THRESHOLD && (*it_gr)->get_quality() <= 1.0)
	      do_save = true;
	  }
	  else{
	    do_iteration = true;
	  }
	  
	  /* iteration if: 
	     - grasp not stable
	     - wrong contacts */
	  if (do_iteration || 
	      ((*it_gr)->get_quality() <= QUALITY_MIN_THRESHOLD)){

	    if (iteration(**it_gr)){	      
	      /* save final position to grasp class */
	      do_save = true;
	    }
	  }
	}

#ifdef GRASPITDBG
	else std::cout << "PL_OUT: MoveIt failed." << std::endl;
#endif

      }
    }

#ifdef GRASPITDBG
    else 
      std::cout<<"PL_OUT: putIt failed. Stepping to next grasp." << std::endl;
#endif

    if (do_save){
      
      /* change radius in vis window according to quality */
      (*it_gr)->get_graspRepresentation()->changeRadius((*it_gr)->get_quality());
      
      /* save final position to grasp class */
      savePosition(**it_gr);
    }
    else
      (*it_gr)->remove_graspRepresentation();
    
    /* reset color */
    //(*it_gr)->get_graspRepresentation()->resetColor();
    
    /* increment iterator for next step */
    if (it_gr != (*graspList).end())
      it_gr++;
  }
  
  
  /* last grasp reached */
  else{

#ifdef GRASPITDBG
    std::cout << "PL_OUT: Last grasp reached" << std::endl;
#endif

    /* Order List and remove bad grasps */
    orderGraspListByQuality(*graspList);
    if (saveToFile) {graspFile.close(); saveToFile = false;}
    
    /* we are ready; kill idleSensor */
    if (idleSensor != NULL)
      delete idleSensor;
    idleSensor = NULL;
    
    if (render){
      /* put the hand back to starting position */
      my_hand->setTran(origTran);
      my_hand->forceDOFVals(dofs);
    }
 
	PROF_STOP_TIMER(TOTAL_PLANNER);
	PROF_PRINT_ALL;
    emit testingComplete();
    
  }
  if (idleSensor != NULL)
    idleSensor->schedule();
  
  if (!render){
    /* put the hand back to starting position */
    my_hand->setTran(origTran);
    my_hand->forceDOFVals(dofs);
  }
}

/*!
  Saves a completed grasp along with the result of the evaluation as one line
  in the results file.
*/
void
grasp_tester::saveGrasp(double quality){
  graspOut << my_hand->getTran().translation()[0] << " " <<
    my_hand->getTran().translation()[1] << " " <<
    my_hand->getTran().translation()[2] << " " <<
    my_hand->getTran().rotation().w << " " <<
    my_hand->getTran().rotation().x << " " <<
    my_hand->getTran().rotation().y << " " <<
    my_hand->getTran().rotation().z << " " <<
    my_hand->getDOF(0)->getVal() * 180.0/M_PI << " " <<
    quality << endl;
}
    
/*!
  Checks to see if the hand is in collision with anything.
*/
bool 
grasp_tester::handCollision(){

    int numCols = my_world->getCollisionReport(&colReport);

    /*    for (int i=0;i<numCols;i++) {
	if (colReport[i].first->getOwner() == my_hand ||
	    colReport[i].second->getOwner() == my_hand){
	    return true;
	}
	}*/
    return (numCols > 0);
    //    return false;
}

/*!
  Moves the hand a distance of backStepDist back along the grasp approach
  vector, and tries the grasp again.  If the grasp is of minumum quality,
  it saves it and exits with TRUE.  Otherwise it continues stepping the hand
  back and evaluating grasps until the hand collides with another object or
  the fingers no longer touch the object, or the max number of back steps,
  maxItStepNr , is reached.  If no grasp has a quality greater than 0, then
  FALSE is returned.
*/
bool
grasp_tester::iteration(plannedGrasp& pg)
{
  double backStepDist = backStepSize;
  transf actTran;
  vec3 toVec;
  
  for(int step=0; step<maxItStepNr; step++){
    
    /* get actual tran */
    actTran = my_hand->getTran();
    toVec  = actTran.translation() - backStepDist * pg.get_graspDirection().get_dir() / pg.get_graspDirection().get_dir().len();
    
    
    /* take a step back */
    transf to = coordinate_transf(position(toVec.x(), toVec.y(), toVec.z()), 
				  ( - pg.get_fixedFingerDirection()) * 
				  pg.get_graspDirection().get_dir(),
				  - pg.get_fixedFingerDirection());
    
    if (my_hand->setTran(to)) return false;
    
    /* preshape again */
    preshapeIt(pg.get_preshape(), render);
    
    if (render)
      myViewer->render();
    
    if (handCollision()) return false;
    
    /* dont move; close fingers directly */
    my_hand->autoGrasp(render,1);
    //	my_world->findAllContacts();
    my_world->updateGrasps();
    
    //	if (render)
    //	    myViewer->render();
    
    /* if distance is already too big and there are no more contacts,
       the joints have closed to max and more steps won't help */
    if (!checkContactToHand(pg.get_graspableBody())){
      if (my_hand->getName().startsWith("Barrett")){
	/* if joints have closed to max */
	int allClosed = 0;
	for (int i=1; i<4;i++){
	  if (my_hand->getDOF(i)->getVal() == my_hand->getDOF(i)->getMax()){
	    allClosed++;
	  }
	}
	if (allClosed >= 2)
	  return false;
      }
      else return false;
    }
    
    /* Evaluate grasp */
    pg.set_quality(my_grasp->getQM(whichQM)->evaluate());
    
    if (saveToFile) saveGrasp(pg.get_quality());			
    
    /* evaluate, if stable save break */
    if (pg.get_quality() > QUALITY_MIN_THRESHOLD){
      
      return true;
    }
  }
  /* not successful */
  return false;
}

/*!
  Stores the final hand configuration after it has been moved and the grasp
  has performed in the associated candidate grasp record.
*/
void
grasp_tester::savePosition(plannedGrasp& pg){
    finalGraspPosition fgp;
    fgp.set_finalTran(my_hand->getTran());
    std::list<double> dl;
    for (int i=0;i<my_hand->getNumDOF();i++){
	fgp.add_dof(my_hand->getDOF(i)->getVal());
    }
    pg.set_finalGraspPosition(fgp);
}



#ifdef WIN32

/*!
  This function is defined for the WINDOWS version only.  It performs a merge
  sort ordering grasp quality from lowest to highest. The STL sort function 
  seems to work differently, than other STL implementations.
*/
void
sortGrasps(std::list<plannedGrasp*>& grl,
	   std::list<plannedGrasp*>::iterator left,
	   std::list<plannedGrasp*>::iterator right,
	   int size)
{
  if (size > 1) {
    std::list<plannedGrasp*> tempList;
    std::list<plannedGrasp*>::iterator leftCopy = left;
    std::list<plannedGrasp*>::iterator mid;
    std::list<plannedGrasp*>::iterator midCopy;
    int i;
    
    for (mid=left,i=0;i<size/2;i++,mid++);
    sortGrasps(grl,left,mid,size/2);
    sortGrasps(grl,mid,right,size-size/2);
    
    //merge
    midCopy = mid;
    while (leftCopy != mid && midCopy != right) {
      if ((*leftCopy)->get_quality() <= (*midCopy)->get_quality()) {
	tempList.push_back(*leftCopy); leftCopy++;
      }
      else {
	tempList.push_back(*midCopy); midCopy++;
      }
    }
    while (leftCopy != mid) {
      tempList.push_back(*leftCopy); leftCopy++;
    }
    while (midCopy != right) {
      tempList.push_back(*midCopy); midCopy++;
    }
    for (leftCopy = tempList.begin();leftCopy!=tempList.end();leftCopy++,left++)
      *left = *leftCopy;
  }	  
}
#endif

/*!
  Sorts the grasps in \a grl in quality order from highest to lowest,
  and deletes those grasps that have a quality < 0.
*/
void 
grasp_tester::orderGraspListByQuality(std::list<plannedGrasp*>& grl){

  if (grl.empty())
    return;
  
#ifdef WIN32
  //the microsoft stl::list<T>.sort algorithm does not work the same
  //way as other stl implementations, so we sort by hand.
  sortGrasps(grl,grl.begin(),grl.end(),grl.size());
#else
  grl.sort(compareGraspQM());
#endif
  
  while((!grl.empty()) && ((*grl.begin())->get_quality() <= 0.0)){
    delete *(grl.begin());
    grl.pop_front();
  }
  grl.reverse();
  
#ifdef GRASPITDBG
  std::list<plannedGrasp*>::iterator it;
  for (it = grl.begin(); it != grl.end(); it++){
    std::cout << "PL_OUT: QM " << (*it)->get_quality() << std::endl;
  }
#endif

}

/*!
  Examines the contacts on grasped body \a gb , to see if at least one of them
  is between the object and the hand.
*/
bool
grasp_tester::checkContactToHand(GraspableBody *gb){

  std::list<Contact *> contactList = gb->getContacts();
  if (contactList.empty()){
    return false;
  }
  
  std::list<Contact *>::iterator it = contactList.begin();
  
  for (;it!=contactList.end();it++){
    if (((*it)->getBody2()->getOwner() == my_hand) || 
	((*it)->getBody1()->getOwner() == my_hand)){
      return true;
    }
  }
  return false;
}

/*!
  Sets the transform of the hand to the pose of the candidate grasp \a pg .
  If \a render_in is TRUE, the new pose of the hand is rendered.
*/
bool 
grasp_tester::putIt(plannedGrasp* pg, bool render_in){

  int result;
  transf to = 
    coordinate_transf(position(pg->get_graspDirection().get_point().x(),
			       pg->get_graspDirection().get_point().y(),
			       pg->get_graspDirection().get_point().z()), 
		      ( - pg->get_fixedFingerDirection()) * 
		      pg->get_graspDirection().get_dir(),
		      - pg->get_fixedFingerDirection());
  
  result = my_hand->setTran(to);
  if (render_in)
    myViewer->render();
  
  return result;
}

/*!
  Sets the DOF values of the hand to values associated with preshape \a p , and
  if \a render_in is TRUE, this new configuration is rendered.
*/
bool
grasp_tester::preshapeIt(preshape p, bool render_in)
{
  double a,f1,f2,f3;
  p.get_preshape(a,f1,f2,f3);
  double v[4];
  
  v[0]=M_PI/180.0 * a;
  v[1]=M_PI/180.0 * f1;
  v[2]=M_PI/180.0 * f2;
  v[3]=M_PI/180.0 * f3;
  
  my_hand->forceDOFVals(v);
  
  if (render_in)
    myViewer->render();
  return true;
}

/*!
  Moves the hand to a position that is 1 meter away from its current
  position in the direction of the hand approach vector \a gd.  Since
  the hand's moveTo function is used, the motion will stop if it encounters
  any obstructions, i.e. the object to be grasped, along the way.  If 
  \a render_in is TRUE, the result of this motion is rendered.
*/
bool
grasp_tester::moveIt(cartesianGraspDirection gd, bool render_in)
{
   
  transf to(my_hand->getTran().rotation(), 
	    my_hand->getTran().translation() + 1000 * gd.get_dir());

  my_hand->moveTo(to,50*Contact::THRESHOLD,M_PI/36.0);

  if (render_in)
    myViewer->render();
  return true;
}

/*!
  Sets up the grasp visualization window.  If one exists already it is
  deleted.  The window consists of a render area, and the scene graph uses
  the same camera as the main viewer.  The primitives sub-graph, pointed to by
  \a prim , is added using the same transform as \a myBody has in the main
  world.  The candidate grasps can then be added to this scene later.
*/
void 
grasp_tester::setupGraspVisWindow(GraspableBody* myBody,SoGroup *prim)
{

  if (projectionViewer) {
    QWidget *topShell = projectionViewer->getShellWidget();
    delete projectionViewer;
    delete topShell;
  }
  
  /* get global stuff from ivmgr */
  updateGlobals();
  
  SoSeparator *VisTop = new SoSeparator();
  SoTransformSeparator *lightSep = new SoTransformSeparator();
  SoRotation *lightDir = new SoRotation();
  SoSeparator *objSep = new SoSeparator();
  glRoot = new SoSeparator();
  
  lightDir->rotation.connectFrom(&myViewer->getCamera()->orientation);
  lightSep->addChild(lightDir);
  lightSep->addChild(myViewer->getHeadlight());

  objSep->addChild(myBody->getIVTran());
  objSep->addChild(prim);
  
  VisTop->addChild(myViewer->getCamera());
  VisTop->addChild(lightSep);
  VisTop->addChild(objSep);
  VisTop->addChild(glRoot);
  
  projectionViewer = new SoQtRenderArea();
  projectionViewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
  projectionViewer->setBackgroundColor(SbColor(1,1,1));
  projectionViewer->setTitle("GraspIt!");
  
  projectionViewer->setSceneGraph(VisTop);
  projectionViewer->show();
}

/*!
  Given a list of candidate grasps, \a grList , this creates a new grasp
  representation for each one.  These representations can be seen
  in the grasp visualization window.
*/
void 
grasp_tester::visualizePlannedGrasps(std::list<plannedGrasp*> grList)
{

  std::list <plannedGrasp*>::iterator gl_it;
  SbMatrix mat1;
  SbMatrix mat2;
  SbVec3f pos,approach,thumb;
  grasp_representation *gRep;

  for (gl_it=grList.begin(); gl_it!=grList.end(); gl_it++){

    pos.setValue((*gl_it)->get_graspDirection().get_point().x(),
		 (*gl_it)->get_graspDirection().get_point().y(),
		 (*gl_it)->get_graspDirection().get_point().z());

    approach.setValue((*gl_it)->get_graspDirection().get_dir().x(),
		      (*gl_it)->get_graspDirection().get_dir().y(),
		      (*gl_it)->get_graspDirection().get_dir().z());    

    thumb.setValue((*gl_it)->get_fixedFingerDirection().x(),
		   (*gl_it)->get_fixedFingerDirection().y(),
		   (*gl_it)->get_fixedFingerDirection().z());


    /* visualize */
    mat1.setTransform(pos,SbRotation(SbVec3f(0.,1.,0.),approach),
		      SbVec3f(1.,1.,1.));
    
    mat2.setTransform(pos,SbRotation(SbVec3f(0.,1.,0.),thumb),
		      SbVec3f(1.,1.,1.));

    gRep = new grasp_representation(mat1, mat2, glRoot);
    (*gl_it)->set_graspRepresentation(gRep);

  }
  return;
}

/*!
  Sets the testing parameters. \a itStepNr is the max number of backsteps that
  can be taken, and \a stepSize is the length in millimeters of each
  backstep.
*/
bool 
grasp_tester::set_testingParameters(int itStepNr, double stepSize){
    if (itStepNr < 0 || stepSize < 0.0)
	return false;
    maxItStepNr = itStepNr;
    backStepSize = stepSize;
    return true;
}

/*!
  Returns the values of the testing parameters.
*/
void 
grasp_tester::get_testingParameters(int& itStepNr, double& stepSize){
    itStepNr = maxItStepNr;
    stepSize = backStepSize;
    return;
}

/******************
   Local Variables:
   mode:c++
   End:
******************/


