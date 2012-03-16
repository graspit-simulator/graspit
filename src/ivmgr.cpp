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
// Author(s): Andrew T. Miller 
//
// $Id: ivmgr.cpp,v 1.65 2009/11/20 23:03:32 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the IVmgr class which handles 3D user interaction.
*/

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <exception>
#include <stdexcept>
#include <typeinfo>

#include <q3listbox.h>
#include <QApplication>
#include <QThread>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoBoxHighlightRenderAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/draggers/SoHandleBoxDragger.h>
#include <Inventor/draggers/SoRotateDiscDragger.h>
#include <Inventor/draggers/SoCenterballDragger.h>
#include <Inventor/draggers/SoTranslate1Dragger.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/fields/SoSFTime.h>
#include <Inventor/manips/SoHandleBoxManip.h>
#include <Inventor/manips/SoCenterballManip.h>
#include <Inventor/nodekits/SoWrapperKit.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoBlinker.h>
#include <Inventor/nodes/SoColorIndex.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoLight.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <Inventor/sensors/SoNodeSensor.h>
#include <Inventor/SoSceneManager.h>
#include <Inventor/Qt/SoQt.h>

#include "pointers.dat"
#include "ivmgr.h"
#include "SoArrow.h"
#include "SoTorquePointer.h"
#include "SoComplexShape.h"
#include "robot.h"
#include "joint.h"
#include "humanHand.h"
#include "body.h"
#include "contact.h"
#include "grasp.h"
#include "world.h"
#include "mainWindow.h"
#include "matvec3D.h"
//hmmm not sure this is right
#include "graspitGUI.h"

//#define GRASPITDBG
#include "debug.h"

#ifdef USE_DMALLOC
include "dmalloc.h"
#endif 

#define FORCE_SCALE 5.0

#ifdef GRASPITDBG
extern FILE *debugfile;
#endif

//! Keeps track of Inventor draggers attached to worldElements or DOF's
/*! 
  A dragger is an Inventor object which allows the user to use the mouse to
  move worldElements or DOF's.  Each time a dragger is created, a new instance
  of this structure is create to keep track of the dragger and information
  related to it.  There are 4 types of draggers currently used:
  <ul>
  <li>For moving the base of a robot, an object or an obstacle:
  <ul>
  <li>A handlebox allows 3D translation of the element
  <li>A centerball allows 3D rotation of the element about an arbitrary point
  </ul>
  <li>For moving a DOF:
  <ul>
  <li>An arrow allows translation of prismatic DOF's
  <li>A disc allows rotation of revolute DOF's
  </ul>
  </ul>
*/
struct DraggerInfo {
  //! Points to the associated robot or object for handleboxes and centerballs 
  WorldElement *selectedElement;

  //! Points to the associated DOF for DOF draggers
  DOF *dof;

  //! Points to the Inventor root of the dragger sub-tree.
  SoSeparator *draggerSep;

  //! Points to the dragger itself
  SoDragger *dragger;

  //! Points to the Inventor transform controlling the position of the dragger relative to
  SoTransform *draggerTran;

  //! The last recorded translation of a handlebox dragger
  SbVec3f lastTran;

  //! The last recorded rotation of a centerball dragger
  SbRotation lastRot;

  //! The last recorded center of rotation of a centerball dragger
  SbVec3f lastCent;
  
  //! The current translation of a centerball dragger
  vec3 centerballTransl;

  //! The last recorded angle or distance for DOF draggers
  double lastVal;
};

//! A list of the current draggers and their related info 
std::list<DraggerInfo *> draggerInfoList;


void
StereoViewer::computeSeekFinalOrientation()
{
	float f = getCamera()->focalDistance.getValue();
	float sb = mFocalPlane / f;
	getCamera()->setBalanceAdjustment( sb );
}


void StereoViewer::setStereo(bool s)
{
	if (s) {
		stereoOn = true;
		setStereoType( SoQtViewer::STEREO_QUADBUFFER );
		getCamera()->setStereoAdjustment(100.0);
		setStereoOffset(10.0);
		computeSeekFinalOrientation();
	} else {
		stereoOn = false;
		setStereoType( SoQtViewer::STEREO_NONE );
	}
}

StereoViewer::StereoViewer(QWidget *parent) : SoQtExaminerViewer(parent) 
{
	stereoOn = false;
	mFocalPlane = 200.0;
}

IVmgr *IVmgr::ivmgr = 0;


/*!
  Initializes the IVmgr instance.  It creates a new World that is the main
  world that the user interacts with.  It creates the examiner viewer that
  allows the user to navigate within the world and interact with it. It sets
  up the root of the entire Inventor scene graph tree as well as sub-nodes
  that handle callbacks for user selections and key presses.  Sub-tree roots
  for draggers and wireframe models, which indicate when bodies are selected,
  are also created.
*/
IVmgr::IVmgr(QWidget *parent, const char *name, Qt::WFlags f) : 
  QWidget(parent,name,f)
{
  ivmgr = this;
  camerafp = NULL;
  CtrlDown = FALSE;
  ShiftDown = FALSE;
  dynForceMat = NULL;

  currTool = TRANSLATE_TOOL;

#ifdef GRASPITDBG
  printf("Starting Inventor...\n");
#endif

  // Initialize the main world
  world = new World(NULL,"mainWorld", this);
  setupPointers();

  // Create the viewer
  myViewer = new StereoViewer(parent);

  //this->setFocusProxy(myViewer->getWidget());

  sceneRoot = new SoSeparator;
  sceneRoot->ref();

  //add this before the mouseEventCB which otherwise captures the click!
  draggerRoot = new SoSeparator;
  sceneRoot->addChild(draggerRoot);

  //add keyboard callback
  SoEventCallback *keyEventNode = new SoEventCallback;
  keyEventNode->addEventCallback(SoKeyboardEvent::getClassTypeId(), keyPressedCB,NULL);
  sceneRoot->addChild(keyEventNode);

  // Add callback to detect if modifier keys are held down during mouse clicks
  SoEventCallback *mouseEventCB = new SoEventCallback;
  mouseEventCB->addEventCallback(SoMouseButtonEvent::getClassTypeId(), shiftOrCtrlDownCB);
  sceneRoot->addChild(mouseEventCB);

  // an empty separator used in the make handlebox routine
  junk = new SoSeparator; junk->ref(); 

  // create and set up the selection node
  selectionRoot = new SoSelection;
  sceneRoot->addChild(selectionRoot);

  // Add selection and deselection callbacks
  selectionRoot->addSelectionCallback(selectionCB, NULL);
  selectionRoot->addDeselectionCallback(deselectionCB, NULL);
  selectionRoot->setPickFilterCallback(pickFilterCB, NULL);
  selectionRoot->addChild(world->getIVRoot());

  wireFrameRoot = new SoSeparator;
  sceneRoot->addChild(wireFrameRoot);

  //comment these out if only single-threaded operation will be used
  //myViewer->setRenderMutex(&mRenderMutex);
  //world->setRenderMutex(&mRenderMutex);

  myViewer->show();
  myViewer->setSceneGraph(sceneRoot);
  myViewer->setTransparencyType(SoGLRenderAction::DELAYED_BLEND);
  myViewer->setBackgroundColor(SbColor(1,1,1));

  myViewer->viewAll();
  mDBMgr = NULL;
  mDBMgr = NULL;
}

//! Not used right now
void
IVmgr::setStereoWindow(QWidget *parent)
{
	StereoViewer *sViewer = new StereoViewer(parent);
	sViewer->show();
	sViewer->setSceneGraph(sceneRoot);
	sViewer->setTransparencyType(SoGLRenderAction::DELAYED_BLEND);
	sViewer->setBackgroundColor(SbColor(1,1,1));
	//sViewer->setCamera( myViewer->getCamera() );
	sViewer->viewAll();
	sViewer->setDecoration(false);
}

/*!
  Deselects all elements, and deletes the main world.  Deletes the remaining
  Inventor scene graph and deletes the Inventor viewer.
*/
IVmgr::~IVmgr()
{
  if (camerafp) fclose(camerafp);
  std::cout << "deleting IVmgr" << std::endl;
  selectionRoot->deselectAll();
  delete world;
  junk->unref();
  if (dynForceMat) dynForceMat->unref();
  pointers->unref();
  sceneRoot->unref();
  delete myViewer;
}

/*!
  Deselects all world elements, deletes the world, and creates a new world.
*/
void
IVmgr::emptyWorld()
{
  selectionRoot->deselectAll();
  selectionRoot->removeChild(world->getIVRoot());
  delete world;
  world = new World(NULL, "MainWorld", this);
  //comment out here and where another world is created to stop using mutexes
  //world->setRenderMutex(&mRenderMutex);
  selectionRoot->addChild(world->getIVRoot());
}

/*!
  Deselects all elements and sets the current tool type.
*/
void IVmgr::setTool(ToolType newTool)
{
  selectionRoot->deselectAll();
  currTool = newTool;
}

/*!
  Tells Inventor to read the binary data stored in the static pointersData to
  create a model of the arrow used in the coordinate axes and force pointers,
  and a model of the torque pointer.
*/
void
IVmgr::setupPointers()
{
  SoInput in;
  in.setBuffer((void *) pointersData, (size_t) sizeof(pointersData));
  pointers = SoDB::readAll(&in);
  //pointers = new SoSeparator;
  pointers->ref();
}

/*!
  Starts the main event loop.
*/
void
IVmgr::beginMainLoop()
{
  //SoQt::show(MainWindow);
  SoQt::mainLoop();
}

/*!
	Draws the force pointers that show the resultant static forces applied
	to objects as a result of coupled robot dofs touching them.
*/
void
IVmgr::drawUnbalancedForces()
{
	for (int b=0; b<world->getNumGB(); b++) {
		drawBodyWrench( world->getGB(b), world->getGB(b)->getExtWrenchAcc() );
	}
}

/*!
  Draws the force and torque pointers that indicate the relative size and 
  orientation of the force and torque components of the worst case disturbance
  wrench for each grasped object.  The pointers are added to the object's
  Inventor sub-graph.
 */
void
IVmgr::drawWorstCaseWrenches()
{
  Grasp *grasp;
  double minWrench[6];

  for (int h=0;h<world->getNumHands();h++) {
    grasp = world->getHand(h)->getGrasp();
	if (!grasp->getObject()) continue;

    // Get the grasp wrench most difficult for this grasp to apply
    grasp->getMinWrench(minWrench);

	//normalise the wrench to be sure we can see it
	double forceScale  = minWrench[0]*minWrench[0] + minWrench[1]*minWrench[1] + minWrench[2]*minWrench[2];
	forceScale = sqrt(forceScale);
	double torqueScale = minWrench[3]*minWrench[3] + minWrench[4]*minWrench[4] + minWrench[5]*minWrench[5];
	torqueScale = sqrt(torqueScale);
	double scale = 5.0 / std::max(forceScale, torqueScale);

    // The worst case disturbance wrench is the opposite of the minWrench
	for (int i=0; i<6; i++) {
		minWrench[i] = -scale * minWrench[i];
	}

	drawBodyWrench( grasp->getObject(), minWrench );
  }
}

void
IVmgr::drawBodyWrench(GraspableBody *body, const double *wrench)
{
    SoSeparator *fSep = NULL;
    SoSeparator *tSep = NULL;
    body->getIVWorstCase()->removeAllChildren();
 
    SbVec3f forceDir(wrench[0],wrench[1],wrench[2]);
    if (forceDir.length() > 0.0) {
      SoArrow *forcePtr = new SoArrow;
      forcePtr->height = forceDir.length()*FORCE_SCALE;
	  if (forceDir.length()*FORCE_SCALE > 200.0) {
		  DBGP("Too long.");
		  forcePtr->height = 200.0;
	  }
      
      SoRotation *fRot = new SoRotation;
      fRot->rotation.setValue(SbRotation(SbVec3f(0,1,0),forceDir));
      
      fSep = new SoSeparator;
      fSep->addChild(fRot);
      fSep->addChild(forcePtr);
    }
    
    SbVec3f torqueDir(wrench[3],wrench[4],wrench[5]);
    if (torqueDir.length() > 0.0) {
      SoTorquePointer *torquePtr = new SoTorquePointer;  
      torquePtr->height = torqueDir.length()*FORCE_SCALE;
	  if (torqueDir.length()*FORCE_SCALE > 200.0) {
		  DBGP("Too long.");
		  torquePtr->height = 200.0;
	  }
      
      SoRotation *tRot = new SoRotation;
      tRot->rotation.setValue(SbRotation(SbVec3f(0,1,0),torqueDir));
      
      tSep = new SoSeparator;
      tSep->addChild(tRot);
      tSep->addChild(torquePtr);
    }
    
    if (fSep || tSep) {
      SoMaterial *mat = new SoMaterial;  
      mat->diffuseColor = SbColor(0.8f,0,0.8f);
      mat->ambientColor = SbColor(0.2f,0,0.2f);
      mat->emissiveColor = SbColor(0.4f,0,0.4f);
      
      SoSeparator *pointerRoot = new SoSeparator;
      pointerRoot->addChild(mat);
      if (fSep) pointerRoot->addChild(fSep);
      if (tSep) pointerRoot->addChild(tSep);
      
      // Remove previous worstcase pointers from the object and add new ones
      body->getIVWorstCase()->addChild(pointerRoot);
    }
}

/*!
  This routine adds arrows within each of the friction cones that indicate
  the size of the dynamic contact forces that were solved for.
  This still doesn't handle the display of any torsional forces at soft
  finger contacts.  If dynamics are not on, blinker nodes are added so that
  when a contact force is selected from the list the associated contact force
  indicator will blink.
*/
void
IVmgr::drawDynamicForces()
{
	std::list<Contact *> contactList;
	std::list<Contact *>::iterator cp;
	SbVec3f forceVec;
	double *contactForce;

	if (!dynForceMat) {
		dynForceMat = new SoMaterial;
		dynForceMat->diffuseColor = SbColor(0.8f,0.8f,0);
		dynForceMat->ambientColor = SbColor(0.2f,0.2f,0);
		//dynForceMat->emissiveColor = SbColor(0.8,0.8,0);
		dynForceMat->ref();
	}

	int numContacts=0;
	if (!world->dynamicsAreOn()) {
		//count the number of contacts
		for (int b=0; b<world->getNumGB(); b++) {
			int sz = world->getGB(b)->getContacts().size();
			numContacts += sz;
			int numChildren = world->getGB(b)->getIVContactIndicators()->getNumChildren();
			for (int i=numChildren-1; i>=sz; i--){
				world->getGB(b)->getIVContactIndicators()->removeChild(i);
			}
		}
		contactForceBlinkerVec.clear();
		if (numContacts == 0) return;
		contactForceBlinkerVec.reserve(numContacts);
	}

	for (int b=0; b<world->getNumGB(); b++) {
		contactList = world->getGB(b)->getContacts();
		for (cp=contactList.begin();cp!=contactList.end();cp++) {
			contactForce = (*cp)->getDynamicContactWrench();   
			forceVec.setValue(contactForce[0],contactForce[1],contactForce[2]);
			forceVec *= FORCE_SCALE;
			//don't add large arrows because they mess up the OpenGL clipping planes
			if (forceVec.length() > 200) {
				forceVec *= 200.0 / forceVec.length();
			}
			SoArrow *arrow = new SoArrow;
			arrow->height = forceVec.length();
			arrow->cylRadius = 0.25;
			arrow->coneRadius = 0.5;
			if (arrow->height.getValue() < arrow->coneHeight.getValue()) {
				arrow->coneHeight = arrow->height.getValue() / 2.0;
			}     
			SoTransform *tran = new SoTransform;
			(*cp)->getContactFrame().toSoTransform(tran);      
			SoRotation *rot = new SoRotation;
			rot->rotation.setValue(SbRotation(SbVec3f(0,1,0),forceVec));
            
			SoSeparator *ptr = new SoSeparator;    
			ptr->addChild(tran);
			ptr->addChild(rot);
			ptr->addChild(dynForceMat);
			ptr->addChild(arrow);

			int lastChild = world->getGB(b)->getIVContactIndicators()->getNumChildren();
			if (world->dynamicsAreOn()) {
				world->getGB(b)->getIVContactIndicators()->insertChild(ptr, lastChild);
			} else {
				contactForceBlinkerVec.push_back(new SoBlinker);
				contactForceBlinkerVec.back()->addChild(ptr);
				contactForceBlinkerVec.back()->on = false;
				contactForceBlinkerVec.back()->whichChild = 0;
				world->getGB(b)->getIVContactIndicators()->insertChild(
					contactForceBlinkerVec.back(),lastChild);
			}
		}
	}
}

/*!
  Called when a contact is selected from the contact list, and causes the
  associated contact force indicator to blink.
*/
void
IVmgr::hilightObjContact(int contactNum)
{
	if ((int)contactForceBlinkerVec.size() > contactNum) {
		contactForceBlinkerVec[contactNum]->on = true;
	} else {
		DBGA("Highlight blinker " << contactNum << " requested, but only " 
			<< contactForceBlinkerVec.size() << " present.");
	}
}

/*!
  Called when a contact is deselected from the contact list, and causes the
  associated contact force indicator to stop blinking.
*/
void
IVmgr::unhilightObjContact(int contactNum)
{
	if ((int)contactForceBlinkerVec.size() > contactNum) {
		contactForceBlinkerVec[contactNum]->on = false;
		contactForceBlinkerVec[contactNum]->whichChild = 0;
	} else {
		DBGA("Unhighlight blinker " << contactNum << " requested, but only " 
			<< contactForceBlinkerVec.size() << " present.");
	}
}

/*!
  This is a callback routine that is invoked anytime a handlebox or centerball
  is manipulated by the user. It requires a pointer to the draggerInfo
  structure of the manipulated dragger.  If the center of centerball was
  changed, it simply updates the variable tracking the centerball translation.
  If a handlebox is moved or a centerball is rotated this computes the new
  body transform and calls the worldElement::moveTo routine.  After the move
  is completed or a contact prevents further motion, it asks the world to
  update all grasps, and sets the new dragger position.
*/
void
IVmgr::transRot(DraggerInfo *dInfo)
{
	//static int count = 0;
	//DBGA("Callback " << count++);
	
	SoCenterballDragger *myCenterball;
	SoHandleBoxDragger *myHandleBox;
	bool translating;

	Quaternion origRotation,desiredRotation;
	vec3 origTranslation,desiredTranslation, center,scale;
	transf newTran;

	//is it a handlebox dragger (translating) or a centerball (rotating)
	if (dInfo->dragger->isOfType(SoHandleBoxDragger::getClassTypeId())) {
		myHandleBox = (SoHandleBoxDragger *)dInfo->dragger;
		translating = true;
	} else {
		myCenterball = (SoCenterballDragger *)dInfo->dragger;
		translating = false;
	}
  
	// if this callback is due to a recentering of the centerball dragger,
	// then no actual movement is performed but hand's current translation
	// and rotation values need to be recomputed.
	if (!translating) {
		SbVec3f centerTran = myCenterball->center.getValue() - dInfo->lastCent;
		if (centerTran.length() > 1.0e-3) {
			DBGP("RECENTERING");
			DBGP("current center: " << myCenterball->center.getValue()[0] << " " <<
									myCenterball->center.getValue()[1] << " " <<
									myCenterball->center.getValue()[2]);
			DBGP("last center: " << dInfo->lastCent[0] << " " << 
									dInfo->lastCent[1] << " " << 
									dInfo->lastCent[2]);
			scale.set(dInfo->draggerTran->scaleFactor.getValue());    
			center.set(myCenterball->center.getValue());
			center*=scale[0];    
			transf recenterTran = translate_transf(center)*
								dInfo->selectedElement->getTran() * translate_transf(-center);
			dInfo->centerballTransl = recenterTran.translation();
			dInfo->lastCent = myCenterball->center.getValue();
			return;
		}
	}
	
	// disable the callback while we are moving things
	//this is no longer needed since we replaced the valueChanged callback 
	//with a motion callback
	//SbBool enabled = dInfo->dragger->enableValueChangedCallbacks(FALSE);
    
	origTranslation = dInfo->selectedElement->getTran().translation();
	origRotation = dInfo->selectedElement->getTran().rotation();

	// save the desired position or orientation and set what percent of the 
	// desired moved will be accomplished in each step
	SbVec3f temp = dInfo->lastTran;
	if (translating) {
		if ((myHandleBox->translation.getValue() - dInfo->lastTran).length()<0.00001) {
			myHandleBox->translation.setValue(dInfo->lastTran); 
			//if (enabled) myHandleBox->enableValueChangedCallbacks(TRUE);
			return;
		}
		desiredTranslation.set(myHandleBox->translation.getValue());
		scale.set(dInfo->draggerTran->scaleFactor.getValue());
		desiredTranslation[0] *= scale[0];
		desiredTranslation[1] *= scale[1];
		desiredTranslation[2] *= scale[2];
		//origRotation.set(dInfo->draggerTran->rotation.getValue());
		desiredTranslation = origRotation*desiredTranslation;

		DBGP("desired Translation: " << desiredTranslation);
		//not needed?
		//if ((desiredTranslation - origTranslation).len() == 0.0) return;
		newTran = transf(origRotation,desiredTranslation);
	} else {
		if (myCenterball->rotation.getValue().equals(dInfo->lastRot,0.00001f)) {
			myCenterball->rotation.setValue(dInfo->lastRot); 
			//if (enabled) myCenterball->enableValueChangedCallbacks(TRUE);
			return;
		}
		desiredRotation.set(myCenterball->rotation.getValue());
		scale.set(dInfo->draggerTran->scaleFactor.getValue());    
		center.set(myCenterball->center.getValue());
		center*=scale[0];    
		//this does not work if the centerball has been recentered
		//hope to fix this at some point
		newTran = translate_transf(-center) * transf(desiredRotation,vec3::ZERO) *
				  translate_transf(center) * translate_transf(dInfo->centerballTransl);
	}
	dInfo->selectedElement->moveTo(newTran,50*Contact::THRESHOLD,M_PI/36.0);
	world->updateGrasps();

	DBGP("new pos: "<<dInfo->selectedElement->getTran());
	if (translating) {
		SbVec3f newPos = (origRotation.inverse() * dInfo->selectedElement->getTran().translation()).toSbVec3f();
		newPos[0]/=scale[0];
		newPos[1]/=scale[1];
		newPos[2]/=scale[2];
		dInfo->lastTran = newPos;
		myHandleBox->translation.setValue(newPos);
	} else {
		SbRotation newRot = dInfo->selectedElement->getTran().rotation().toSbRotation();
		dInfo->lastRot = newRot;
		myCenterball->rotation.setValue(newRot);
#ifdef GRASPITDBG
		SbVec3f ax;
		float ang;
		newRot.getValue(ax,ang);
		desiredRotation.set(myCenterball->rotation.getValue());
		std::cout << "setting ball to: "<<desiredRotation<<std::endl;
#endif
	}

	//if (enabled) dInfo->dragger->enableValueChangedCallbacks(TRUE);
}

void 
IVmgr::revoluteJointClicked(DraggerInfo *dInfo)
{
  SbVec3f axis;
  float angle;
  SoRotateDiscDragger *dragger = (SoRotateDiscDragger *)dInfo->dragger;
  dragger->rotation.getValue(axis,angle);
  dInfo->lastVal = angle;

  Robot *robot = (Robot *) dInfo->selectedElement;
  robot->emitUserInteractionStart();
}

void 
IVmgr::revoluteJointFinished(DraggerInfo *dInfo)
{
  Robot *robot = (Robot *) dInfo->selectedElement;
  robot->emitUserInteractionEnd();
}

/*!
  The callback routine is invoked whenever a disc dragger controlling a
  revolute DOF is moved by the user.  It requires a pointer to the dragger's
  associated info structure.  It prevents the user from moving the dragger
  past the minimum or maximum DOF limits.  It calls the associated robot's
  moveDOFto routine to peform the move, and after it is completed or a contact
  prevents further motion it asks the world to update the grasps and sets the
  current angle of the disc dragger.
*/
void 
IVmgr::revoluteJointChanged(DraggerInfo *dInfo)
{
  SbVec3f axis;
  float angle;
  double desiredAngle;
  Robot *robot = (Robot *)dInfo->selectedElement;
  SoRotateDiscDragger *dragger = (SoRotateDiscDragger *)dInfo->dragger;
  DOF *dof = dInfo->dof;
  double *dofVals= new double[robot->getNumDOF()];
  double *stepBy = new double[robot->getNumDOF()];
  int d;


#ifdef GRASPITDBG
  printf("in jointChangedCB\n");
#endif

  SbBool enabled = dragger->enableValueChangedCallbacks(FALSE);

  dragger->rotation.getValue(axis,angle);
  
  if (fabs(angle - dInfo->lastVal) < 0.00001) {
      dragger->rotation.setValue(axis,dInfo->lastVal);
      if (enabled) dragger->enableValueChangedCallbacks(TRUE);
      return;
  }
  
//  desiredAngle = (double) (axis[2] * angle);

  double relAngle = (double)axis[2] * (angle - dInfo->lastVal);
  if (relAngle > M_PI) relAngle -= 2 * M_PI;
  if (relAngle <= -M_PI) relAngle += 2 * M_PI;
  desiredAngle = dof->getVal() + (double)(relAngle);
  dInfo->lastVal = angle;

#ifdef GRASPITDBG
  printf("axis[2]: %f angle: %f\n",axis[2],angle);
#endif

  // Check that the desired angle has not exceeded the DOF limits
  //if (desiredAngle - dof->getVal() > M_PI)
  //  desiredAngle -= 2*M_PI; 
  if (desiredAngle > dof->getMax())
    desiredAngle = dof->getMax();
  else if (desiredAngle < dof->getMin())
    desiredAngle = dof->getMin();

  robot->getDOFVals(dofVals);

#ifdef GRASPITDBG
  printf("desiredDOFVal: %.1f deg -- %15.15lf rad\n",desiredAngle * 180.0 / M_PI, desiredAngle);
#endif
  
  dofVals[dof->getDOFNum()] = desiredAngle;
  for(d=0;d<robot->getNumDOF();d++) {
//    stepBy[d]=0.0;
	  stepBy[d] = M_PI/36.0;
#if 0
    printf("dof %d, to: %lf\n",d,dofVals[d]);
#endif
  }
  stepBy[dof->getDOFNum()] = M_PI/36.0;
  
  // Perform the move
  robot->moveDOFToContacts(dofVals,stepBy,true);
  //this is just so that a potential eigen grasp dialog knows about this
  robot->emitConfigChange();
  world->updateGrasps();

#ifdef GRASPITDBG
  printf("newDOFVal: %15.15lf\n",dof->getVal());
#endif
  
  angle = (float) dof->getVal();
  //if (dof->getVal() >= 0.0 && (dof->getVal() % (2*M_PI)) < M_PI) {
  //    axis[2] = 1.0f;
  //    angle = (float)dof->getVal();
  //  }
  //  else {
  //    axis[2] = -1.0f;
  //    angle = (float) -dof->getVal();
  //  }

  angle = dof->getVal() / axis[2];
//  dInfo->lastVal = angle;
#ifdef GRASPITDBG
  printf("setting axis[2]: %f angle: %f dof val: %f\n",axis[2],angle,dof->getVal());
#endif

  dragger->rotation.setValue(axis,angle);
  
  if (enabled)
    dragger->enableValueChangedCallbacks(TRUE);

  delete [] dofVals;
  delete [] stepBy;
}

/*!
  The callback routine is invoked whenever an arrow dragger controlling a
  prismatic DOF is moved by the user.  It requires a pointer to the dragger's
  associated info structure.  It prevents the user from moving the dragger
  past the minimum or maximum DOF limits.  It calls the associated robot's
  moveDOFto routine to peform the move, and after it is completed or a contact
  prevents further motion it asks the world to update the grasps and sets the
  current translation of the arrow dragger.
*/
void
IVmgr::prismaticJointChanged(DraggerInfo *dInfo)
{
  SbVec3f transl(0,0,0);
  double desiredTransl;
  SoTranslate1Dragger *dragger = (SoTranslate1Dragger *)dInfo->dragger;
  Robot *robot = (Robot *)dInfo->selectedElement;
  DOF *dof = dInfo->dof;
  double *dofVals = new double[robot->getNumDOF()];
  double *stepBy = new double[robot->getNumDOF()];

  SbBool enabled = dragger->enableValueChangedCallbacks(FALSE);

  float scale=robot->getDOFDraggerScale(dof->getDOFNum());

  // Must multiply the dragger value by its scale
  desiredTransl = (double) (dragger->translation.getValue()[0] * scale);

  if (desiredTransl > dof->getMax()) {
    desiredTransl = dof->getMax();
  }
  else if (desiredTransl < dof->getMin()) {
    desiredTransl = dof->getMin();
  }

  robot->getDOFVals(dofVals);

#ifdef GRASPITDBG
  printf("desiredDOFVal: %le\n",desiredTransl);
#endif
  
  dofVals[dof->getDOFNum()] = desiredTransl;
  for(int d=0;d<robot->getNumDOF();d++) stepBy[d]=0.0;
  stepBy[dof->getDOFNum()] = 50*Contact::THRESHOLD;
  
  // Perform the move
  robot->moveDOFToContacts(dofVals,stepBy,true);
  //this is just so that a potential eigen grasp dialog knows about this
  robot->emitConfigChange();
  world->updateGrasps();

  transl[0] = (float) dof->getVal()/scale;
  dragger->translation.setValue(transl);

  if (enabled)
    dragger->enableValueChangedCallbacks(TRUE);

  delete [] dofVals;
  delete [] stepBy;
}

/*! 
  Creates a centerball dragger for the provided worldElement around the
  body surroundMe, and sets up the value changed callback for the dragger.
  For a free body, the worldElement and surroundMe point to the same body.
  For a robot, the worldElement points to the robot and surround me points
  to the base link of the robot.
 */
void
IVmgr::makeCenterball(WorldElement *selectedElement,Body *surroundMe)
{
	SoCenterballDragger *myCenterball = new SoCenterballDragger;
	SoSeparator *sep = new SoSeparator;
	SoTransform *draggerTran = new SoTransform;
  
	// Compute the bounding box of the surroundMe body
	SoGetBoundingBoxAction *bba = new SoGetBoundingBoxAction(myViewer->getViewportRegion());
	bba->apply(surroundMe->getIVGeomRoot());
	float maxRad = (bba->getBoundingBox().getMax() - bba->getBoundingBox().getMin()).length();
	delete bba;

	maxRad /= 2.0;
  
	draggerTran->translation = selectedElement->getTran().translation().toSbVec3f();
	draggerTran->scaleFactor.setValue(maxRad,maxRad,maxRad);
	myCenterball->rotation = selectedElement->getTran().rotation().toSbRotation();

	sep->addChild(draggerTran);
	sep->addChild(myCenterball);
	draggerRoot->addChild(sep);

	// Create the dragger info structure
	DraggerInfo *dInfo = new DraggerInfo;
	dInfo->draggerSep = sep;
	dInfo->dragger = myCenterball;
	dInfo->draggerTran = draggerTran;
	dInfo->selectedElement = selectedElement;
	dInfo->lastCent = myCenterball->center.getValue();
	DBGP("lastCent " << dInfo->lastCent[0] << " "<<dInfo->lastCent[1]<<" "<<dInfo->lastCent[2]);
	
	dInfo->centerballTransl = selectedElement->getTran().translation();
	draggerInfoList.push_back(dInfo);
	DBGP("NUM DRAGGER INFOS "<<draggerInfoList.size());

	//the value changed callback is behaving strangely so we replaced it
	//with motion callback which is a lot more stable. if you need the 
	//functionality of the valueChanged callback, beware of erratic
	//behavior where the callback is called again and again for no reason.
	//myCenterball->addValueChangedCallback(transRotCB,dInfo);  
	myCenterball->addMotionCallback(transRotCB,dInfo);  
}

/*! 
  Creates a handlebox dragger for the provided worldElement around the
  body surroundMe, and sets up the value changed callback for the dragger.
  For a free body, the worldElement and surroundMe point to the same body.
  For a robot, the worldElement points to the robot and surround me points
  to the base link of the robot.
 */
void
IVmgr::makeHandleBox(WorldElement *selectedElement,Body *surroundMe)
{
  SoSeparator *sep = new SoSeparator;
  draggerRoot->addChild(sep);

  // Elminate many parts of the handlebox that would allow body scaling etc...
  SoHandleBoxDragger *myHandleBox = new SoHandleBoxDragger;
  myHandleBox->setPart("extruder1",new SoSeparator);
  myHandleBox->setPart("extruder2",new SoSeparator);
  myHandleBox->setPart("extruder3",new SoSeparator);
  myHandleBox->setPart("extruder4",new SoSeparator);
  myHandleBox->setPart("extruder5",new SoSeparator);
  myHandleBox->setPart("extruder6",new SoSeparator);
  myHandleBox->setPart("uniform1",new SoSeparator);
  myHandleBox->setPart("uniform2",new SoSeparator);
  myHandleBox->setPart("uniform3",new SoSeparator);
  myHandleBox->setPart("uniform4",new SoSeparator);
  myHandleBox->setPart("uniform5",new SoSeparator);
  myHandleBox->setPart("uniform6",new SoSeparator);
  myHandleBox->setPart("uniform7",new SoSeparator);
  myHandleBox->setPart("uniform8",new SoSeparator);
  myHandleBox->setPart("extruder1Active",new SoSeparator);
  myHandleBox->setPart("extruder2Active",new SoSeparator);
  myHandleBox->setPart("extruder3Active",new SoSeparator);
  myHandleBox->setPart("extruder4Active",new SoSeparator);
  myHandleBox->setPart("extruder5Active",new SoSeparator);
  myHandleBox->setPart("extruder6Active",new SoSeparator);
  myHandleBox->setPart("uniform1Active",new SoSeparator);
  myHandleBox->setPart("uniform2Active",new SoSeparator);
  myHandleBox->setPart("uniform3Active",new SoSeparator);
  myHandleBox->setPart("uniform4Active",new SoSeparator);
  myHandleBox->setPart("uniform5Active",new SoSeparator);
  myHandleBox->setPart("uniform6Active",new SoSeparator);
  myHandleBox->setPart("uniform7Active",new SoSeparator);
  myHandleBox->setPart("uniform8Active",new SoSeparator);
  
  // compute the bounding box of the body  
  SoGetBoundingBoxAction *bba = new SoGetBoundingBoxAction(myViewer->getViewportRegion());
  bba->apply(surroundMe->getIVGeomRoot());
  SbVec3f bbmin,bbmax;
  bba->getBoundingBox().getBounds(bbmin,bbmax);
  delete bba;

   // make the dragger (a 2x2x2 box) slightly bigger than the bounding box.
  SbVec3f scale = ((bbmax - bbmin)/1.9f);
  double maxScale = MAX(MAX(scale[0],scale[1]),scale[2]);
  if (maxScale == 0.0) {   // not good, object has 0 volume bbox!
    DBGP( "0 volume bounding box!");
    return;
  }

  // prevent any of the sides from having a dimension much smaller than another
  for (int i=0;i<3;i++) {
    if (scale[i]/maxScale < .01) {
      scale[i] = 0.01 * maxScale;
	}
  }

  SbRotation rot = selectedElement->getTran().rotation().toSbRotation();

  SbVec3f transl =
    (selectedElement->getTran().rotation().inverse()*
     selectedElement->getTran().translation()).toSbVec3f();

  transl[0] = transl[0] / scale[0];
  transl[1] = transl[1] / scale[1];
  transl[2] = transl[2] / scale[2];

  // compute the offset from the body's frame to the dragger's center
  SbVec3f offsetTransl;
  rot.multVec((bbmax+bbmin)/2.0f,offsetTransl);

  SoTransform *draggerTran = new SoTransform;

  myHandleBox->translation= transl;
  draggerTran->scaleFactor.setValue(scale);
  draggerTran->rotation = rot;
  draggerTran->translation = offsetTransl;

  sep->addChild(draggerTran);
  sep->addChild(myHandleBox);

  DraggerInfo *dInfo = new DraggerInfo;
  dInfo->draggerSep = sep;
  dInfo->dragger = myHandleBox;
  dInfo->draggerTran = draggerTran;
  dInfo->selectedElement = selectedElement;
  draggerInfoList.push_back(dInfo);

  DBGP("Callback added");
  //the value changed callback is behaving strangely so we replaced it
  //with motion callback which is a lot more stable. if you need the 
  //functionality of the valueChanged callback, beware of erratic
  //behavior where the callback is called again and again for no reason.
  //  myHandleBox->addValueChangedCallback(transRotCB,dInfo);
  myHandleBox->addMotionCallback(transRotCB,dInfo);  
}

/*! 
  Creates the joint draggers for each DOF in the provided kinematic chain,
  and sets up the value changed callback for each dragger.  Only the first
  joint in the chain associated with a given DOF has a dragger connected to
  it.  In other words, passive joints don't have draggers.  The scale of
  the joint dragger is controlled by a parameter found in the joints section
  of the robot configuration file.
 */
void
IVmgr::makeJointDraggers(Robot *robot,KinematicChain *chain)
{
  int j,d;
  int *activeDOFs = new int[robot->getNumDOF()];
  SoSeparator *jointDraggerSep = new SoSeparator;
  DraggerInfo *dInfo;
  bool firstDragger=true;

  //jointDraggerSep->ref();
  jointDraggerSep->addChild(robot->getBase()->getIVTran());
  DBGP("make draggers; value: " << chain->getIVTran()->translation.getValue()[0]);
  jointDraggerSep->addChild(chain->getIVTran());

  for (d=0;d<robot->getNumDOF();d++)
    activeDOFs[d] = -1;

  for (j=chain->getNumJoints()-1;j>=0;j--)
    activeDOFs[chain->getJoint(j)->getDOFNum()] = j;
	
  for(d=0;d<robot->getNumDOF();d++) {
    if ((j=activeDOFs[d]) != -1) {
      SoSeparator *sep = new SoSeparator;
      dInfo = new DraggerInfo;
      dInfo->selectedElement = robot;
      dInfo->dof = robot->getDOF(d);
      
      // add the parent separator for all the joint draggers to the
      // first draggerInfo, because it includes not only the draggers
      // but also the transforms between them, and we'll delete them
      // all at once.
      if (firstDragger) {
        dInfo->draggerSep = jointDraggerSep;
        firstDragger=false;
      }
      else
        dInfo->draggerSep = NULL;
      
      //      for (l=0;l<chain->getNumLinks();l++)
      //	if (j<=chain->getLastJoint(l)) break;

      if (chain->getJoint(j)->getType() == REVOLUTE) {
        SoScale *dialSize = new SoScale;
        SoRotateDiscDragger *myDisc = new SoRotateDiscDragger;
        
        float scale = robot->getDOFDraggerScale(d);
        dialSize->scaleFactor.setValue(SbVec3f(scale,scale,scale));
       	SoTranslation *dTrans = new SoTranslation;
        dTrans->translation.setValue(0,0,chain->getJoint(j)->getDH()->getD());
        
        //sep->addChild(dTrans);
        sep->addChild(dialSize);
        sep->addChild(myDisc);
	
        myDisc->rotation.setValue(SbVec3f(0.0,0.0,1.0),(float) robot->getDOF(d)->getVal());
        //dInfo->lastVal = (float) robot->getDOF(d)->getVal();
        myDisc->addStartCallback(revoluteJointClickedCB,dInfo);
        myDisc->addValueChangedCallback(revoluteJointChangedCB,dInfo);
        myDisc->addFinishCallback(revoluteJointFinishedCB,dInfo);
        dInfo->dragger = myDisc;
      }
      else { // prismatic
        SoTransform *arrowTran = new SoTransform;
        SoTranslate1Dragger *myArrow = new SoTranslate1Dragger;
        SoBaseColor *arrowBC = new SoBaseColor;
        float scale = robot->getDOFDraggerScale(d);
        arrowBC->rgb.setValue(1,1,1);
        arrowTran->scaleFactor.setValue(SbVec3f(scale,scale,scale));
        arrowTran->rotation.setValue(SbVec3f(0,1,0),(float)-M_PI/2.0f);
        arrowTran->translation.setValue(SbVec3f(0,-scale,0));
        sep->addChild(arrowBC);
        sep->addChild(arrowTran);
        sep->addChild(myArrow);
	
        myArrow->translation.setValue(SbVec3f((float)robot->getDOF(d)->getVal()/scale,0,0));
        dInfo->lastVal = (float) robot->getDOF(d)->getVal()/scale;
        myArrow->addValueChangedCallback(prismaticJointChangedCB,dInfo);
        dInfo->dragger = myArrow;
      }
      // now it will update its IVTran whenever it moves
      chain->getJoint(j)->setDraggerAttached(true);
      
      jointDraggerSep->addChild(sep);
      jointDraggerSep->addChild(chain->getJoint(j)->getIVTran());
      draggerInfoList.push_back(dInfo);
    }
  }
  draggerRoot->addChild(jointDraggerSep);

#ifdef GRASPITDBG
  std::cout << "NUM DRAGGER INFOS "<<draggerInfoList.size()<<std::endl;
#endif
  delete activeDOFs;
}

/*!
  Creates a white wireframe copy of the model sub-tree passed to it in
  elementRoot, and adds it to the wireframe root.  This is used to provide
  a visual indication when a body has been selected by the user.
 */
void
IVmgr::drawWireFrame(SoSeparator *elementRoot)
{
  SoLightModel *lm = new SoLightModel;
  lm->model = SoLightModel::BASE_COLOR;

  SoDrawStyle *ds = new SoDrawStyle;
  ds->style = SoDrawStyle::LINES;
  ds->setOverride(TRUE);

  SoBaseColor *bc = new SoBaseColor;
  bc->rgb.setValue(1,1,1);
  bc->setOverride(TRUE);

  SoSeparator *sep = new SoSeparator;
  sep->addChild(lm);
  sep->addChild(ds);
  sep->addChild(bc);
  sep->addChild(elementRoot);

  wireFrameRoot->addChild(sep);
}

/*!
  This callback routine is invoked when a user clicks within the scene and
  controls what Inventor body is picked.  If a dragger is clicked, a empty
  path is returned because we don't want the dragger to be selected in the
  Inventor sense.  If the translate or rotate tool is currently being used
  and a link of a kinematic chain is clicked, pick the whole chain.
  If the base of a robot is clicked, pick the whole robot.  If the current
  tool is the select tool, and a link is clicked that has already been
  selected, pick the whole robot.  Otherwise pick the body that was clicked.
*/
SoPath *
IVmgr::pickFilter(const SoPickedPoint *pick)
{
  int i,l,r,f,b;
  SoPath *p = pick->getPath();
  Robot *robot;

  if (p->getTail()->isOfType(SoDragger::getClassTypeId())||
      p->getTail()->isOfType(SoTransformManip::getClassTypeId()))
    {
	  DBGP("dragger or manip picked");
      SoPath *newP = new SoPath(sceneRoot);
      return newP;
    }
  
  for (i=p->getLength() - 1; i>=0; i--) {
    SoNode *n = p->getNode(i);
    for (r=0;r<world->getNumRobots(); r++) {
      robot = world->getRobot(r);
      
      if (currTool != SELECT_TOOL) {
		for (f=0;f<robot->getNumChains();f++)
		  if (n == robot->getChain(f)->getIVRoot())
			return p->copy(0,i+1);                  // select the chain

		if (n == robot->getBase()->getIVRoot())
		return p->copy(0,i);                      // select the whole robot

		if (robot->inherits("HumanHand"))
		{
			for (f=0; f<((HumanHand*)robot)->getNumTendons(); f++)
				if ( n == ((HumanHand*)robot)->getTendon(f)->getIVRoot() )
					return p->copy(0,i+1);
		}

	  }
      else {
		if (n == robot->getBase()->getIVRoot() && world->isSelected(robot->getBase())) 
			return p->copy(0,i);                      // select the whole robot
  
		for (f=0;f<robot->getNumChains();f++)
		  for (l=0;l<robot->getChain(f)->getNumLinks();l++)
			 if (n == robot->getChain(f)->getLink(l)->getIVRoot()&&	world->isSelected(robot->getChain(f)->getLink(l)))
				return p->copy(0,i-1);                // select the whole robot

		if (robot->inherits("HumanHand"))
		{
			for (f=0; f<((HumanHand*)robot)->getNumTendons(); f++)
				if ( n == ((HumanHand*)robot)->getTendon(f)->getIVRoot() )
					return p->copy(0,i+1);
		}

	  }
    }
  }
  
  for (i=p->getLength() - 1; i>=0; i--) {
    SoNode *n = p->getNode(i);
    for(b=0;b<world->getNumBodies();b++)      
      if (n == world->getBody(b)->getIVRoot()) {
	  #ifdef GRAPSITDBG
		printf("pickfilter: body %s picked\n",world->getBody(b)->getName());
	  #endif
	  return p->copy(0,i+1);                      // select body
      }
  }
  
  return NULL;
}

/*!
  This callback routine is invoked whenever an Inventor selection occurs.
  When this routine is called because of a user click, the path sent to this
  routine is determined by the pickFilter.  The action that occurs upon
  selection depends on the current tool selected.  If a kinematic chain is
  selected, and the rotate or translate tool is selected, joint draggers are
  created.  Otherwise, if the rotate tool is being used, then a centerball is
  created around the chosen body.  If the translate tool is being used, then
  a handlebox is created around the chosen body.  If the select tool is being
  used, the chosen body is selected within GraspIt! and a wireframe version
  of the body is drawn to indicate the selection.
*/
void 
IVmgr::handleSelection(SoPath *p)
{
  int r,b,f;
  bool selectionFound=false;
  Robot *robot;

#ifdef GRASPITDBG	  
  printf("in selectionCB: \n");
  fprintf(stderr,"in selectionCB: \n");
  
  fprintf(stderr,"*************NUM SELECTED %d*****************\n",selectionRoot->getNumSelected());
#endif

  if (p->getTail()->isOfType(SoDragger::getClassTypeId())) {
#ifdef GRASPITDBG
    printf("selection path tail is dragger\n");
#endif
    return;
  }
  else if (p->getTail() == sceneRoot) {
#ifdef GRASPITDBG
    printf("selection path tail is sceneRoot\n");
#endif
    return;
  }
  else {
    for (r=0;r<world->getNumRobots();r++) {
      robot = world->getRobot(r);      
      if (p->getTail() == robot->getIVRoot()) {
#ifdef GRASPITDBG
        printf("robot selected\n");
#endif
        selectionFound = true;
        if (currTool == ROTATE_TOOL)
          makeCenterball(robot,robot->getBase());
        else if (currTool == TRANSLATE_TOOL)
          makeHandleBox(robot,robot->getBase());
        else if (currTool == SELECT_TOOL) {
          world->selectElement(robot);
          drawWireFrame(robot->getIVRoot());
        }
      }
      else {
        for (f=0;f<robot->getNumChains();f++)
          if (p->getTail() == robot->getChain(f)->getIVRoot()) {
            selectionFound = true;
            makeJointDraggers(robot,robot->getChain(f));
          }
        if ( robot->inherits("HumanHand") )
        {
          for (f=0; f< ((HumanHand*)robot)->getNumTendons(); f++)
            if (p->getTail() == ((HumanHand*)robot)->getTendon(f)->getIVRoot())
            {
              selectionFound = true;
              world->selectTendon( ((HumanHand*)robot)->getTendon(f));
            }
        }
      }
    }
    if (!selectionFound) {
      // check if one of the objects was selected
      for (b=0;b<world->getNumBodies();b++) {
	if (p->getTail() == world->getBody(b)->getIVRoot()) {
          //#ifdef GRASPITDBG
	  printf("body %s selected\n",world->getBody(b)->getName().latin1());
          //#endif
	  selectionFound = true;
	  if (currTool == ROTATE_TOOL)
	    makeCenterball(world->getBody(b),world->getBody(b));
	  else if (currTool == TRANSLATE_TOOL)
	    makeHandleBox(world->getBody(b),world->getBody(b));
	  else if (currTool == SELECT_TOOL) {
	    world->selectElement(world->getBody(b));
	    drawWireFrame(world->getBody(b)->getIVRoot());
	  }
	  break;
	}
      }
    }    
  }
}

/*!
  This callback routine is invoked whenever an Inventor deselection occurs.
  If the select tool is being used this deselects the element within GraspIt!
  and deletes the wireframe.  Otherwise this removes the draggers, from the
  deselected body or robot.
*/
void
IVmgr::handleDeselection(SoPath *p)
{
  int r,b,f,j;
  Robot *robot;
  WorldElement *deselectedElement=NULL;
  std::list<DraggerInfo *>::iterator dp;

#ifdef GRASPITDBG	  
  printf("in deselectionCB: \n");
#endif


#ifdef GRASPITDBG	  
  if (p->getTail()->isOfType(SoDragger::getClassTypeId()))
    printf("deselecting dragger\n");
#endif
  
  for (r=0;r<world->getNumRobots() && !deselectedElement;r++) {
    robot = world->getRobot(r);

    // check if the base was deselected
    if (p->getTail() == robot->getIVRoot()) {
      deselectedElement = robot;
    } else {
      // check if a chain was deselected
	  for (f=0;f<robot->getNumChains() && !deselectedElement;f++) {
		if (p->getTail() == robot->getChain(f)->getIVRoot()) {
			DBGP("deselecting chain " << f);
			for (j=0;j<robot->getChain(f)->getNumJoints();j++)
				robot->getChain(f)->getJoint(j)->setDraggerAttached(false);
			deselectedElement = robot;
		}
	  }
	  if ( robot->inherits("HumanHand") ){
			for (f=0; f< ((HumanHand*)robot)->getNumTendons(); f++)
				if (p->getTail() == ((HumanHand*)robot)->getTendon(f)->getIVRoot())	{
					deselectedElement = robot;
					//let's keep the tendon selected so we can play with robot joints 
					//and see how it affeccts the tendon
					//world->deselectTendon();
				}
		}

	}
  }

  for (b=0;b<world->getNumBodies() && !deselectedElement;b++)
    if (p->getTail() == world->getBody(b)->getIVRoot()) {
      DBGP("deselecting body " << world->getBody(b)->getName().latin1());
      deselectedElement = world->getBody(b);
    }

  if (deselectedElement) {
    if (currTool == SELECT_TOOL) {
      world->deselectElement(deselectedElement);
      for (b=0;b<wireFrameRoot->getNumChildren();b++) {
		SoSeparator *wfSep = (SoSeparator *)wireFrameRoot->getChild(b);
		if (wfSep->getChild(wfSep->getNumChildren()-1) ==  deselectedElement->getIVRoot()) {
			wireFrameRoot->removeChild(b);
			break;
		}
      }
	}
    else {
      dp=draggerInfoList.begin();
      while (dp!=draggerInfoList.end()) {
		if ((*dp)->selectedElement == deselectedElement) {
		  if ((*dp)->draggerSep) {
			draggerRoot->removeChild((*dp)->draggerSep);
		  }
		  delete *dp;
		  dp = draggerInfoList.erase(dp);
		  DBGP("removing dragger info");
		} else {
		  dp++;
		}
	  }
    }
  }
}

/*!
  If the select tool is currently being used and the user hits the delete key,
  then all currently selected bodies will be deleted.  This routine does not
  allow links of robots to be deleted, because of the change it would
  require in the robot structure.  Sometime in the future we'll deal with that.
  However, if the whole robot is selected, it will be deleted.
*/
void
IVmgr::deleteSelections()
{
  int i,r,b;

  // If a whole robot is selected, delete it
  for (i=selectionRoot->getNumSelected()-1;i>=0;i--) {
    for (r=0;r<world->getNumRobots();r++)
      if (selectionRoot->getPath(i)->getTail() ==
	  world->getRobot(r)->getIVRoot()) {
	selectionRoot->deselect(i);
	world->destroyElement(world->getRobot(r));
	break;
      }
  }

  // If a body is selected, first make sure it isn't a link before deleting it.
  // Later, we'll want to be able to delete links, but for now it's too
  // complicated, and would mean the robot structure would have to change
  // dynamically
  for (i=selectionRoot->getNumSelected()-1;i>=0;i--) {
    for (b=0;b<world->getNumBodies();b++)
      if (selectionRoot->getPath(i)->getTail() ==
	  world->getBody(b)->getIVRoot()) {
	if (!world->getBody(b)->inherits("Link")) {
	  selectionRoot->deselect(i);
	  world->destroyElement(world->getBody(b));
	}
	break;
      }
  }
}

void
IVmgr::deselectBody(Body *b)
{
	int i;
	for (i=selectionRoot->getNumSelected()-1;i>=0;i--) {
		if (selectionRoot->getPath(i)->getTail() == b->getIVRoot()) {
			if (!b->inherits("Link")) {
				selectionRoot->deselect(i);
			}
		break;
		}
	}
}
/*!
  Given a filename with an appropriate image format file extension, this
  will render the current scene to an image file.  It currently uses a white
  background and performs anti-aliasing by blending the images from 5 slightly
  different camera angles.  One camera headlight is used for lighting.
 */
void 
IVmgr::saveImage(QString filename)
{
  SoQtRenderArea *renderArea;
  SoNode *sg;                // scene graph
  const SbColor white(1, 1, 1);
  //  const SbColor black(0,0,0);
  SoGLRenderAction *glRend;
  SoOffscreenRenderer *myRenderer;

  renderArea = myViewer;
  sg = sceneRoot;

  glRend = new SoGLRenderAction(renderArea->getViewportRegion());
  glRend->setSmoothing(TRUE);
  glRend->setNumPasses(5);
  glRend->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND);
  
  myRenderer = new SoOffscreenRenderer(glRend);
  myRenderer->setBackgroundColor(white);

#ifdef GRASPITDBG
  if (myRenderer->isWriteSupported("jpg"))
	std::cout << " supports jpg" << std::endl;
  else
	std::cout << "no jpg support" << std::endl;
#endif

  int numtypes = myRenderer->getNumWriteFiletypes();
  SbList<SbName> extList;
  SbString fullname;
  SbString desc;
  for (int i=0;i<numtypes;i++) {
    myRenderer ->getWriteFiletypeInfo(i,extList,fullname,desc);
#ifdef GRASPITDBG
    std::cout <<std::endl;
    for (int j=0;j<extList.getLength();j++)
      std::cout << extList[j].getString() <<" ";
    std::cout <<std::endl<<fullname.getString()<<std::endl<<desc.getString()<<std::endl;

#endif
  }
  

  SoSeparator *renderRoot = new SoSeparator;
  renderRoot->ref();
  renderRoot->addChild(myViewer->getCamera());
  SoTransformSeparator *lightSep = new SoTransformSeparator;
  SoRotation *lightDir = new SoRotation;
  lightDir->rotation.connectFrom(&myViewer->getCamera()->orientation);
  lightSep->addChild(lightDir);
  lightSep->addChild(myViewer->getHeadlight());
  
  renderRoot->addChild(lightSep);  
  renderRoot->addChild(sg);
  
  myRenderer->render(renderRoot);
  
  SbBool result;
  result = myRenderer->writeToFile(SbString(filename.latin1()),
				   SbName(filename.section('.',-1)));

#ifdef GRASPITDBG
  if (result)
    fprintf(stderr,"saved\n");
  else
    fprintf(stderr,"not saved\n");
#endif
  
  renderRoot->unref();
  delete myRenderer;
}

/*!
  Given a base filename this will save an image of the scene after every
  completed step of the dynamics.  The filename should have a %1 in it, which
  will be replaced by the simulation time in seconds with a precision of 4
  places after the decimal point.  The fileStr should also have a valid image
  format file extension such as .jpg.
 */
void
IVmgr::saveImageSequence(const char *fileStr)
{
  imgSeqStr = fileStr;
  imgSeqCounter = 1;
  connect(world,SIGNAL(dynamicStepTaken()),this,SLOT(saveNextImage()));

}

/*!
  This calls saveImage after substituting the current world simulation time
  into the filename.
*/
void
IVmgr::saveNextImage()
{  
  saveImage(QString(imgSeqStr).
	    arg(world->getWorldTime(),0,'f',4));
}

/*!
  Given a filename, this will save the current camera position after each
  completed step of the dynamics.  This allows the user to move the camera
  during a dynamics simulation, and have each position saved so that when the
  simulation is performed again to make an image sequence, the camera will
  follow the saved trajectory.
*/
int
IVmgr::saveCameraPositions(const char *filename)
{
  if (!(camerafp=fopen(filename,"w"))) return FAILURE;
  connect(world,SIGNAL(dynamicStepTaken()),this,SLOT(saveCameraPos()));
  return SUCCESS;
}

/*!
  Given a filename to read camera positions from, this will set the camera
  position after each completed step of the dynamics.  This is useful when
  saving an image sequence to a file, because due to delays, the camera
  cannot easily be controlled directly.
*/
int
IVmgr::useSavedCameraPositions(const char *filename)
{
  if (!(camerafp=fopen(filename,"r"))) return FAILURE;
  connect(world,SIGNAL(dynamicStepTaken()),this,SLOT(restoreCameraPos()));
  return SUCCESS;
}

/*!
  Saves the current camera position as a line in the currently open camera
  position file.
*/
void
IVmgr::saveCameraPos()
{
  float x,y,z;
  float q1,q2,q3,q4;
  
  myViewer->getCamera()->position.getValue().getValue(x,y,z);
  myViewer->getCamera()->orientation.getValue().getValue(q1,q2,q3,q4);
  fprintf(camerafp,"%f %f %f %f %f %f %f\n",x,y,z,q1,q2,q3,q4);  
}

/*!
  Reads a camera position from the currently open camera position file and
  sets the viewer camera to that position.
*/
void
IVmgr::restoreCameraPos()
{
  float x,y,z;
  float q1,q2,q3,q4;
 
  if(fscanf(camerafp,"%f %f %f %f %f %f %f\n",&x,&y,&z,&q1,&q2,&q3,&q4) <= 0) {
    DBGA("restoreCameraPos - Failed to read camera pose");
    return;
  }  
  myViewer->getCamera()->position.setValue(x,y,z);
  myViewer->getCamera()->orientation.setValue(q1,q2,q3,q4);
}

void 
IVmgr::setCamera(double px, double py, double pz, double q1, double q2, double q3, double q4, double fd)
{
  myViewer->getCamera()->position.setValue(px,py,pz);
  myViewer->getCamera()->orientation.setValue(q1,q2,q3,q4);
  myViewer->getCamera()->focalDistance.setValue(fd);
}

void 
IVmgr::getCamera(float &px, float &py, float &pz, float &q1, float &q2, float &q3, float &q4, float &fd)
{
  myViewer->getCamera()->position.getValue().getValue(px,py,pz);
  myViewer->getCamera()->orientation.getValue().getValue(q1,q2,q3,q4);
  fd = myViewer->getCamera()->focalDistance.getValue();
}

void 
IVmgr::setCameraTransf(transf tr)
{
	const vec3 t = tr.translation();
	const Quaternion q = tr.rotation();
	myViewer->getCamera()->position.setValue( t.x(), t.y(), t.z() );
	myViewer->getCamera()->orientation.setValue( q.x, q.y, q.z, q.w );
}

transf
IVmgr::getCameraTransf()
{
	transf tr;
	float px, py, pz, qx, qy, qz, qw;
	myViewer->getCamera()->position.getValue().getValue(px,py,pz);
	myViewer->getCamera()->orientation.getValue().getValue(qx,qy,qz,qw);
	Quaternion q = Quaternion(qw,qx,qy,qz);
	tr.set( q, vec3(px,py,pz) );
	return tr;
}

/*!
  This callback routine is invoked when the user presses a key while the
  arrow (not the hand) viewer tool is selected.  The space bar toggles the
  state of the dynamics.  The delete key deletes currently selected bodies.
  The G key starts an autograps of the current hand.
*/
void
IVmgr::keyPressed(SoEventCallback *eventCB)
{
  const SoEvent *event = eventCB->getEvent();

  if (SO_KEY_RELEASE_EVENT(event,SPACE)) {
    if (world->dynamicsAreOn()) world->turnOffDynamics();
    else world->turnOnDynamics();
  }

  //can't use simple macro with DELETE because win32 headers define DELETE
  if (SoKeyboardEvent::isKeyReleaseEvent(event, SoKeyboardEvent::KEY_DELETE)) {
    if (currTool == SELECT_TOOL)
      ivmgr->deleteSelections();
  }

  if (SO_KEY_RELEASE_EVENT(event,G)) {
    if (world->getCurrentHand()) {
	  fprintf(stderr,"Autograsp!\n");
	  world->getCurrentHand()->approachToContact(30);
      world->getCurrentHand()->autoGrasp(true);
      world->updateGrasps();
    }
  }

  if (SO_KEY_RELEASE_EVENT(event,R)) {
	  world->getCurrentHand()->restoreState();
  }

  if (SO_KEY_RELEASE_EVENT(event,C)) {
	  Hand *h = world->getCurrentHand();
	  Hand *clone = new Hand(world, "Hand clone");
	  clone->cloneFrom(h);
	  clone->setRenderGeometry(false);
	  clone->showVirtualContacts(false);
	  world->addRobot(clone);
	  world->toggleCollisions(false, h, clone);
	  clone->setTran( h->getTran() );
  }

  if (SO_KEY_RELEASE_EVENT(event,S)) {
          world->getCurrentHand()->saveState();
	  /*
	  if (myViewer->isStereoOn())
		  graspItGUI->getMainWindow()->stereoOff();
	  else
		  graspItGUI->getMainWindow()->stereoOn();
		  */
  }

  if (SoKeyboardEvent::isKeyReleaseEvent(event,SoKeyboardEvent::PAD_ADD)) {
	  if (myViewer->isStereoOn()) {
		myViewer->mFocalPlane += 50.0;
		myViewer->setStereo(true);
	  }
  }
  if (SoKeyboardEvent::isKeyReleaseEvent(event,SoKeyboardEvent::PAD_SUBTRACT)) {
	  if (myViewer->isStereoOn()) {
		  myViewer->mFocalPlane -= 50.0;
		  myViewer->setStereo(true);
	  }
  }
  
  //static bool left = true;  

  static SoSeparator *indicators = NULL;
  if (SO_KEY_RELEASE_EVENT(event,V)) {
	  fprintf(stderr,"Distance test\n");
	  if (!indicators) {
		  indicators = new SoSeparator();
		  world->getIVRoot()->addChild(indicators);
	  }
	  indicators->removeAllChildren();
	  if (world->getNumGB() < 2) return;
	  Body *b1 = world->getGB(0);
	  Body *b2 = world->getGB(1);
	  position p1, p2;
	  world->getDist(b1, b2, p1, p2);
	  p1 = p1 * b1->getTran();
	  p2 = p2 * b2->getTran();

	  //vec3 v3 = world->pointDistanceToBody(p1, b2);
	  //v3 = (p1 - position::ORIGIN) + v3;
	  //p2 = position( v3.x(), v3.y(), v3.z());

	  SoSphere *sphere = new SoSphere();
	  sphere->radius = 2;

	  SoSeparator *sep1 = new SoSeparator();
	  SoTransform *tran1 = new SoTransform();
	  tran1->translation.setValue(p1.x(), p1.y(), p1.z());
	  sep1->addChild(tran1);
	  sep1->addChild(sphere);
	  indicators->addChild(sep1);

	  SoSeparator *sep2 = new SoSeparator();
	  SoTransform *tran2 = new SoTransform();
	  tran2->translation.setValue(p2.x(), p2.y(), p2.z());
	  sep2->addChild(tran2);
	  sep2->addChild(sphere);
	  indicators->addChild(sep2);
  }

}

// static callback routines
void
IVmgr::keyPressedCB(void *,SoEventCallback *eventCB)
{
  ivmgr->keyPressed(eventCB);
}

void
IVmgr::transRotCB(void *dInfo,SoDragger *dragger)
{ 
  ((DraggerInfo *)dInfo)->dragger = dragger;  //to avoid compiler warning
  ivmgr->transRot((DraggerInfo *)dInfo);
}

void
IVmgr::revoluteJointChangedCB(void *dInfo,SoDragger *dragger)
{
  ((DraggerInfo *)dInfo)->dragger = dragger;  //to avoid compiler warning
  ivmgr->revoluteJointChanged((DraggerInfo *)dInfo);
}

void
IVmgr::revoluteJointClickedCB(void *dInfo,SoDragger *dragger)
{
  ((DraggerInfo *)dInfo)->dragger = dragger;  //to avoid compiler warning
  ivmgr->revoluteJointClicked((DraggerInfo *)dInfo);
}

void
IVmgr::revoluteJointFinishedCB(void *dInfo,SoDragger *dragger)
{
  ((DraggerInfo *)dInfo)->dragger = dragger;  //to avoid compiler warning
  ivmgr->revoluteJointFinished((DraggerInfo *)dInfo);
}

void
IVmgr::prismaticJointChangedCB(void *dInfo,SoDragger *dragger)
{
  ((DraggerInfo *)dInfo)->dragger = dragger;  //to avoid compiler warning
  ivmgr->prismaticJointChanged((DraggerInfo *)dInfo);
}

void
IVmgr::shiftOrCtrlDownCB(void *,SoEventCallback *eventCB)
{
  const SoEvent *event = eventCB->getEvent();
  if (SO_MOUSE_RELEASE_EVENT(event,BUTTON1))
      ivmgr->setCtrlDown(event->wasCtrlDown());
  if (SO_MOUSE_PRESS_EVENT(event,BUTTON1)) 
      ivmgr->setShiftDown(event->wasShiftDown());
}

SoPath*
IVmgr::pickFilterCB(void *,const SoPickedPoint *pick)
{
  return ivmgr->pickFilter(pick);
}

void
IVmgr::selectionCB(void *,SoPath *path)
{
  ivmgr->handleSelection(path);
}

void
IVmgr::deselectionCB(void *,SoPath *path)
{
  ivmgr->handleDeselection(path);
}

void IVmgr::setStereo(bool s)
{
	myViewer->setStereo(s);
}

void IVmgr::flipStereo()
{
}
