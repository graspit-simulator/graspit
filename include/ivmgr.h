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
// Author(s):  Andrew T. Miller 
//
// $Id: ivmgr.h,v 1.20 2009/06/16 19:29:41 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the IVmgr class which handles 3D user interaction.
 */
#ifndef IVMGR_HXX
#include <Inventor/SbBasic.h>
#include <Inventor/SbLinear.h>
#include <list>
#include <vector>
#include <qstring.h>
#include <qwidget.h>
#include "material.h"

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

struct VCReportType;
class SbVec3f;
class transf;
class position;

class QWidget;
class SoQtExaminerViewer;
class SoBlinker;
class SoCenterballDragger;
class SoHandleBoxDragger;
class SoTranslate1Dragger;
class SoRotateDiscDragger;
class SoDragger;
class SoMaterial;
class SoPath;
class SoPickedPoint;
class SoScale;
class SoSelection;
class SoSeparator;
class SoText2;
class SoTransform;
class SoEventCallback;
class SoSensor;
class SoNodeSensor;
class SoQtRenderArea;
class World;
class WorldElement;
class Robot;
class HumanHand;
class KinematicChain;
class DOF;
class GraspableBody;
class Body;
class Finger;
class Grasp;
class GWS;
class GWSprojection;
class QualityMeasure;
struct DraggerInfo;

namespace db_planner {
	class DatabaseManager;
}

#define HANDS_DIR "../../hands/"
#define OBJECTS_DIR "../../objects/"
#define MAX_POLYTOPES 15
#define MAX_COLLISIONS 5

typedef double col_Mat4[4][4];

//! Adds a couple of convenience function for better stereo viewing
/*! The native SoQt stereo mode has a couple of annoying problems setting
	inter-eye distance. This provides an easy way to switch between normal
	and stereo viewing, asks to use the quad buffer and takes care of the
	camera stereo adjustment. 

	For normal (non-stereo) rendering, you can ignore its existance.
*/
class StereoViewer : public SoQtExaminerViewer {
protected:
	bool stereoOn;
	void computeSeekFinalOrientation();
public:
	void setStereo(bool s);
	bool isStereoOn(){return stereoOn;}
	StereoViewer(QWidget *parent);
	float mFocalPlane;
};

enum ToolType {TRANSLATE_TOOL,ROTATE_TOOL,SELECT_TOOL};

//! Handles 3D interactions with the world
/*!
  There is one instance of the Inventor Manager within the application.  It
  handles all 3D user interaction with the main world.  An examiner viewer
  provides the user with a way to manipulate a virtual camera to view the
  simulation world.  The user can also use the mouse to select bodies, or
  create draggers that allow manipulation of elements within the world.
  This class also handles the display of contact forces and some other
  indicators, and can render and save an image of the current scene.
 */
class IVmgr : public QWidget {
  Q_OBJECT

 private:
  //! Global ivmgr pointer for use with static callback routines.  There is only one ivmgr.    
  static IVmgr *ivmgr;

  //! Points to the main world associated with this iteraction manager
  World *world;

  //! File pointer used to read or record camera positions when making a movie
  FILE *camerafp;

  //! Base filename fir recording an image sequence 
  const char *imgSeqStr;

  //! Current frame counter used in recording an image sequence
  int imgSeqCounter;

  //! Current interaction tool selected (translate, rotate, or select)
  ToolType currTool;

  //! A flag indicating whether the control key is down during a mouse click
  SbBool CtrlDown;

  //! A flag indicating whether the shift key is down during a mouse click
  SbBool ShiftDown;

  //! Pointers to various structures used for visual feedback.
  SoSeparator *pointers;

  //! An array of pointers to blinker nodes that correspond to individual contact force indicators
  std::vector<SoBlinker*> contactForceBlinkerVec;

  //! A pointer to the examiner viewer which inventor component facilitating user interaction
  //SoQtExaminerViewer *myViewer;
  StereoViewer *myViewer;

  //! Pointer to the root of the entire Inventor scene graph
  SoSeparator *sceneRoot;

  //! Pointer to the sub-tree of selectable Inventor objects
  SoSelection *selectionRoot;

  //! Pointer to the sub-tree of 
  SoSeparator *draggerRoot;

  //! Pointer to sub-tree containing wireframe versions of bodies when they are selected
  SoSeparator *wireFrameRoot;

  //! Pointer to an empty separator
  SoSeparator *junk;

  //! Pointer to the material node controlling the color of dynamic force indicatores
  SoMaterial *dynForceMat;

  //! The main and only interface for the CGDB; all interaction with the CGDB should go through this.
  db_planner::DatabaseManager *mDBMgr;

  void setupPointers();
  void transRot(DraggerInfo *dInfo);
  void revoluteJointChanged(DraggerInfo *dInfo);
  void revoluteJointFinished(DraggerInfo *dInfo);
  void revoluteJointClicked(DraggerInfo *dInfo);  
  void prismaticJointChanged(DraggerInfo *dInfo);
  void makeCenterball(WorldElement *selectedElement,Body *surroundMe);
  void makeHandleBox(WorldElement *selectedElement,Body *surroundMe);
  void makeJointDraggers(Robot *robot,KinematicChain *chain);
  void drawWireFrame(SoSeparator *elementRoot);
  SoPath *pickFilter(const SoPickedPoint *pick);
  void handleSelection(SoPath* p);
  void handleDeselection(SoPath* p);  
  void deleteSelections();
  void setCtrlDown(SbBool status) {CtrlDown = status;}
  void setShiftDown(SbBool status) {ShiftDown = status;}
  void spaceBar();
  void keyPressed(SoEventCallback *eventCB);

  //! Draws a wrench indicator on a body
  void drawBodyWrench(GraspableBody *body, const double *wrench);

  //! static callback routine for when a body dragger is moved
  static void transRotCB(void *dInfo,SoDragger *dragger);

  //! static callback routine for when a disc dragger is moved
  static void revoluteJointChangedCB(void *dInfo,SoDragger *dragger);
  
  //! static callback routine for when a disc dragger is clicked
  static void revoluteJointClickedCB(void *dInfo,SoDragger *dragger);

  //! static callback routine for when a disc dragger is released
  static void revoluteJointFinishedCB(void *dInfo,SoDragger *dragger);

  //! static callback routine for when an arrow dragger is moved
  static void prismaticJointChangedCB(void *dInfo,SoDragger *dragger);

  //! static callback routine for mouse events
  static void shiftOrCtrlDownCB(void *,SoEventCallback *eventCB);

  //! static callback routine for keyboard events
  static void keyPressedCB(void *,SoEventCallback *eventCB);

  //! static callback routine for selections
  static void selectionCB(void *,SoPath *p);

  //! static callback routine for deselections
  static void deselectionCB(void *,SoPath *p);

  //! static callback routine for pick events (before selections or deselections)
  static SoPath *pickFilterCB(void *,const SoPickedPoint *pick);

public slots:
  void drawDynamicForces();
  void drawWorstCaseWrenches();
  void drawUnbalancedForces();
  void saveNextImage();
  void saveCameraPos();
  void restoreCameraPos();

public:
  IVmgr(QWidget *parent=0,const char *name=0,Qt::WFlags f=0);
  ~IVmgr();

  void deselectBody(Body *b);

  /*! 
    Returns a pointer to the main World that the user interacts with through
    this manager.
   */
  World *getWorld() const {return world;}

  /*!
    Returns a pointer to the Inventor examiner viewer.
  */
  StereoViewer *getViewer() const {return myViewer;}  

  /*!
    Returns the Inventor sub-tree that holds the pointer shapes (arrow and
    torque pointer) read in during start up.
  */
  SoSeparator *getPointers() const {return pointers;}

  void setTool(ToolType newTool);
  void emptyWorld();
  void hilightObjContact(int contactNum);
  void unhilightObjContact(int contactNum);

  void saveImageSequence(const char *fileStr);
  int saveCameraPositions(const char *filename);
  int useSavedCameraPositions(const char *filename);
  //! Sets the camera position, orientaion and focal distance
  void setCamera(double px, double py, double pz, double q1, double q2, double q3, double q4, double fd);
  //! Gets the camera position, orientaion and focal distance
  void getCamera(float &px, float &py, float &pz, float &q1, float &q2, float &q3, float &q4, float &fd);
  void setCameraTransf(transf tr);
  transf getCameraTransf();

  void saveImage(QString filename);
  void beginMainLoop();

  void setStereo(bool s);
  //! Not implemented
  void flipStereo();

  //! Get the main database manager, when CGDB support is enabled
  db_planner::DatabaseManager* getDBMgr(){return mDBMgr;}
  //! Set the main database manager. Should only be called by the DB connection dialog
#ifdef CGDB_ENABLED
  void setDBMgr(db_planner::DatabaseManager *mgr){mDBMgr = mgr;}
#else
  void setDBMgr(db_planner::DatabaseManager*){}
#endif
  void setStereoWindow(QWidget *parent);
};
#define IVMGR_HXX
#endif
