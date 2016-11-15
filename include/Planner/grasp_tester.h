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
// $Id: grasp_tester.h,v 1.5 2009/03/25 22:10:24 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_tester.h                                           */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the grasp_tester class (part of grasp planner)
*/

#ifndef GRASP_TESTER_H
#define GRASP_TESTER_H

/* Minimum quality to accept grasp as stable. */
#define QUALITY_MIN_THRESHOLD 0.0

/* When a grasp is initially unstable, grasp_tester tries to
   step back from the object several times and determines the
   grasp quality there. This affects strongly the computing time. */
#define MAX_ITERATION_STEPS_PER_GRASP 20

/* Step size in mm when iterating backwards. */
#define BACK_ITERATION_STEP_SIZE 4.0

#define DEG_2_RAD M_PI/180.0

#include <QObject>
#include <QFile>

#include <QTextStream>

#include <vector>

#include "matvec3D.h"
#include "collisionStructures.h"
#include "grasp_directions.h"
#include "grasp_preshape.h"
#include "grasp_grasps.h"

class SoQtExaminerViewer;
class World;
class Hand;
class Grasp;
class GraspableBody;
class IVmgr;
class QualityMeasure;
class SoQtRenderArea;
class SoGroup;
class SoSensor;
class SoSeparator;
class Body;


//! This class is used to evaluate a set of candidate grasps
/*!
  Given a set of candidate grasps to test, this will perform each one and
  evaluate it using a quality measure of the user's choice.  For each grasp
  it performs the following steps:
  
  - Place and orient the hand at the starting pose of the candidate grasp
  - Set the hand %DOF values to the preshape specified
  - Make sure there are no collisions with other obstacles
  - Move the hand along the approach vector until contact occurs
  - Check that there is at least on contact with the object to be grasped
  - Close the fingers until contact blocks further movement
  - Evaluate the grasp

  If it is not a force closure grasp, it moves the hand back by a small step
  and tries the grasp again.  This is repeated until a force closure grasp is
  found or a max number of back steps is reached.  The results of the
  evaluations can be saved in a text file.

  In order to allow user interaction during the testing process (it takes a
  while), an Inventor idle sensor is used to test one grasp at a time when
  the system is otherwise idle.

  This class also handles setting up the grasp visualization window which
  shows all the position and orientation of all candidate grasps with respect
  to the object or the shape primitives.  As the testing process continues,
  this is updated to show the relative quality of each evaluated grasp.

  The result after testing is a list of the force closure grasps sorted in
  quality order from best to worst.

  When the testing is complete a signal is emitted (which is why this class
  must be derived from QObject).
  
 */
class grasp_tester : public QObject
{
  Q_OBJECT

private:

  //! A pointer to the main viewer
  SoQtExaminerViewer *myViewer;

  //! A pointer to the world containing the hand and object
  World              *my_world;

  //! A pointer to the hand doing the grasping
  Hand               *my_hand;

  //! A pointer to the grasped body
  GraspableBody      *my_body;

  //! A pointer to the hand's grasp class
  Grasp              *my_grasp;

  //! A pointer to the Inventor manager
  IVmgr                *ivmgr;

  //! Index of the quality measure used for evaluation
  int                  whichQM;

  //! Whether or not to save evaluation results to a text file
  bool                 saveToFile;

  //! The output file for testing results
  QFile               graspFile;

  //! Output stream that results are written to
  QTextStream         graspOut;

  //! A pointer to the grasp visualization window
  SoQtRenderArea     *projectionViewer;

  //! A pointer to the scene graph containing the shape primitives of the grasped object
  SoGroup *primitives;

  //! Collision report for testing hand collisions
  CollisionReport colReport;

  //! The list of grasps to be evaluated
    std::list<plannedGrasp*>* graspList;

  //! Whether or not to render hand motion during testing
  bool render;

  //! An iterator into the graspList
  std::list<plannedGrasp*>::iterator it_gr;

  //! Stores original pose of the grasping hand before testing begins
  transf origTran;

  //! An array of the original DOF values for the hand stored before testing begins
    double *dofs;
    
  //! A pointer to the Inventor sensor that allows testing only when the user isn't doing anything
  SoSensor *idleSensor;

  //! All visualized grasps are added to this separator
  SoSeparator        *glRoot;

  //! Number of grasps to be tested
  int nrOfGrasps;

  //! Number of grasps that have been tested
  int actualGraspNr;

  //! Maximum number of backsteps that may be taken
  int maxItStepNr;

  //! Distance in mm of on backstep
  double backStepSize;

/*
 * methods
 */
  bool putIt(plannedGrasp *pg, bool render_in=false);
  bool preshapeIt(preshape p, bool render_in=false);
  bool moveIt(cartesianGraspDirection gd, bool render_in=false);
  
  //  int  moveTo(transf); 
  bool checkContactToHand(GraspableBody* gb);
  void orderGraspListByQuality(std::list<plannedGrasp*>& grl);
  void saveGrasp(double quality);
  
  void updateGlobals();
  void savePosition(plannedGrasp& pg);
  bool handCollision();
  
  bool iteration(plannedGrasp& pg);
  static void testItCB(void *,SoSensor *sensor);
  void testIt();
  
signals:
  /*! This signal is emitted after the last grasp has been tested. */
  void testingComplete();
  
public:
  grasp_tester();
  ~grasp_tester();
  
  void setupGraspVisWindow(GraspableBody* myBody,SoGroup *prim);
  void visualizePlannedGrasps(std::list<plannedGrasp*> grList);
  bool callTestIt(std::list<plannedGrasp*>& graspList_in, bool render_in);
  
  bool set_testingParameters(int itStepNr, double stepSize);
  void get_testingParameters(int& itStepNr, double& stepSize);

  /*! Sets the index of the quality measure to use for evalutation. */
  void useQM(int qmNum) {whichQM = qmNum;}

  void saveGraspsToFile(const QString& filename,bool append);
  void continueTests();
  void pauseTests();
  
  //   struct timeval get_computingTime();
};


#endif

/******************
   Local Variables:
   mode:c++
   End:
******************/


