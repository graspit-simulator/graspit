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
// $Id: grasp_manager.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_manager.h                                          */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the grasp_manager class (part of grasp planner)
*/

#ifndef __GRASP_MANAGER_H__
#define __GRASP_MANAGER_H__
#include <qstring.h>

/* This is the number of iterations used to compute
   a grasp. One iteration consists of:
   - plan grasps
   - compare them to eventually existing ones to
     extract those near to the best ones in the
     last set
   - test the remaining grasps 
   MAYBE THIS ISNT NEEDED ANY MORE, SHOULD THINK
   ABOUT IT... */
//#define NR_OF_PLAN_TEST_ITERATIONS 3

/* When doing an iteration step, maybe not all grasps with 
   quality > 0.0 should be considered as "good" grasps.
   Only grasps with quality >= ITERATION_QUALITY_THRESH
   are taken into account while looking for those in the
   new set that are worth testing. Should be accessible
   from the interface. */
#define ITERATION_QUALITY_THRESH 0.05

/* This is a double which defines the max distance for the
   compare step above. Grasps further away than this
   from any tested and stable grasp are not tested in
   this step. */
#define MAX_GRASP_DISTANCE_IN_ITERATION 0.2

/* When computing new grasps for an iteration step,
   more grasps are genererated than in the step before.
   This means better accuracy as well as longer testing 
   time. These defines fix the step size for each planning
   parameter. */
/* > 0 */
#define DEG_STEPS_STEP 40
/* > 0 and even */
#define PAR_PLANES_STEP 4
/* 1 or 2 (this is no step size, it only defines the value from seconds step on) */
#define DEG_180_STEP 1
/* >0 */
#define ROTATIONS_STEP 3

class GraspableBody;
class plannedGrasp;
class grasp_planner;
class grasp_presenter;
class grasp_tester;
class IVmgr;

//! Manages the various components of the grasp planning system.
/*!
  This controls the grasp planning process.  It is responsible for reading
  the shape primitives of the grasped object and calling the planner to
  generate candidate grasps, or it can read a set of candidate
  grasps from a text file.  Then it calls the tester to test the candidates.
  The tester returns a sorted list of the force closure grasps which are then
  sent to the presenter to allow the user to view each grasp in succession.
*/
class grasp_manager{

  //! A pointer to the grasped object
    GraspableBody *my_body;

  //! A list of candidate grasps, before testing.  After testing, contains a sorted list of force closure grasps.
    std::list<plannedGrasp*> graspList;

  //! if TRUE, the whole testing process is visualized. 
    bool renderIt;

  //! if TRUE, previously generated grasps (if there are any) are used to pick those out of the current set to be tested that are near to stable grasps.
    bool doIteration;

  //! TRUE if the graspList has not yet been sent to the presenter.
    int graspsNotShown;

  //! a pointer to the grasp planner
    grasp_planner *myPlanner;

  //! a pointer to the grasp presenter
    grasp_presenter* myPresent;

  //! a pointer to the inventor manager
  IVmgr *ivmgr;

  //! a pointer to the shape primitive scene graph
  SoGroup *primitives;

  // a pointer to the grasp tester, now global because of idle Sensor stuff */
//    grasp_tester  *myTester;

    /* same as define above */
//    int nrOfPlanTestIteration;

  //! a threshold value for deteriming if grasps in a new set are close enough to previously found good grasp
    double maxdist;
    
  //! a quality threshold value for determining if a grasp in the first set of tested grasps can be considered "good"
    double itQualThresh;

//    struct timeval computingTime;

  //! Counter for keeping statistics
    int nrOfPlannedGrasps;

  //! Counter for keeping statistics
    int nrOfStableGrasps;

    /* changes the parameters of the planner according to
       actual iteration step so that planning gets more accurate
       with increasing stepnumber */
//    void changePlanningParameters(int);

  void loadPrimitives();
  void compareGraspListsByDist(std::list<plannedGrasp*>&, std::list<plannedGrasp*>);

public:
    grasp_manager();
    ~grasp_manager();

  int readCandidateGraspsFile(const QString& filename);
  void generateGrasps();
  void testGrasps();
    void showGrasps(int next);
    void chooseGrasp();

    void set_render(bool);
    bool get_render()const;

    void set_doIteration(bool);
    bool get_doIteration()const;

    bool set_iterationParameters(double,double);
    void get_iterationParameters(double&,double&)const;

    grasp_planner* get_graspPlanner()const;
    grasp_tester*  get_graspTester()const;
};


#endif
/******************
   Local Variables:
   mode:c++
   End:
******************/



