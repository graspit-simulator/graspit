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
// Author(s):  Matei T. Ciocarlie
//
// $Id: preGraspCheckTask.h,v 1.2 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

#ifndef _PREGRASPCHECKTASK_H_
#define _PREGRASPCHECKTASK_H_

#include <vector>

#include "taskDispatcher.h"

class Hand;
class GraspableBody;
namespace db_planner {
  class Grasp;
}

//! Checks if the pre-grasps of the grasps in the database are valid
/*! Given a hand and an object, will load of the grasps from the database.
  For each grasp, will then check if a pre-grasp (defined as below) is valid
  and also if the path from pre-grasp to grasp is valid. Finally, will
  compute the min distance between the hand and the object for the pre-grasp.

  A pre-grasp is defined starting from a grasp as follows: first, open the 
  gripper a pre-specified amount. Then, back up along the approach direction
  for a pre-specified ammount.
 */
class PreGraspCheckTask : public Task {
 protected:
  //! The hand we are planning with
  Hand *mHand;
  //! The object we are planning on
  GraspableBody *mObject;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;

  //! Checks and sets the pre-grasp for a given grasp; returns false if an error is encountered
  bool checkSetGrasp(db_planner::Grasp *grasp);
  //! Computes the pre-grasp fora given grasp; returns true if pre-grasp is not reachable
  bool computePreGrasp(db_planner::Grasp *grasp);

  //! Loads the needed hand in the simulation world, if it's not already there
  void loadHand();

  //! Loads the needed object in the simulation world
  void loadObject();

  //! Empties a list of grasps; should really use some smart memory management here
  void emptyGraspList(std::vector<db_planner::Grasp*> &graspList);

 public:
  //! Just a stub for now
  PreGraspCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		    db_planner::TaskRecord rec);
  //! Removes the object that has been used from the sim world, but not the hand
  ~PreGraspCheckTask();

  //! Actually does all the work
  virtual void start();

  //! Checks if the pre-grasp starting from the current hand position is valid
  static bool preGraspCheck(Hand *hand);
};

#endif
