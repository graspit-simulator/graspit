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
// $Id: graspTransferCheckTask.h,v 1.1 2010/04/12 20:16:11 cmatei Exp $
//
//######################################################################

#ifndef _GRASPTRANSFERCHECKTASK_H_
#define _GRASPTRANSFERCHECKTASK_H_

#include "taskDispatcher.h"

class Hand;
class GraspableBody;
namespace db_planner {
  class Grasp;
}

//! Checks which two grasps of the same object can be executed concurrently
class GraspTransferCheckTask : public Task {
 private:
  //! The hands we are planning with
  Hand  *mHand1, *mHand2;
  //! The object we are planning on
  GraspableBody *mObject;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;
  
  //! Checks if a pair of grasps is compatible and writes it to the database
  bool checkGraspCombo(db_planner::Grasp* grasp1, db_planner::Grasp* grasp2);
  
 public:
  //! Just a stub for now
  GraspTransferCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
			 db_planner::TaskRecord rec);
  //! Removes the object that has been used from the sim world, but not the hand
  ~GraspTransferCheckTask();

  //! Actually does all the work
  virtual void start();
};

#endif
