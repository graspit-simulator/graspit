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
// $Id: tableCheckTask.h,v 1.1 2010/04/12 20:16:11 cmatei Exp $
//
//######################################################################

#ifndef _TABLECHECKTASK_H_
#define _TABLECHECKTASK_H_

#include "preGraspCheckTask.h"

class Hand;
class GraspableBody;
class Body;
namespace db_planner {
  class Grasp;
}

//! Checks if grasps are feasible in the presence of a table that the object is sitting on
/*! Table is hardcoded to be oriented along the z axis */
class TableCheckTask : public PreGraspCheckTask {
 private:

  //! The table we are using
  Body *mTable;

  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;

  //! Checks and sets validity for a grasp; returns false if an error is encountered
  bool checkSetGrasp(db_planner::Grasp *grasp);

  //! Returns the smallest table clearance between the grasp and the pre-grasp
  /*! Returns a negative value if either is in collision with the table */
  double getTableClearance(db_planner::Grasp *grasp);

 public:
  //! Loads in the table
  TableCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		 db_planner::TaskRecord rec);

  //! Removes the object and the table that has been used from the sim world, but not the hand
  ~TableCheckTask();

  //! Also places the table in the right position "under" the object
  virtual void start();
};

#endif
