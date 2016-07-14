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
// $Id: graspClusteringTask.h,v 1.1 2009/12/01 18:47:20 cmatei Exp $
//
//######################################################################

#ifndef _GRASPCLUSTERINGTASK_H_
#define _GRASPCLUSTERINGTASK_H_

#include "taskDispatcher.h"

class GraspitDBGrasp;

//! Selects the representative grasps (i.e. cluster centers) from a list of grasps
/*! Works by selecting the best grasp from a list, then setting it as the
  representative grasp for all other grasps that are close to it. Then does this
  iteratively on the remaining grasps.

  Two grasps are deemed close based on the position and orientation of they define
  plus the gripper angle.

  Will set the "cluster_rep" property of ALL grasps in the list based on what it
  thinks, i.e. mark it "true" for all those it thinks should be cluster reps and
  "false" for all others. 
 */
class GraspClusteringTask : public Task {
 private:
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;

  //! Returns true if two grasps are close enough to be clustered together
  bool clusterGrasps(const GraspitDBGrasp *g1, const GraspitDBGrasp *g2);
 public:
  //! Just a stub for now
  GraspClusteringTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		      db_planner::TaskRecord rec);
  //! Nothing to do here
  ~GraspClusteringTask(){}

  //! Actually does all the work
  virtual void start();
};


#endif
