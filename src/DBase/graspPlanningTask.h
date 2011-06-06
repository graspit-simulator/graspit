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
// $Id: graspPlanningTask.h,v 1.1 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

#ifndef _GRASPPLANNINGTASK_H_
#define _GRASPPLANNINGTASK_H_

#include <QObject>

#include "taskDispatcher.h"

class Hand;
class GraspableBody;
class EGPlanner;
class GraspPlanningState;

//! Plans grasps for the hand and object, and stores them in the database
/*! For now, it does its own cleanup in the sense that it will remove from 
    the world and delete the object that was used for planning. However, it 
    will leave the hand in, as it might be used by subsequent tasks.

    On startup, it will load the hand it needs, unless that hand is already
    the currently selected hand in the world. It will also load the object
    it needs. It will not delete anything from the world ar startup.

    The init and cleanup so that the world is used by subsequent tasks is
    not well-defined yet, needs more work. Exactly what initialization and 
    cleanup in the GraspIt world such a planner should do is unclear, might 
    change in the future.
    
 */
class GraspPlanningTask : public QObject, public Task {
  Q_OBJECT
 private:
  //! The hand we are planning with
  Hand *mHand;
  //! The object we are planning on
  GraspableBody *mObject;
  //! The planner that we are using
  EGPlanner *mPlanner;
  //! The index of the last solution that was already saved in the database
  int mLastSolution;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;
  
  //! Saves a solution grasp to the database
  bool saveGrasp(const GraspPlanningState *gps);
  
public:
  //! Just a stub for now
  GraspPlanningTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
                    db_planner::TaskRecord rec);
  //! Removes the object that has been used from the sim world, but not the hand
  ~GraspPlanningTask();
  //! Loads the hand and the object, initializes and starts a loop planner
  virtual void start();
public slots:
  //! Connected to the loopUpdate() signal of the planner
  void plannerLoopUpdate();
  //! Connected to the complete() signal of the planner
  void plannerComplete();
};

#endif
