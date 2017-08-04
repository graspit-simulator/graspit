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
// $Id: tableCheckTask.cpp,v 1.1 2010/04/12 20:16:11 cmatei Exp $
//
//######################################################################

#include "graspit/DBase/tableCheckTask.h"

#include "graspit/graspitCore.h"
#include "graspit/ivmgr.h"
#include "graspit/world.h"
#include "graspit/robot.h"
#include "graspit/body.h"
#include "graspit/debug.h"
#include "graspit/EGPlanner/searchState.h"

#include "graspit/DBase/DBPlanner/db_manager.h"
#include "graspit/DBase/graspit_db_grasp.h"

TableCheckTask::TableCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr,
                               db_planner::TaskRecord rec) : PreGraspCheckTask(disp, mgr, rec)
{
  //load the table
  World *world = graspitCore->getWorld();
  QString path = QString(getenv("GRASPIT")) + QString("/models/objects/plane.xml");
  mTable = world->importBody("Body", path);
  if (!mTable) {
    DBGA("Failed to load table");
    mStatus = FAILED;
  }
}

TableCheckTask::~TableCheckTask()
{
  World *world = graspitCore->getWorld();
  if (mTable) {
    world->destroyElement(mTable, true);
  }
}

void TableCheckTask::start()
{
  if (mStatus == FAILED) { return; }

  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mPlanningTask.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = FAILED;
    return;
  }

  loadHand();
  if (mStatus == FAILED) { return; }

  loadObject();
  if (mStatus == FAILED) { return; }

  //place the table in the right position
  //start way under the object
  mTable->setTran(transf(Quaternion::Identity(), vec3(0.0, 0.0, -200.0)));
  //and move up until it touches the object
  transf tr(Quaternion::Identity(), vec3(0.0, 0.0, 100.0));

  World *world = graspitCore->getWorld();
  world->toggleCollisions(false, mHand, mTable);
  mTable->moveTo(tr, 5.0, M_PI / 36.0);
  world->toggleCollisions(true, mHand, mTable);
  DBGA("Table z location: " << mTable->getTran().translation().z());

  //load all the grasps
  std::vector<db_planner::Grasp *> graspList;
  if (!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)) {
    DBGA("Load grasps failed");
    mStatus = FAILED;
    emptyGraspList(graspList);
    return;
  }

  bool success = true;
  std::vector<db_planner::Grasp *>::iterator it;
  for (it = graspList.begin(); it != graspList.end(); it++) {
    if (!checkSetGrasp(*it)) {
      success = false;
      break;
    }
  }

  emptyGraspList(graspList);
  if (success) { mStatus = DONE; }
  else { mStatus = FAILED; }
}

bool TableCheckTask::checkSetGrasp(db_planner::Grasp *grasp)
{
  double distance = getTableClearance(grasp);

  if (!mDBMgr->SetGraspTableClearance(grasp, distance)) {
    DBGA("Failed to mark table clearance in database");
    return false;
  }

  DBGA("Saved clearance: " << distance);
  return true;
}

double TableCheckTask::getTableClearance(db_planner::Grasp *grasp)
{
  //place the hand in position
  GraspPlanningState *graspState = static_cast<GraspitDBGrasp *>(grasp)->getFinalGraspPlanningState();
  graspState->execute();

  //check distance for grasp
  World *world = graspitCore->getWorld();
  double distance = world->getDist(mHand, mTable);
  if (distance < 0) {
    DBGA(" Grasp is in collision with table");
    return distance;
  }
  DBGA(" Grasp clearance: " << distance);
  //check if pre-grasp is feasible
  if (!computePreGrasp(grasp)) {
    DBGA(" Pre-grasp is in collision with table");
    return -1.0;
  }

  double pre_distance = world->getDist(mHand, mTable);
  DBGA(" Pre-grasp clearance: " << pre_distance);

  return std::min(distance, pre_distance);
}

