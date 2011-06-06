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
// $Id: graspTransferCheckTask.cpp,v 1.1 2010/04/12 20:16:11 cmatei Exp $
//
//######################################################################

#include "graspTransferCheckTask.h"

#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "searchState.h"
#include "DBPlanner/db_manager.h"

#include "preGraspCheckTask.h"

#include "graspit_db_grasp.h"
#include "graspit_db_model.h"

#include "debug.h"

GraspTransferCheckTask::GraspTransferCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
					       db_planner::TaskRecord rec) : Task (disp, mgr, rec)
{
  //nothing so far
}


GraspTransferCheckTask::~GraspTransferCheckTask()
{
  //remove the planning object from the world, but do not delete it
  mObject->getWorld()->destroyElement(mObject, false);
  //clean up the loaded geometry
  //the model itself is left around. we don't have a good solution for that yet
  static_cast<GraspitDBModel*>(mPlanningTask.model)->unload();
}

void emptyGraspListHack(std::vector<db_planner::Grasp*> &graspList)
{
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
}

void GraspTransferCheckTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mPlanningTask.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = ERROR;
    return;
  }

  World *world = graspItGUI->getIVmgr()->getWorld();

  if ( !world->getNumHands()) {
    QString handPath = GraspitDBGrasp::getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp transfer task: loading hands from " << handPath.latin1());	      
    mHand1 = static_cast<Hand*>(world->importRobot(handPath));
    mHand2 = static_cast<Hand*>(world->importRobot(handPath));
    if ( !mHand1 || !mHand2 ) {
      DBGA("Failed to load hand(s)");
      mStatus = ERROR;
      return;
    }
  } else if ( world->getNumHands()==2 &&
	      GraspitDBGrasp::getHandDBName(world->getHand(0)) == QString(mPlanningTask.handName.c_str()) &&
	      GraspitDBGrasp::getHandDBName(world->getHand(1)) == QString(mPlanningTask.handName.c_str()) ) {
    mHand1 = world->getHand(0);
    mHand2 = world->getHand(1);
  } else {
    DBGA("Grasp transfer task: wring hand(s) found");
    mStatus = ERROR;
    return;
  }
   
  //load the object
  GraspitDBModel *model = static_cast<GraspitDBModel*>(mPlanningTask.model);
  if (model->load(world) != SUCCESS) {
    DBGA("Grasp Planning Task: failed to load model");
    mStatus = ERROR;
    return;
  }
  mObject = model->getGraspableBody();
  mObject->addToIvc();
  world->addBody(mObject);

  //load all the grasps for hand 1
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(mHand1));
  std::vector<db_planner::Grasp*> graspList1;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList1)){
    DBGA("Load grasps failed for hand 1");
    mStatus = ERROR;
    emptyGraspListHack(graspList1);
    return;
  }
  //load all the grasps for hand 1
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(mHand2));
  std::vector<db_planner::Grasp*> graspList2;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList2)){
    DBGA("Load grasps failed for hand 2");
    mStatus = ERROR;
    emptyGraspListHack(graspList2);
    return;
  }

  //do the actual work
  bool success = true;
  for (int g1=0; g1<graspList1.size(); g1++) {
    for (int g2=g1; g2<graspList2.size(); g2++) {
      if (g1==g2) continue;
      DBGA("Checking combo " << g1 << " -- " << g2);      
      if (checkGraspCombo(graspList1[g1], graspList2[g2])) {
	//insert combo in database
	if (!mDBMgr->InsertGraspPair(graspList1[g1], graspList2[g2])) {
	  DBGA("  error inserting pair in database");
	  success = false;
	  break;
	} else {
	  DBGA("  inserted in database");
	}
      }
    }
  }

  emptyGraspListHack(graspList1);
  emptyGraspListHack(graspList2);
  if (success) mStatus = DONE;
  else mStatus = ERROR;
}

/*! Checks if grasp2 can be executed while grasp1 is active. 
  - executed both grasps
  - if collisions exist, returns false
  - checks if the pre-grasp for grasp2 can be executed
*/
bool GraspTransferCheckTask::checkGraspCombo(db_planner::Grasp* grasp1, db_planner::Grasp* grasp2)
{
  //place the hands in position
  GraspPlanningState *graspState1 = static_cast<GraspitDBGrasp*>(grasp1)->getFinalGraspPlanningState();
  graspState1->execute();  
  GraspPlanningState *graspState2 = static_cast<GraspitDBGrasp*>(grasp2)->getFinalGraspPlanningState();
  graspState2->execute();

  //check for collisions
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (!world->noCollision()) {
    DBGA("  initial grasps are in collision");
    return false;
  }

  //check for pre-grasp 2
  if (!PreGraspCheckTask::preGraspCheck(mHand2)) {
    DBGA("  pre grasp 2 fails");
    return false;
  }

  //put hand 2 back
  graspState2->execute();

  //check for pre-grasp 1
  if (!PreGraspCheckTask::preGraspCheck(mHand1)) {
    DBGA("  pre grasp 1 fails");
    return false;
  }

  return true;
}
