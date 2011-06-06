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
// $Id: preGraspCheckTask.cpp,v 1.5 2010/09/30 18:49:06 cmatei Exp $
//
//######################################################################

#include "preGraspCheckTask.h"

#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "searchState.h"
#include "DBPlanner/db_manager.h"

#include "graspit_db_grasp.h"
#include "graspit_db_model.h"

#include "debug.h"

PreGraspCheckTask::PreGraspCheckTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
				     db_planner::TaskRecord rec) : Task (disp, mgr, rec)
{
  //nothing so far
}


PreGraspCheckTask::~PreGraspCheckTask()
{
  //remove the planning object from the world, but do not delete it
  mObject->getWorld()->destroyElement(mObject, false);
  //clean up the loaded geometry
  //the model itself is left around. we don't have a good solution for that yet
  static_cast<GraspitDBModel*>(mPlanningTask.model)->unload();
}

void PreGraspCheckTask::emptyGraspList(std::vector<db_planner::Grasp*> &graspList)
{
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
}

void PreGraspCheckTask::loadHand()
{
  World *world = graspItGUI->getIVmgr()->getWorld();
  
  //check if the currently selected hand is the same as the one we need
  //if not, load the hand we need
  if (world->getCurrentHand() && 
      GraspitDBGrasp::getHandDBName(world->getCurrentHand()) == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    mHand = world->getCurrentHand();
  } else {
    QString handPath = GraspitDBGrasp::getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp Planning Task: loading hand from " << handPath.latin1());	      
    mHand = static_cast<Hand*>(world->importRobot(handPath));
    if ( !mHand ) {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return;
    }
  }
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(mHand));

  //check for virtual contacts
  if (mHand->getNumVirtualContacts()==0) {
    DBGA("Specified hand does not have virtual contacts defined");
    mStatus = ERROR;
    return;
  }
}

void PreGraspCheckTask::loadObject()
{
  World *world = graspItGUI->getIVmgr()->getWorld();

  GraspitDBModel *model = static_cast<GraspitDBModel*>(mPlanningTask.model);
  if (model->load(world) != SUCCESS) {
    DBGA("Grasp Planning Task: failed to load model");
    mStatus = ERROR;
    return;
  }
  mObject = model->getGraspableBody();
  mObject->addToIvc();
  world->addBody(mObject);
}

void PreGraspCheckTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mRecord.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task id ");
    mStatus = ERROR;
    return;
  }

  loadHand();
  if (mStatus == ERROR) return;

  loadObject();
  if (mStatus == ERROR) return;

  //load all the grasps
  std::vector<db_planner::Grasp*> graspList;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)){
    DBGA("Load grasps failed");
    mStatus = ERROR;
    emptyGraspList(graspList);
    return;
  }

  bool success = true;
  std::vector<db_planner::Grasp*>::iterator it;
  for (it=graspList.begin(); it!=graspList.end(); it++) {
    if (!checkSetGrasp(*it)) {
      success = false;
      break;
    }
  }

  emptyGraspList(graspList);
  if (success) mStatus = DONE;
  else mStatus = ERROR;
}

bool PreGraspCheckTask::checkSetGrasp(db_planner::Grasp *grasp)
{
  if (!computePreGrasp(grasp)) {
    DBGA("Pre-grasp creation fails");
    //delete from database    
    if (!mDBMgr->DeleteGrasp(grasp)) {
      DBGA("Failed to delete grasp with id " << grasp->GraspId() << " from database");
      return false;
    }
    return true;
  }
  
  //sanity check
  if (!mHand->getWorld()->noCollision()) {
    DBGA("Collision detected for pre-grasp!");
    if (!mDBMgr->DeleteGrasp(grasp)) {
      DBGA("Failed to delete grasp with id " << grasp->GraspId() << " from database");
      return false;
    }
    return true;
  }

  //compute distance to object
  double clearance = mHand->getWorld()->getDist(mHand, mObject);
  grasp->SetClearance(clearance);

  //create pre-grasp
  GraspPlanningState *newPreGrasp = new GraspPlanningState(mHand);
  newPreGrasp->setPostureType(POSE_DOF, false);
  newPreGrasp->setPositionType(SPACE_COMPLETE, false);
  newPreGrasp->setRefTran(mObject->getTran(), false);
  newPreGrasp->saveCurrentHandState();
  //set pre-grasp
  static_cast<GraspitDBGrasp*>(grasp)->setPreGraspPlanningState(newPreGrasp);

  //save pre-grasp along with clearance in database
  //for now, we update by deleting and re-inserting
  if (!mDBMgr->DeleteGrasp(grasp)) {
    DBGA("Failed to delete grasp with id " << grasp->GraspId() << " from database");
    return false;
  }  

  /*
  //move the grasp back by 42mm
  transf t(Quaternion::IDENTITY, vec3(-42, 0, 0));
  GraspitDBGrasp* dbg = static_cast<GraspitDBGrasp*>(grasp);

  GraspPlanningState *movedPreGrasp = new GraspPlanningState(dbg->getPreGraspPlanningState());
  movedPreGrasp->getPosition()->setTran( t * movedPreGrasp->getPosition()->getCoreTran() );
  dbg->setPreGraspPlanningState(movedPreGrasp);

  GraspPlanningState *movedFinalGrasp = new GraspPlanningState(dbg->getFinalGraspPlanningState());
  movedFinalGrasp->getPosition()->setTran( t * movedFinalGrasp->getPosition()->getCoreTran() );
  dbg->setFinalGraspPlanningState(movedFinalGrasp);
  */

  //and save to database
  if (!mDBMgr->SaveGrasp(grasp)) {
    DBGA("Failed to save new grasp to database");
    return false;
  }
  DBGA("Pre-grasp inserted");
  return true;				
}

bool PreGraspCheckTask::computePreGrasp(db_planner::Grasp *grasp)
{
  //place the hand in position
  GraspPlanningState *graspState = static_cast<GraspitDBGrasp*>(grasp)->getFinalGraspPlanningState();
  graspState->execute();  

  //check the pre-grasp
  return preGraspCheck(mHand);
}

bool PreGraspCheckTask::preGraspCheck(Hand *hand)
{
  //the entire range of motion for the pr2_gripper_2010
  //also, positive change means open gripper for pr2_gripper
  double OPEN_BY = 0.323;
  //go back 5cm 
  double RETREAT_BY = -100;

  //open fingers
  std::vector<double> dof(hand->getNumDOF(),0.0);
  std::vector<double> stepSize(hand->getNumDOF(), 0.0);
  hand->getDOFVals(&dof[0]);
  for (int d=0; d<hand->getNumDOF(); d++) {
    dof[d] += OPEN_BY;
    if (dof[d] < hand->getDOF(d)->getMin()) {
      dof[d] = hand->getDOF(d)->getMin();
    }
    if (dof[d] > hand->getDOF(d)->getMax()) {
      dof[d] = hand->getDOF(d)->getMax();
    }
    stepSize[d] = M_PI/36.0;
  }
  hand->moveDOFToContacts(&dof[0], &stepSize[0], true, false);

  //check if move has succeeded
  for (int d=0; d<hand->getNumDOF(); d++) {
    if ( fabs( dof[d] - hand->getDOF(d)->getVal() ) > 1.0e-5) {
      DBGP("  trying to open to " << dof[d] << "; only made it to " << hand->getDOF(d)->getVal());
      DBGA("  open gripper fails");
      return false;
    }
  }

  //retreat along approach direction
  if (hand->approachToContact(RETREAT_BY, false)) {
    //we have hit something
    DBGA("  retreat fails");
    return false;
  }

  return true;
}
