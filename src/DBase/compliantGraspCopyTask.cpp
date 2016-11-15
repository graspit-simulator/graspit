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
// $Id: compliantGraspCopyTask.cpp,v 1.1 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#include "compliantGraspCopyTask.h"

#include <memory>

#include "world.h"
#include "robot.h"
#include "matvec3D.h"
#include "searchState.h"
#include "pr2Gripper.h"

#include "DBPlanner/db_manager.h"
#include "graspit_db_grasp.h"
#include "debug.h"

CompliantGraspCopyTask::CompliantGraspCopyTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
                                               db_planner::TaskRecord rec) : PreGraspCheckTask (disp, mgr, rec)
{
  //nothing so far
}

void CompliantGraspCopyTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mPlanningTask.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = ERROR;
    return;
  }

  loadHand();
  if (mStatus == ERROR) return;

  if (!mHand->isA("Pr2Gripper2010")) {
    DBGA("Compliant copy task only works on the PR2 gripper");
    mStatus = ERROR;
    return;
  }
  Pr2Gripper2010* gripper = static_cast<Pr2Gripper2010*>(mHand);

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
    GraspPlanningState *graspState = static_cast<GraspitDBGrasp*>(*it)->getFinalGraspPlanningState();
    gripper->setCompliance(Pr2Gripper2010::NONE);
    graspState->execute();
    DBGA("Compliant copy around finger 0");
    if (!compliantCopy(*it, Pr2Gripper2010::FINGER0)) {
      success = false;
      break;
    }
    gripper->setCompliance(Pr2Gripper2010::NONE);
    graspState->execute();
    DBGA("Compliant copy around finger 1");
    if (!compliantCopy(*it, Pr2Gripper2010::FINGER1)) {
      success = false;
      break;
    }
  }
  gripper->setCompliance(Pr2Gripper2010::NONE);

  emptyGraspList(graspList);
  if (success) mStatus = DONE;
  else mStatus = ERROR;
}

bool CompliantGraspCopyTask::compliantCopy(const db_planner::Grasp *grasp, Pr2Gripper2010::ComplianceType compliance)
{
  //open slowly (2 degrees increments)
  //also, positive change means open for the pr2 gripper
  double OPEN_BY = 0.035; 
  Pr2Gripper2010* gripper = static_cast<Pr2Gripper2010*>(mHand);
  gripper->setCompliance(compliance);

  std::vector<double> dof(mHand->getNumDOF(),0.0);
  std::vector<double> stepSize(mHand->getNumDOF(), M_PI/36.0);
  mHand->getDOFVals(&dof[0]);
  bool done = false;
  transf lastTran = mHand->getTran();
  while (!done)
  {
    //DBGA("Move loop");
    //open the hand a little bit
    for (int d=0; d<mHand->getNumDOF(); d++) {
      dof[d] += OPEN_BY;
    }
    mHand->checkSetDOFVals(&dof[0]);
    mHand->moveDOFToContacts(&dof[0], &stepSize[0], true, false);

    //if we are far enough from the last saved grasp, go ahead and save a new one
    transf currentTran = mHand->getTran();
    if (!similarity(currentTran, lastTran)) {
      //save the grasp
      DBGA("Storing a compliant copy");
      //compliance messes up the pre-grasp computation
      gripper->setCompliance(Pr2Gripper2010::NONE);
      if (!checkStoreGrasp(grasp)) {
        return false;
      }
      gripper->setCompliance(compliance);
      //remember the last saved grasp
      lastTran = currentTran;
    }
      
    //if we have not opened as much as we wanted to, we've hit something; we are done
    for (int d=0; d<mHand->getNumDOF(); d++) {
      if ( fabs(mHand->getDOF(d)->getVal() - dof[d]) > 1.0e-5 || 
           dof[d] == mHand->getDOF(d)->getMin() || 
           dof[d] == mHand->getDOF(d)->getMax()) {
        //DBGA("Done moving");
        done = true;
        break;
      }
    }    
  }
  return true;
}

bool CompliantGraspCopyTask::checkStoreGrasp(const db_planner::Grasp *original)
{
  //sanity check if the world is in collision
  if (!mHand->getWorld()->noCollision()) {
    DBGA(" World is in collision");
    return true;
  }
  //create the new grasp as a copy of the old one
  //this should copy score and everything
  const GraspitDBGrasp *graspit_original = static_cast<const GraspitDBGrasp*>(original);
  std::auto_ptr<GraspitDBGrasp> newGrasp(new GraspitDBGrasp(*graspit_original));
  //new grasp is a compliant copy of the old one
  newGrasp->SetCompliantCopy(true);
  newGrasp->SetCompliantOriginalId(original->GraspId());
  //compliant copy grasps are never cluster reps
  newGrasp->SetClusterRep(false);
  //set the grasp posture
  GraspPlanningState *graspState = new GraspPlanningState(mHand);
  graspState->setPostureType(POSE_DOF, false);
  graspState->setPositionType(SPACE_COMPLETE, false);
  graspState->setRefTran(mObject->getTran(), false);
  graspState->saveCurrentHandState();
  newGrasp->setFinalGraspPlanningState(graspState);

  //prepare the pre-grasp
  GraspPlanningState *preGraspState = new GraspPlanningState(mHand);
  preGraspState->setPostureType(POSE_DOF, false);
  preGraspState->setPositionType(SPACE_COMPLETE, false);
  preGraspState->setRefTran(mObject->getTran(), false);
  //compute the pre-grasp posture; careful to leave the hand in the right place
  mHand->saveState();
  bool pre_grasp_result = computePreGrasp(newGrasp.get());
  preGraspState->saveCurrentHandState();
  mHand->restoreState();
  //check the pre-grasp
  if (!pre_grasp_result) {
    DBGA(" Pre-grasp creation fails");
    return true;
  }
  //sanity check
  if (!mHand->getWorld()->noCollision()) {
    DBGA(" World is in collision AFTER PREGRASP COMPUTATION");
    return true;
  }
  //set the pre-grasp
  newGrasp->setPreGraspPlanningState(preGraspState);
  //compute distance to object for pre grasp
  double clearance = mHand->getWorld()->getDist(mHand, mObject);
  newGrasp->SetClearance(clearance);

  //store the new grasp in the database
  if (!mDBMgr->SaveGrasp(newGrasp.get())) {
    DBGA(" Error writing new grasp to database");
    return false;
  }
  return true;
}

/*! This uses a slightly different similarity metric, more in tune with what we use for clustering
  but different from the one used during actual grasp planning, which will lead to inconsistencies
  in the sampling. This *should* not create real problems though.

  The angular threshold does not really play any role here as compliant copies *should* not have
  any angular differences.
*/  
bool CompliantGraspCopyTask::similarity(const transf &t1, const transf &t2)
{
  //7.5mm distance threshold
  double DISTANCE_THRESHOLD = 7.5;
  //15 degrees angular threshold
  double ANGULAR_THRESHOLD = 0.26;
  
  vec3 dvec = t1.translation() - t2.translation();
  double d = dvec.len();
  if (d > DISTANCE_THRESHOLD) return false;
  
  Quaternion qvec = t1.rotation() * t2.rotation().inverse();
  vec3 axis; double angle;
  qvec.ToAngleAxis(angle,axis);
  if (angle >  M_PI) angle -= 2*M_PI;
  if (angle < -M_PI) angle += 2*M_PI;
  if (fabs(angle) > ANGULAR_THRESHOLD) return false;
  
  return true;
}

/*! This is copied over from the distance function of the SearchState so that we use
  the same metric as when doing the actual grasp planning.
*/
bool CompliantGraspCopyTask::searchSimilarity(const transf &t1, const transf &t2)
{
  //we use the same threshold as in grasp planning
  double DISTANCE_THRESHOLD = 0.01;

  vec3 dvec = t1.translation() - t2.translation();
  double d = dvec.len() / mObject->getMaxRadius();  
  Quaternion qvec = t1.rotation() * t2.rotation().inverse();
  vec3 axis; double angle;
  qvec.ToAngleAxis(angle,axis);
  //0.5 weight out of thin air
  double q = 0.5 * fabs(angle) / M_PI;
  if (std::max(d,q) > DISTANCE_THRESHOLD) return false;
  return true;
}
