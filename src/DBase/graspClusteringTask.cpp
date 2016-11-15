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
// $Id: graspClusteringTask.cpp,v 1.4 2010/09/01 23:55:27 cmatei Exp $
//
//######################################################################

#include "graspClusteringTask.h"

#include <algorithm>

#include "world.h"
#include "robot.h"
#include "graspitGUI.h"
#include "matvec3D.h"
#include "searchState.h"
#include "DBPlanner/db_manager.h"

#include "graspit_db_grasp.h"

#include "debug.h"

GraspClusteringTask::GraspClusteringTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
					 db_planner::TaskRecord rec) : Task (disp, mgr, rec)
{
  //nothing so far
}

/*! Will also load the hand, even though the hand is not explicitly used. It is needed for 
  the grasp allocator, plus it might be needed for retrieving DOF values, for example if 
  the grasp is stored in the database as eigengrasp values.
*/
void GraspClusteringTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mRecord.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = ERROR;
    return;
  }

  World *world = graspItGUI->getIVmgr()->getWorld();
  Hand *hand;
 
  //check if the currently selected hand is the same as the one we need
  //if not, load the hand
  if (world->getCurrentHand() && 
      GraspitDBGrasp::getHandDBName(world->getCurrentHand()) == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    hand = world->getCurrentHand();
  } else {
    QString handPath = GraspitDBGrasp::getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp Planning Task: loading hand from " << handPath.latin1());	      
    hand = static_cast<Hand*>(world->importRobot(handPath));
    if ( !hand ) {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return;
    }
  }
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(hand));

  //load all the grasps
  std::vector<db_planner::Grasp*> graspList;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)){
    DBGA("Load grasps failed");
    mStatus = ERROR;
    while (!graspList.empty()) {
      delete graspList.back();
      graspList.pop_back();
    }
    return;
  }

  //sort grasps by energy (hard-coded in, maybe later we'll allow other sorting)
  std::sort(graspList.begin(), graspList.end(), db_planner::Grasp::CompareEnergy);

  //if all goes well, we are done
  mStatus = DONE;
  int clusters = 0;
  DBGA("Clustering " << graspList.size() << " grasps");

  while (!graspList.empty()) {
    //pop the front (best grasp)
    db_planner::Grasp* repGrasp = graspList.front();
    graspList.erase(graspList.begin());

    //compliant_copy grasps are ignored by clustering tasks
    if (repGrasp->CompliantCopy()) {
      delete repGrasp;
      continue;
    }

    //mark it as cluster center in the database
    if (!mDBMgr->SetGraspClusterRep(repGrasp, true)) {
      DBGA("Failed to mark cluster rep in database");
      mStatus = ERROR;
      delete repGrasp;
      break;
    }
    clusters++;

    //find other grasps in its cluster
    int cloud=0;
    std::vector<db_planner::Grasp*>::iterator it = graspList.begin();
    while(it!=graspList.end()) {

      //compliant_copy grasps are ignored by clustering tasks
      if ( !(*it)->CompliantCopy() &&  
           clusterGrasps(static_cast<GraspitDBGrasp*>(repGrasp), static_cast<GraspitDBGrasp*>(*it)) ) {
	(*it)->SetClusterRep(false);
	//mark it as non-center in the database
	if (!mDBMgr->SetGraspClusterRep(*it, false)) {
	  DBGA("Failed to mark non-cluster rep in database");
	  mStatus = ERROR;
	  break;
	}
	cloud++;
	delete *it;
	it = graspList.erase(it);
      } else {
	it++;
      }

    }
    DBGA("  Marked cluster of size " << cloud);
    delete repGrasp;
    if (mStatus == ERROR) break;
  }  
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
  DBGA("Successfully marked " << clusters << " clusters");
}

/*! Only looks at the relative distance between gripper positions for the final grasps.
  Returns true if both of the follosing are true:
  - translation between gripper locations is less than DISTANCE_THRESHOLD
  - rotation angles between gripper locations is less than ANGULAR_THRESHOLD.

  Both thresholds are hard-coded in here.
 */
bool GraspClusteringTask::clusterGrasps(const GraspitDBGrasp *g1, const GraspitDBGrasp *g2)
{
  //2 cm distance threshold
  double DISTANCE_THRESHOLD = 20;
  //30 degrees angular threshold
  double ANGULAR_THRESHOLD = 0.52;
  
  transf t1 = g1->getHand()->getApproachTran() * g1->getFinalGraspPlanningState()->getTotalTran();
  transf t2 = g2->getHand()->getApproachTran() * g2->getFinalGraspPlanningState()->getTotalTran();

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

