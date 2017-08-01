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
// $Id: graspPlanningTask.cpp,v 1.3 2010/02/11 18:38:25 cmatei Exp $
//
//######################################################################

#include "graspit/DBase/graspPlanningTask.h"

#include <QString>

#include "graspit/mytools.h"
#include "graspit/graspitCore.h"
#include "graspit/ivmgr.h"
#include "graspit/world.h"
#include "graspit/robot.h"
#include "graspit/body.h"
#include "EGPlanner/searchState.h"
#include "graspit/EGPlanner/loopPlanner.h"
#include "graspit/DBase/DBPlanner/db_manager.h"

#include "graspit/DBase/graspit_db_grasp.h"
#include "graspit/DBase/graspit_db_model.h"
#include "graspit/DBase/DBPlanner/ros_database_manager.h"
using namespace db_planner;

#include "graspit/debug.h"

GraspPlanningTask::GraspPlanningTask(TaskDispatcher *disp,
                                     db_planner::DatabaseManager *mgr,
                                     db_planner::TaskRecord rec) :
  Task(disp, mgr, rec),
  mObject(NULL),
  mPlanner(NULL)
{
}

GraspPlanningTask::~GraspPlanningTask()
{
  //remove the planning object from the world, but do not delete it
  if (mObject) {
    mObject->getWorld()->destroyElement(mObject, false);
    //clean up the loaded geometry
    //the model itself is left around. we don't have a good solution for that yet
    static_cast<GraspitDBModel *>(mPlanningTask.model)->unload();
  }
  delete mPlanner;
}

void GraspPlanningTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mRecord.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = FAILED;
    return;
  }

  World *world = graspitCore->getWorld();

  //check if the currently selected hand is the same as the one we need
  //if not, load the hand we need
  if (world->getCurrentHand() && world->getCurrentHand()->getDBName() == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    mHand = world->getCurrentHand();
  } else {
    QString handPath = mDBMgr->getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp Planning Task: loading hand from " << handPath.latin1());
    mHand = static_cast<Hand *>(world->importRobot(handPath));
    if (!mHand) {
      DBGA("Failed to load hand");
      mStatus = FAILED;
      return;
    }
  }
  //check for virtual contacts
  if (mHand->getNumVirtualContacts() == 0) {
    DBGA("Specified hand does not have virtual contacts defined");
    mStatus = FAILED;
    return;
  }

  //load the object
  GraspitDBModel *model = static_cast<GraspitDBModel *>(mPlanningTask.model);
  if (model->load(world) != SUCCESS) {
    DBGA("Grasp Planning Task: failed to load model");
    mStatus = FAILED;
    return;
  }
  mObject = model->getGraspableBody();
  mObject->addToIvc();
  world->addBody(mObject);

  //initialize the planner
  GraspPlanningState seed(mHand);
  seed.setObject(mObject);
  seed.setPositionType(SPACE_AXIS_ANGLE);
  seed.setPostureType(POSE_EIGEN);
  seed.setRefTran(mObject->getTran());
  seed.reset();

  mPlanner = new LoopPlanner(mHand);
  QObject::connect(mPlanner, SIGNAL(loopUpdate()), this, SLOT(plannerLoopUpdate()));
  QObject::connect(mPlanner, SIGNAL(complete()), this, SLOT(plannerComplete()));

  mPlanner->setEnergyType("CONTACT_ENERGY");
  mPlanner->setContactType(CONTACT_PRESET);
  mPlanner->setMaxSteps(65000);
  mPlanner->setRepeat(true);
  //max time set from database record
  if (mPlanningTask.taskTime >= 0) {
    mPlanner->setMaxTime(mPlanningTask.taskTime);
  } else {
    mPlanner->setMaxTime(-1);
  }
  static_cast<SimAnnPlanner *>(mPlanner)->setModelState(&seed);

  if (!mPlanner->resetPlanner()) {
    DBGA("Grasp Planning Task: failed to reset planner");
    mStatus = FAILED;
    return ;
  }

  //load all already known grasps so that we avoid them in current searches
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(mHand));
  std::vector<db_planner::Grasp *> graspList;
  if (!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)) {
    //for now, we don't know if this means "no grasps found" or "error" so we assume the first
    DBGA("No grasps found in database for model " << mPlanningTask.model->ModelName());
  }
  //and pass them on to the planner
  for (size_t i = 0; i < graspList.size(); i++) {
    GraspPlanningState *state = new GraspPlanningState(static_cast<GraspitDBGrasp *>(graspList[i])
                                                       ->getFinalGraspPlanningState());
    state->setObject(mObject);
    state->setPositionType(SPACE_AXIS_ANGLE, true);
    //careful here - is it the same posture in both spaces?
    state->setPostureType(POSE_EIGEN, true);
    static_cast<LoopPlanner *>(mPlanner)->addToAvoidList(state);
  }
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
  mLastSolution = mPlanner->getListSize();
  DBGA("Planner starting off with " << mLastSolution << " solutions");
  mPlanner->startPlanner();
  mStatus = RUNNING;
}

void GraspPlanningTask::plannerComplete()
{
  //save solutions that have accumulated in last loop
  plannerLoopUpdate();
  //finish
  mStatus = DONE;
}

void GraspPlanningTask::plannerLoopUpdate()
{
  if (mStatus != RUNNING) { return; }
  //save all new solutions to database
  for (int i = mLastSolution; i < mPlanner->getListSize(); i++) {
    //copy the solution so we can change it
    GraspPlanningState *sol = new GraspPlanningState(mPlanner->getGrasp(i));
    //convert it's tranform to the Quaternion__Translation format
    //make sure you pass it sticky=true, otherwise information is lost in the conversion
    sol->setPositionType(SPACE_COMPLETE, true);
    //we will want to save exact DOF positions, not eigengrasp values
    //again, make sure sticky=true
    sol->setPostureType(POSE_DOF, true);
    //we are ready to save it
    if (!saveGrasp(sol)) {
      DBGA("Grasp Planning Task: failed to save solution to dbase");
      mStatus = FAILED;
      break;
    }
  }
  if (mStatus == FAILED) {
    // this is a bit of a hack, but ensures that the planner will stop
    // as soon as it attempts to take another step. If we specifically start
    // the planner from in here, it causes problem, as this is called from inside
    // the planner callback
    mPlanner->setMaxSteps(0);
  } else {
    DBGA(mPlanner->getListSize() - mLastSolution << " solutions saved to database");
  }
  mLastSolution = mPlanner->getListSize();
}

bool GraspPlanningTask::saveGrasp(const GraspPlanningState *gps)
{
  GraspitDBModel *dbModel = mObject->getDBModel();
  assert(dbModel);

  db_planner::Grasp *grasp = new db_planner::Grasp;

  std::vector<double> contacts = grasp->GetContacts();

  grasp->SetSourceModel(*(static_cast<db_planner::Model *>(dbModel)));
  grasp->SetHandName(mHand->getDBName().toStdString());
  grasp->SetEpsilonQuality(0.0);
  grasp->SetVolumeQuality(0.0);
  grasp->SetEnergy(gps->getEnergy());
  grasp->SetClearance(0.0);
  grasp->SetClusterRep(false);

  grasp->SetSource("EIGENGRASPS");

  std::vector<double> tempArray;
  //the posture
  for (int i = 0; i < gps->readPosture()->getNumVariables(); ++i) {
    tempArray.push_back(gps->readPosture()->readVariable(i));
  }
  grasp->SetPregraspJoints(tempArray);
  grasp->SetFinalgraspJoints(tempArray);

  //the position
  tempArray.clear();
  for (int i = 0; i < gps->readPosition()->getNumVariables(); ++i) {
    tempArray.push_back(gps->readPosition()->readVariable(i));
  }
  grasp->SetPregraspPosition(tempArray);
  grasp->SetFinalgraspPosition(tempArray);

  //contacts
  //for some reason, the grasp's contact vector gets initialized to a mess!
  tempArray.clear();
  grasp->SetContacts(tempArray);

  std::vector<db_planner::Grasp *> graspList;
  graspList.push_back(grasp);

  bool result = mDBMgr->SaveGrasps(graspList);
  delete grasp;
  return result;
}
