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
// $Id: taskDispatcher.cpp,v 1.8 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#include "taskDispatcher.h"

#include <Inventor/sensors/SoTimerSensor.h>

#ifdef ROS_DATABASE_MANAGER
#include "DBPlanner/ros_database_manager.h"
#endif
#include "DBPlanner/sql_database_manager.h"

#include "graspitGUI.h"
#include "graspit_db_model.h"

#include "graspPlanningTask.h"
#include "preGraspCheckTask.h"
#include "graspTransferCheckTask.h"
#include "graspClusteringTask.h"
#include "tableCheckTask.h"
#include "compliantGraspCopyTask.h"

#include "debug.h"

TaskDispatcher::TaskDispatcher() : mDBMgr(NULL) , mCurrentTask(NULL), mStatus(READY)
{
        //note that we do not also schedule this guy
        mSensor = new SoTimerSensor(sensorCB, this);
        mSensor->setInterval( SbTime( 3.0 ));

	//hard-coded for now
	mMaxTasks = -1;
	mCompletedTasks = 0;
}

TaskDispatcher::~TaskDispatcher()
{
	if (mCurrentTask) {
		//this should not really happen
		DBGA("Dispatcher: deleting current task on cleanup");
		delete mCurrentTask;
	}
	delete mDBMgr;
	delete mSensor;
}

int TaskDispatcher::connect(std::string host, int port, std::string username, 
			   std::string password, std::string database) 
{
	delete mDBMgr;
	//careful: we pass a null for the grasp allocator as we don't know
	//yet which hand we'll be using
	std::ostringstream port_str;
	port_str << port;
#ifdef ROS_DATABASE_MANAGER
	mDBMgr = new db_planner::RosDatabaseManager(host, port_str.str(), username, password, database,
						    NULL,NULL);
        //use the special allocator for models that get geometry directly from the database
        GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(mDBMgr);
        mDBMgr->SetModelAllocator(allocator);
	if (!mDBMgr->isConnected()) {
		DBGA("DBase operator: Connection failed");
		delete mDBMgr; mDBMgr = NULL;
		return -1;
	}
	return 0;
#else
	DBGA("Task dispatcher only tested using the ROS database manager, which is not available");
	return -1;
#endif
}

/*! Gets a new task from the database and starts it. Possible outcomes:
  - no more tasks in database; sets status to NO_TASK
  - max number of tasks exceeded; sets status to DONE
  - error in reading the task; sets status to ERROR
  - error in starting the task; sets status to READY
  - task has started and needs us to surrender control; sets status to RUNNING
  - task is finished in one shot; sets status to READY
*/
void TaskDispatcher::startNewTask()
{
    //check if something is already running
    assert(!mCurrentTask);

    // check if we have completed the max number of tasks
    if (mMaxTasks >= 0 && mCompletedTasks >= mMaxTasks) {
        mStatus = DONE;
	return;
    }

    db_planner::TaskRecord rec;
    std::vector<std::string> empty;
    if (!mDBMgr->AcquireNextTask(&rec, empty)) {
        DBGA("Dispatcher: error reading next task");
	mStatus = ERROR;
	return;
    }
    DBGA("Task id: " << rec.taskId);
    //task type 0 is reserved for no task to do
    if (!rec.taskType.empty()) {
        DBGA("Dispatcher: no tasks to be executed");
	mStatus = NO_TASK;
	return;
    }
    mCurrentTask = mFactory.getTask(this, mDBMgr,rec);
    if (!mCurrentTask) {
        DBGA("Dispatcher: can not understand task type: " << rec.taskType);
	mStatus = ERROR;
	return;
    }

    //start the next task
    mCurrentTask->start();

    if (mCurrentTask->getStatus() == Task::RUNNING) {
        //task needs us to surrender control
        mStatus = RUNNING;
	DBGA("Dispatcher: started task of type " << rec.taskType);
    } else if (mCurrentTask->getStatus() == Task::DONE) {
        //task is done in one shot
	mStatus = READY;
	DBGA("Dispatcher: completed one-shot task of type " << rec.taskType);
    } else {
        // task had an error
        mStatus = READY;
	DBGA("Dispatcher: error starting task of type " << rec.taskType);
	return;
    }
}

/*! Checks on the current task; if it is finished, cleans up after it and marks the 
  result in the database.

  If task is finished, sets status to READY, unless there is an error marking the 
  finished task in the database, in which case status is set to ERROR.

  Nore that even if the task finishes with an error, the dispatcher will be READY
  for the next task (not abort altogether). The task that had an error is marked
  as such in the database. However, if there is an error in communicating with 
  the database, the dispatcher will abort altogether.
*/
void TaskDispatcher::checkCurrentTask()
{
    assert(mCurrentTask);

    switch (mCurrentTask->getStatus()) {
        case Task::RUNNING:
	    break;
        case Task::ERROR:
	    mStatus = READY;
	    //mark the task as error in the database
	    if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId, "ERROR")) {
	        DBGA("Dispatcher: error marking completed task");
		mStatus = ERROR;
	    }
	    delete mCurrentTask; mCurrentTask = NULL;
	    break;
        case Task::DONE:
	    mStatus = READY;
	    mCompletedTasks++;
	    //mark the task as completed in the database
	    if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId, "COMPLETED")) {
	        DBGA("Dispatcher: error marking completed task");
		mStatus = ERROR;
	    }
	    delete mCurrentTask; mCurrentTask = NULL;
	    break;
    }
}

/*! Will start tasks as long as there are tasks to be run. If the tasks are of the one-shot type,
  it just loops in here as long as it has tasks. If the task is event-based and needs us to 
  surrender control, it will surrender control but schedule the timer to come back here and 
  check on the task later.
*/
void TaskDispatcher::mainLoop()
{
    if (mSensor->isScheduled()) {
        mSensor->unschedule();
    }
    while(1){
        if(mCurrentTask) {
	    checkCurrentTask();
	}
	if (mStatus == READY) {
	    startNewTask();
	}
	switch(mStatus) {
	    case DONE:
		graspItGUI->exitMainLoop();		
	        return;
	    case ERROR:
		graspItGUI->exitMainLoop();		
	        return;
	    case NO_TASK:
		graspItGUI->exitMainLoop();		
	        return;
 	    case RUNNING:
	        mSensor->schedule();			
	        return;
	    case READY:
	        break;
	}
    }
}

void TaskDispatcher::sensorCB(void *data, SoSensor*)
{
	TaskDispatcher *dispatch = static_cast<TaskDispatcher*>(data);
	dispatch->mainLoop();
}

Task* TaskFactory::getTask(TaskDispatcher *op, db_planner::DatabaseManager *mgr, 
			   db_planner::TaskRecord rec)
{
  if (rec.taskType == "EMPTY") return new EmptyTask(op, mgr, rec);	  
  else if (rec.taskType == "EMPTY_ONE_SHOT") return new EmptyOneShotTask(op, mgr, rec);	  
  else if (rec.taskType == "GRASP_PLANNING") return new GraspPlanningTask(op, mgr, rec);
  else if (rec.taskType == "PREGRASP_CHECK") return new PreGraspCheckTask(op, mgr, rec);
  else if (rec.taskType == "GRASP_CLUSTERING") return new GraspClusteringTask(op, mgr, rec);
  else if (rec.taskType == "GRASP_TRANSFER") return new GraspTransferCheckTask(op, mgr, rec);
  else if (rec.taskType == "TABLE_CHECK") return new TableCheckTask(op, mgr, rec);
  else if (rec.taskType == "COMPLIANT_COPY")return new CompliantGraspCopyTask(op, mgr, rec);
  else return NULL;
}

void EmptyTask::start()
{
	mSensor = new SoTimerSensor(sensorCB, this);
	mSensor->setInterval( SbTime( 4.0 ));
	mSensor->schedule();			
	mStatus = RUNNING;
	DBGA("Empty task has started");
}

void EmptyTask::finish() 
{
	DBGA("Empty task has finished");
	mStatus = DONE;
}

EmptyTask::~EmptyTask() 
{
	DBGA("Empty task deleted");
	delete mSensor;
}

void EmptyTask::sensorCB(void *data, SoSensor*)
{
	EmptyTask *task = static_cast<EmptyTask*>(data);
	task->finish();
}

EmptyOneShotTask::~EmptyOneShotTask() 
{
	DBGA("Empty one-shot task deleted");
}

