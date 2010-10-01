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
// $Id: taskDispatcher.h,v 1.3 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

#ifndef _TASK_DISPATCHER_H_
#define _TASK_DISPATCHER_H_

#include <string>

#include "DBPlanner/task.h"

class SoSensor;
class SoTimerSensor;

namespace db_planner {
	class DatabaseManager;
	class Model;
	class Grasp;
}

class TaskDispatcher;

//! A task is an database record of some experiment that we want to run
/*! Usually, a task will involve an object, a hand, and some sort of
    experiment, such as planning grasps and recording them in the database.
    This class encapsulates a task, plus it offers some connections so that
    the task can be run automatically by a TaskDispatcher.

    A Task is responsible for its own event management. It can use a callback
    system (i.e. surrender control and then get it back later via its own timers
    or callbacks). For example, this can be achieved by using one of the EGPlanners,
    which do that natively.

    A Task must implement the following interface:
    
    - at the beginning, the TaskDispatcher will call the start() fctn. The 
    Task must start its own event management from there. 
    
    - if the Task is one-shot, it can do all the work in start() and set its 
    status to either DONE or ERROR at the end. The Dispatcher will see this, 
    clean up and go to the next Task.

    - if the Task is event based, start() must set up the Task event loop, then 
    leave mStatus = RUNNING, and then surrender control. The Dispatcher will in
    turn surrender control and let the Task event loop do its thing.

    - when the Task is done, it is responsible for setting its own mStatus to
    DONE. The TaskDispatcher wakes up periodically and checks on this. When it
    sees that the status of the Task is DONE it will clean up and continue to
    another task.
*/
class Task {
 public:
	enum Status{RUNNING, ERROR, DONE};
 protected:
	//! The current status of this task
	Status mStatus;
	//! The Dispatcher that started this task
	TaskDispatcher* mDispatcher;
	//! A database mgr that can be used to access the dbase
	/*! Acquired from the TaskDispatcher */
	db_planner::DatabaseManager *mDBMgr;
	//! The dbase record of the task being executed
	db_planner::TaskRecord mRecord;

 public:
        Task(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) 
		: mDispatcher(disp), mDBMgr(mgr), mRecord(rec) {};
	virtual ~Task(){}
	virtual void start() = 0;
	//! Returns the current status of this task
	Status getStatus(){return mStatus;}
	//! Returns a copy of the dbase record of this task
	db_planner::TaskRecord getRecord(){return mRecord;}
};

//! Given a taskType, returns an instance of the right type of task
class TaskFactory {
 public:
	inline Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
			     db_planner::TaskRecord rec);
};

//! A high-level executive that dispatches tasks based on the contents of the database
/*! The TaskDispatcher is in charge of reading the list of tasks to be
    executed from the database, creating and startinng the appropriate
    instances of the Task class and then monitoring them.

    It can run either one-shot tasks (that do something and they're done) or even-based
    tasks which have their own callback system. In the latter case, the Dispatcher uses 
    a callback timer of its own to wake up periodically and check on the currently running 
    task.

    On exit, the Dispatcher will exit GraspIt's main loop, thus terminating the
    application. Before doing that, it will set its own status to inform the
    main app of the outcome of the tasks. It can exit because:

    - it has performed a pre-set maximum number of tasks

    - no more tasks to be executed are listed in the database

    - there was an error communicating with the database

    Note that if a Task itself finishes with an error, the Dispacther will mark that in the
    database, then proceed to the next task.
*/
class TaskDispatcher 
{
 public:
        enum Status {READY, NO_TASK, ERROR, RUNNING, DONE};
 private:
	//! A factory for instantiating the right type of task
	TaskFactory mFactory;
	//! The db mgr used to connect to the dbase
	db_planner::DatabaseManager *mDBMgr;
	//! The task currently being executed
	Task *mCurrentTask;

	//! The status of the Dispatcher
	Status mStatus;
	//! The number of tasks completed so far
	int mCompletedTasks;
	//! Max number of tasks to be completed. -1 means no max limit
	int mMaxTasks;
	//! The timer sensor used to wake up periodically
	SoTimerSensor *mSensor;
 public:
	TaskDispatcher();
	~TaskDispatcher();

	//! Connects to the database. Returns 0 on success
	int connect(std::string host, int port, std::string username, 
		    std::string password, std::string database);

	//! Checks the status of the current task; cleans up after it if done
	void checkCurrentTask();
	//! Attempts to read a task from the dbase and start it
	void startNewTask();
	//! Main operation loop, called periodically
	void mainLoop();

	//! Returns the status of the Dispatcher
	Status getStatus() const {return mStatus;}

	//! Static sensor callback, just calls mainLoop()
	static void sensorCB(void *data, SoSensor*);
};

//! An empty task used to test and debug the dispatcher
/*! Simply calls a timer and completes a few seconds after starting.
*/
class EmptyTask : public Task
{
 private:
	SoTimerSensor *mSensor;
	void finish();
 public:
        EmptyTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
	          Task(disp, mgr, rec), mSensor(NULL) {}
	~EmptyTask();
	void start();
	static void sensorCB(void *data, SoSensor*);
};

//! An empty one-shot task used to test and debug the dispatcher
/*! Simply finishes immediately with success
*/
class EmptyOneShotTask : public Task
{
 public:
        EmptyOneShotTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
                         Task(disp, mgr, rec) {}
	~EmptyOneShotTask();
	void start(){mStatus = DONE;}
};

#endif
