//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
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
#include "egPlanner.h"
#include "robot.h"
#include "mytools.h"
#include "world.h"
#include "debug.h"
#include "graspitGUI.h"
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/sensors/SoSensor.h>


class GraspableBody;
class GraspPlanningState;
class GraspitDBModel;

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
protected:
	//! The hand we are planning with
	Hand *mHand;
	//! The object we are planning on
	GraspableBody *mObject;
	//! The planner that we are using
	EGPlanner *mPlanner;
	//! The index of the last solution that was already saved in the database
	int mLastSolution;

	//! Saves a solution grasp to the database
	bool saveGrasp(const GraspPlanningState *gps);

	void getHand();
	virtual void getObject();
	void startPlanner();
private:
	bool saveGrasps(){
		DBGA("Commented out for debug");
		return true;
		//		plannerLoopUpdate();
		//		if (mStatus == ERROR)
		//			return false;
		//		return true;
	}
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
		virtual void plannerComplete();
		void plannerUpdated();
};
/*Grasp planning task with factory for creating planner for added flexibility
The thing the factory creates must know how to destroy itself completely
FIXME - when branched graspits are merged, this should be completely merged in to GenericGraspPlanningTask*/
template <class EGPlannerFactory, class GraspSaver>
class GenericGraspPlanningTask:public GraspPlanningTask{
protected:
	EGPlannerFactory & mPlannerFactory;
	GraspSaver saver;
	//used to exit and kill the current planner
	SoTimerSensor *mTimerSensor;
	bool saveGrasps(){return saver.saveGraspList(mPlanner);};
	virtual void getObject(){
		if(mRecord.misc.find(".xml") == std::string::npos){
			GraspPlanningTask::getObject();
			return;
		}

		World * w = mHand->getWorld();
		//load world
		if (w->load(QString((string(getenv("GRASPIT")) + mRecord.misc).c_str())) != SUCCESS)
		{
			DBGA("harvardHandPlanningTask:: Failed to load world: " << mRecord.misc <<"\n");
			mStatus=ERROR;
			return;
		}
		if (w->getNumGB() != 1)
		{
			DBGA("harvardHandPlanningTask:: Graspable body number = : " << w->getNumGB() <<"  Fatal Error \n");
			mStatus=ERROR;
			return;
		}
		//set mObject
		mObject = w->getGB(0);
		mObject->setMaterial(wood);
		mHand->findInitialContact(10);
		mObject->setDBModel(static_cast<GraspitDBModel *>(mRecord.model));
		//disable collisions with main object
		for(int bi = 0; bi < w->getNumBodies(); bi ++){
			Body * btest = w->getBody(bi);
			if (btest == mObject)
				continue;
			if(dynamic_cast<Link *>(btest) || dynamic_cast<GraspableBody *>(btest))
				continue;
			w->toggleCollisions(false, btest, mObject);
		}
	};
public:
	GenericGraspPlanningTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr,
		db_planner::TaskRecord rec, EGPlannerFactory * egplannerFactory, const int taskNum):
	GraspPlanningTask(disp, mgr, rec), mPlannerFactory(*egplannerFactory), saver(mgr,taskNum), mTimerSensor(NULL){};
	virtual void start(){
		//sets mHand
		getHand();
		//sets mObject
		getObject();
		//get a planner from the factory

		mPlanner = mPlannerFactory.newPlanner(mHand, static_cast<GraspitDBModel*>(mRecord.model));
		if(graspItGUI->useConsole()){
			mHand->setRenderGeometry(false);
			mObject->setRenderGeometry(false);
			mPlanner->setRenderType(RENDER_NEVER);
		}
		else
			mPlanner->setRenderType(RENDER_LEGAL);
		//max time set from database record
		if (mRecord.taskTime >= 0){
			mPlanner->setMaxTime(mRecord.taskTime);
		} else {
			mPlanner->setMaxTime(-1);
		}
		startPlanner();
	};

	virtual void plannerComplete(){
		if (mStatus != RUNNING) return;

		delete mTimerSensor;
		mTimerSensor = new SoTimerSensor(sensorCB, this);
		mTimerSensor->setInterval( SbTime( 15.0 ));
		mTimerSensor->schedule();
		std::cerr << "timmer scheduled" << std::endl;


		//		mPlanner->pausePlanner();
		//		if(!saveGrasps()) {
		//			mStatus = ERROR;
		//			return;
		//		}
		//		mStatus = DONE;
		return;
	} ;

	static void sensorCB(void *data, SoSensor*)
	{
		GenericGraspPlanningTask* t = (GenericGraspPlanningTask*) data;
		std::cerr << "now, begin to save grasps" << std::endl;
		if(!t->saveGrasps()) {
			t->setStatus(t->ERROR);
			return;
		}
		t->setStatus(t->DONE);
	};

};

template <class EGPlannerFactory, class Saver, int task_descriptor_number>
class GenericGraspPlanningTaskFactory:public TaskFactory{
protected:
	EGPlannerFactory mEGPlannerFactory;
public:
	GenericGraspPlanningTaskFactory(db_planner::DatabaseManager * db): mEGPlannerFactory(db){};
	virtual Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		db_planner::TaskRecord rec){
			// if the task type is not one which this Planner can handle, return null
			if (rec.taskType != task_descriptor_number)
				return NULL;
			return new GenericGraspPlanningTask<EGPlannerFactory, Saver>(disp, mgr, rec, &mEGPlannerFactory, task_descriptor_number);
	};
};



class UpdateTactileContactTaskFactory: public TaskFactory {
private:
	db_planner::DatabaseManager * mDBMgr;
public:
	UpdateTactileContactTaskFactory(db_planner::DatabaseManager * db) : mDBMgr(db) {}
	virtual Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		db_planner::TaskRecord rec);
};


class UpdateTactileContactTask : public QObject, public Task {
	Q_OBJECT
private:
	//! The list of models available in the dbase, as retrieved by the DBMgr
	std::vector<db_planner::Model*> mModelList;
	//! The last model from the dbase that has been added to the Graspit world
	GraspitDBModel *mCurrentLoadedModel;
	//! A list of grasps for a dbase model, retrieved from the DBMgr
	std::vector<db_planner::Grasp*> mGraspList;

	std::vector<double> synthesizeTactileContacts();
	bool resolveCollision();
public:
	UpdateTactileContactTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
	  Task(disp, mgr, rec), mCurrentLoadedModel(NULL) {}
	  virtual void start();

};

/*
This class of task is supposed to test one object and one starting pose
A very generic blind grasping test
Whether using online search based on the cgdb database or using precomputed tactile
experience database is decided in blindPlannerDlg.cpp by the function BlindPlannerDlg::adjustButton_clicked()
*/
class BlindGraspingTestTaskFactory: public TaskFactory {
private:
	db_planner::DatabaseManager * mDBMgr;
public:
	BlindGraspingTestTaskFactory(db_planner::DatabaseManager * db) : mDBMgr(db) {}
	virtual Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		db_planner::TaskRecord rec);
};

class BlindGraspingTestTask : public QObject, public Task {
	Q_OBJECT
private:
	//! The list of models available in the dbase, as retrieved by the DBMgr
	std::vector<db_planner::Model*> mModelList;
	//! The last model from the dbase that has been added to the Graspit world
	GraspitDBModel *mCurrentLoadedModel;
	//! A list of grasps for a dbase model, retrieved from the DBMgr
	std::vector<db_planner::Grasp*> mGraspList;

	//std::vector<double> synthesizeTactileContacts();
	//bool resolveCollision();
public:
	BlindGraspingTestTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
	  Task(disp, mgr, rec), mCurrentLoadedModel(NULL) {}
	  virtual void start();
};

/*
This class of task is supposed to test one object
*/
class BlindGraspingPrecomputePerturbationTaskFactory: public TaskFactory {
private:
	db_planner::DatabaseManager * mDBMgr;
public:
	BlindGraspingPrecomputePerturbationTaskFactory(db_planner::DatabaseManager * db) : mDBMgr(db) {}
	virtual Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		db_planner::TaskRecord rec);
};

class BlindGraspingPrecomputePerturbationTask : public QObject, public Task {
	Q_OBJECT
private:
	//! The list of models available in the dbase, as retrieved by the DBMgr
	std::vector<db_planner::Model*> mModelList;
	//! The last model from the dbase that has been added to the Graspit world
	GraspitDBModel *mCurrentLoadedModel;
	//! A list of grasps for a dbase model, retrieved from the DBMgr
	std::vector<db_planner::Grasp*> mGraspList;
	//! DBMgr
	db_planner::DatabaseManager * mDBMgr;

	//std::vector<double> synthesizeTactileContacts();
	//bool resolveCollision();
public:
	BlindGraspingPrecomputePerturbationTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
	  mDBMgr(mgr),
	  Task(disp, mgr, rec), mCurrentLoadedModel(NULL) {}
	  virtual void start();

};
#endif
