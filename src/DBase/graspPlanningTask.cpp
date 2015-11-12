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
// $Id: graspPlanningTask.cpp,v 1.1 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

#include "graspPlanningTask.h"

#include <QString>

#include "mytools.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "searchState.h"
#include "loopPlanner.h"

#include "DBPlanner/sql_database_manager.h"
#include "graspit_db_grasp.h"
#include "graspit_db_model.h"

#include "SensorInterface.h"
#include "BlindPlanner/blindPlannerUtil.h"
#include "BlindPlanner/blindPlannerDlg.h"

#include "debug.h"

GraspPlanningTask::GraspPlanningTask(TaskDispatcher *disp, 
									 db_planner::DatabaseManager *mgr,
									 db_planner::TaskRecord rec) : Task(disp, mgr, rec)
{
	//nothing so far
}

GraspPlanningTask::~GraspPlanningTask()
{
	//remove the planning object from the world, but do not delete it
	//mObject->getWorld()->destroyElement(mObject, false);
	//clean up the loaded geometry
	//the model itself is left around. we don't have a good solution for that yet
	static_cast<GraspitDBModel*>(mRecord.model)->unload();
	delete mPlanner;
}
void GraspPlanningTask::getHand(){
	World *world = graspItGUI->getIVmgr()->getWorld();

	//check if the currently selected hand is the same as the one we need
	//if not, load the hand we need
	if (world->getCurrentHand() && 
		GraspitDBGrasp::getHandDBName(world->getCurrentHand()) == QString(mRecord.handName.c_str())) {
			DBGA("Grasp Planning Task: using currently loaded hand");
			mHand = world->getCurrentHand();
	} else {
		mHand = GraspitDBGrasp::loadHandFromDBName(QString(mRecord.handName.c_str()));
		if ( !mHand || !mDBMgr->setHand(mHand)) {
			DBGA("Failed to load hand");
			mStatus = ERROR;
			return;
		}
	}
	std::cout << "world has " << world->getNumHands() << "hands" << std::endl;
	//check for virtual contacts
	if (mHand->getNumVirtualContacts()==0) {
		DBGA("Specified hand does not have virtual contacts defined");
		mStatus = ERROR;
		return;
	}
}

void GraspPlanningTask::startPlanner(){
	QObject::connect(mPlanner, SIGNAL(complete()), this, SLOT(plannerComplete()));
	//	QObject::connect(mPlanner, SIGNAL(loopUpdate()), this, SLOT(plannerLoopUpdate()));
	QObject::connect(mPlanner, SIGNAL(update()), this, SLOT(plannerUpdated()));
	if (!mPlanner->resetPlanner()) {
		DBGA("Grasp Planning Task: failed to reset planner");
		mStatus = ERROR;
		return ;
	}

	//load all already known grasps so that we avoid them in current searches
	//...
	mLastSolution = mPlanner->getListSize();
	DBGA("Planner started");
	mPlanner->startPlanner();
	mStatus = RUNNING;
}

void GraspPlanningTask::getObject(){
	//load the object
	GraspitDBModel *model = static_cast<GraspitDBModel*>(mRecord.model);
	DBGA( "loading object: " << model->ModelName().c_str() );
	if (model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
		//attempt repair
		DBGA("Grasp Planning Task: failed to load model");
		mStatus = ERROR;
		return;
	}
	mObject = model->getGraspableBody();
	DBGA("GraspitDBGrasp: loading object: " << model->ModelName().c_str());
	mObject->addToIvc();
	graspItGUI->getIVmgr()->getWorld()->addBody(mObject);
	mObject->setMaterial(wood);
	return;
}
void GraspPlanningTask::start()
{
	//sets mHand
	getHand();
	//sets mObject
	getObject();
	//initialize the planner
	GraspPlanningState seed(mHand);
	seed.setObject(mObject);
	seed.setPositionType(SPACE_AXIS_ANGLE);
	seed.setPostureType(POSE_EIGEN);
	seed.setRefTran(mObject->getTran());
	seed.reset();

	mPlanner = new LoopPlanner(mHand);	
	mPlanner->setEnergyType(ENERGY_CONTACT);
	mPlanner->setContactType(CONTACT_PRESET);
	//	mPlanner->setMaxSteps(65000);
	mPlanner->setRepeat(true);
	//max time set from database record
	if (mRecord.taskTime >= 0){
		mPlanner->setMaxTime(mRecord.taskTime);
	} else {
		mPlanner->setMaxTime(-1);
	}
	static_cast<SimAnnPlanner*>(mPlanner)->setModelState(&seed);
	startPlanner();
}

void GraspPlanningTask::plannerComplete()
{
	DBGA("planner Complete");
	//save solutions that have accumulated in last loop
	//plannerLoopUpdate();
	if(!saveGrasps())
		mStatus = ERROR;
	else	//finish
		mStatus = DONE;
}

void GraspPlanningTask::plannerLoopUpdate()
{
	if (mStatus != RUNNING) return;
	//save all new solutions to database
	for(int i=mLastSolution; i<mPlanner->getListSize(); i++) {
		//copy the solution so we can change it
		GraspPlanningState *sol = new GraspPlanningState(mPlanner->getGrasp(i));
		//convert it's tranform to the Quaternion__Translation format
		//make sure you pass it sticky=true, otherwise information is lost in the conversion
		sol->setPositionType(SPACE_COMPLETE,true);
		//we will want to save exact DOF positions, not eigengrasp values
		//again, make sure sticky=true
		sol->setPostureType(POSE_DOF,true);
		//we are ready to save it
		if (!saveGrasp(sol)) {
			DBGA("GraspPlanningState: failed to save solution to dbase");
			mStatus = ERROR;
			break;
		}
	}
	if (mStatus == ERROR) {
		mPlanner->stopPlanner();
		mStatus = ERROR;
	} else {
		DBGA(mPlanner->getListSize() - mLastSolution << " solutions saved to database");
	}
	mLastSolution = mPlanner->getListSize();
}

bool GraspPlanningTask::saveGrasp(const GraspPlanningState *gps)
{
	GraspitDBModel* dbModel= mObject->getDBModel();
	assert(dbModel);

	db_planner::Grasp* grasp = new db_planner::Grasp;

	std::vector<double> contacts = grasp->GetContacts();

	grasp->SetSourceModel( *(static_cast<db_planner::Model*>(dbModel)) );
	grasp->SetHandName(GraspitDBGrasp::getHandDBName(mHand).toStdString());
	grasp->SetEpsilonQuality(0.0);
	grasp->SetVolumeQuality(0.0);
	grasp->SetEnergy(gps->getEnergy());

	grasp->SetSource("EIGENGRASPS_TASK_"+QString::number(mRecord.taskType).toStdString());

	std::vector<double> tempArray;
	//the posture
	for(int i = 0; i < gps->readPosture()->getNumVariables(); ++i){
		tempArray.push_back(gps->readPosture()->readVariable(i));
	}
	grasp->SetPregraspJoints(tempArray);
	grasp->SetFinalgraspJoints(tempArray);

	//the position
	tempArray.clear();
	for(int i = 0; i < gps->readPosition()->getNumVariables(); ++i){
		tempArray.push_back(gps->readPosition()->readVariable(i));
	}
	grasp->SetPregraspPosition(tempArray);
	grasp->SetFinalgraspPosition(tempArray);

	//contacts
	//for some reason, the grasp's contact vector gets initialized to a mess!
	tempArray.clear();
	grasp->SetContacts(tempArray);

	std::vector<db_planner::Grasp*> graspList;
	graspList.push_back(grasp);

	bool result = mDBMgr->SaveGrasps(graspList);
	delete grasp;
	return result;
}


void GraspPlanningTask::plannerUpdated(){
	if (mStatus == ERROR) {
		mPlanner->stopPlanner();
		mStatus = ERROR;
	} else {
		//DBGA(mPlanner->getListSize() << " solutions found ");
	}
	//  DBGA("Step " << mPlanner->getCurrentStep() << " of " << mPlanner->getMaxSteps() << " Running Time " << mPlanner->getRunningTime() <<"of " << mPlanner->getMaxRunningTime() );
}

//----------------- Update tactile contacts --------------------//
Task* UpdateTactileContactTaskFactory::getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
											   db_planner::TaskRecord rec)
{
	return new UpdateTactileContactTask(disp, mgr, rec);
}

void UpdateTactileContactTask::start()
{
	mStatus = RUNNING;

	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
		mCurrentLoadedModel = NULL;
	}

	//check out the model in the modelList
	GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mRecord.model);
	if(!model){
		DBGA("Cannot recognize the model");
		mStatus = ERROR;
		return;
	}
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
			mStatus = ERROR;
			return;
		}
	}
	//adds the object to the collision detection system
	model->getGraspableBody()->addToIvc();
	//todo: where to dynamic information come from?
	//model->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world so that we can see it
	graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
	//and remember it
	mCurrentLoadedModel = model;

	//clear the grasps
	for(size_t i = 0;i < mGraspList.size(); ++i)
	{
		delete mGraspList[i];
	}
	mGraspList.clear();
	//load the grasps
	if(!mDBMgr->GetGrasps(*mCurrentLoadedModel,mRecord.handName, &mGraspList)){
		DBGA("Load grasps failed");
		mGraspList.clear();
		mStatus = ERROR;
		return;
	}

	//loop through all the grasps of this object
	for(size_t i = 0; i < mGraspList.size(); ++i)
	{
		std::cout << i << " / " << mGraspList.size() << std::endl;
		if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState())
		{

			mStatus = ERROR;
			return;
		}
		static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState()->execute();
		//if(!resolveCollision())
		//{
		//	DBGA("cannot solve collision");
		//}
		graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(false);
		//update the world and grasp
		//	DBGA("Find contacts");
		graspItGUI->getIVmgr()->getWorld()->findAllContacts();
		//	DBGA("Update grasps");
		graspItGUI->getIVmgr()->getWorld()->updateGrasps();

		std::vector<double> results;
		results = synthesizeTactileContacts();

		mDBMgr->UpdateGraspTactileContacts(mGraspList[i]->GraspId(), results);

	}
	//delete the object
	graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
	mCurrentLoadedModel = NULL;

	mStatus = DONE;

}

std::vector<double> UpdateTactileContactTask::synthesizeTactileContacts()
{
	std::vector<double> result;

	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		w->getSensor(sensorInd)->updateSensorModel();
		if(w->getSensor(sensorInd)->getNormalForce() )
		{

			transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
			transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
			transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

			result.push_back( w->getSensor(sensorInd)->getNormalForce() );

			result.push_back( sensorInHand.translation().x() );
			result.push_back( sensorInHand.translation().y() );
			result.push_back( sensorInHand.translation().z() );

			result.push_back( sensorInHand.rotation().w );
			result.push_back( sensorInHand.rotation().x );
			result.push_back( sensorInHand.rotation().y );
			result.push_back( sensorInHand.rotation().z );

		}
	}

	return result;

}

bool UpdateTactileContactTask::resolveCollision()
{
	if(graspItGUI->getIVmgr()->getWorld()->noCollision())
	{
		return true;
	}
	DBGA("in need of collision resolution");

	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	transf newTran = translate_transf(vec3(0,0,-500) * 
		h->getApproachTran()) * h->getTran();
	//move hand back out of collision
	h->setTran(newTran);
	//open the fingers with the spread angle being kept
	h->autoGrasp(false, -1.0);
	//move forward
	return h->approachToContact(500, false);

}



Task* BlindGraspingTestTaskFactory::getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
											db_planner::TaskRecord rec)
{
	return new BlindGraspingTestTask(disp, mgr, rec);
}

void BlindGraspingTestTask::start()
{
	mStatus = RUNNING;

	DBGA("task_id: " << mRecord.taskId );

	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * hand = w->getCurrentHand();
	//load a hand if necessary
	if(!hand){
		//if not, attempt to load the hand with the appropriate name, and then set it to the current world hand
		hand = GraspitDBGrasp::loadHandFromDBName(QString(mRecord.handName.c_str()));
	}

	//load the model
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		w->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
		mCurrentLoadedModel = NULL;
	}

	//check out the model in the modelList
	GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mRecord.model);
	if(!model){
		DBGA("Cannot recognize the model");
		mStatus = ERROR;
		return;
	}
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
			mStatus = ERROR;
			return;
		}
	}
	//adds the object to the collision detection system
	model->getGraspableBody()->addToIvc();
	//todo: where to dynamic information come from?
	//model->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world so that we can see it
	graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
	//and remember it
	mCurrentLoadedModel = model;

	double spread;
	double tx, ty, tz, qw, qx, qy, qz;
	sscanf(mRecord.argument.c_str(),"{%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}", &tx, &ty, &tz, &qw, &qx, &qy, &qz, &spread);

	transf t(Quaternion(qw, qx, qy, qz), vec3(tx, ty, tz));

	BlindPlannerDlg *dlg = new BlindPlannerDlg(NULL);
	bool loaded = false;
	char surfix[100];
	sprintf(surfix,"%d",mRecord.taskId);

	//start the test according to the parameters retrieved above
	{
		//transf backup(Quaternion::IDENTITY, vec3(0,0,-200.0));//back up 200mm
		//set the hand pose back to original
		hand->setTran(t);

		//reset the hand pose to zero and clear the breakaway flags
		resetHandPoseToZero();

		//shape the spread angle
		if(spread < 0.0001)
			spread = 0.0001;
		hand->forceDOFVal(0,spread);

		//move out of collision and grasp
		//in fact there should not be any collision
		if(!w->noCollision())
		{
			std::cout << "collision found while loading the hand with spread " << spread << std::endl;
			
			if(hand->findInitialContact(100))
				std::cout << "collision resolved" << std::endl;
			else
				std::cout << "collision not resolved" << std::endl;
		}

		hand->autoGrasp(!graspItGUI->useConsole());

		if(!loaded)
		{
			dlg->loadExperienceButton_clicked();
			dlg->loadPoseButton_clicked();
		}

		//the record and the test
		dlg->recordQuality(surfix);
		dlg->recordHandPose(surfix);
		dlg->recordSVMQuality(surfix);
		int total_num = 5;
		for(int test_num = 0; test_num < total_num; ++test_num)
		{
			std::cout << "=====>>>>> Adjustment " << test_num << std::endl;
			dlg->compareButton_clicked();//do this extra one to update the NN list which may not be updated in the
										// following if statement

			//when this grasp is not good, we need to do the adjustment. Criteria: 1) not an identity adjustment, AND 2) epsilon_estimation < 0.5, AND dist >= 10
			if(!dlg->isAnIdentityAdjustment() && dlg->qualityEstimate() < 0.5 )// && dlg->getCurrentDistanceToGEDB() >= 10.0)
			{
				dlg->allInOneAdjustButton_clicked();
			}
			//After one adjustment is done
			dlg->recordQuality(surfix, test_num == total_num - 1); //REAL QUALITY
			//dlg->recordTargetNNQuality(surfix, test_num == total_num - 1); //NN'S QUALITY
			dlg->recordDistance(surfix); //DISTANCE TO THE NN, this distance is the current grasp before the adjustment
			dlg->recordHandPose(surfix, test_num == total_num - 1); //HAND POSE
			dlg->recordSVMQuality(surfix, test_num == total_num - 1);
		}
		dlg->compareButton_clicked();
		dlg->recordDistance(surfix, true);
	}
	mStatus = DONE;
}

//pre compute the perturbations for each grasp
Task* BlindGraspingPrecomputePerturbationTaskFactory::getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
															  db_planner::TaskRecord rec)
{
	return new BlindGraspingPrecomputePerturbationTask(disp, mgr, rec);
}

void BlindGraspingPrecomputePerturbationTask::start()
{

	mStatus = RUNNING;

	World *world = graspItGUI->getIVmgr()->getWorld();
	Hand *hand = world->getCurrentHand();
	if(!hand){
		//if not, attempt to load the hand with the appropriate name, and then set it to the current world hand
		hand = GraspitDBGrasp::loadHandFromDBName(QString(mRecord.handName.c_str()));
	}

	DBGA("task_id: " << mRecord.taskId );
	GraspExperienceBase* mGeb = NULL;
	mGeb = new GraspExperienceBase();
	std::string experiencePath = "C:/project/graspit_hao/experience/experience_primitive.txt";
	std::string posePath = "C:/project/graspit_hao/experience/handpose_primitive.txt";
	if( mGeb->loadGraspExperienceBase(experiencePath) )
	{
		std::cout << "Experience data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Experience data base not loaded" << std::endl;
	}

	if( mGeb->loadHandPoses(posePath) )
	{
		std::cout << "Pose data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Pose data base not loaded" << std::endl;
	}

	GraspExperienceEntry g = mGeb->getEntry(mRecord.argument);
	std::vector<GraspExperienceEntry> gl;
	gl.push_back(g);

	HandAdjust ha;
	//ha.connectToDB(dbURL, dbName, dbPort, dbUserName, dbPassword);
	ha.setDBMgr(mDBMgr);
	ha.init(mGeb);
	ha.setTactileNNList(g, gl); //g does not matter here
	double spread;
	char name[512];
	sprintf(name, "perturb_%s.txt", mRecord.argument.c_str());
	//FILE *fp = fopen(name, "w");
	transf t = ha.getHandAdjustmentONLINE(&spread, name, true);
	//fclose(fp);
	mStatus = DONE;
}
