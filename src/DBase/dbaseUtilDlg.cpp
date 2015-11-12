#include "dbaseUtilDlg.h"

#include <set>
#include <string>
#include <Inventor/sensors/SoTimerSensor.h>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "egPlanner.h"
#include "guidedPlanner.h"
#include "world.h"
#include "searchState.h"
#include "body.h"
#include "robot.h"
#include "grasp.h"
#include "searchEnergy.h"

#include "dbase_grasp.h"
#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "DBPlanner/sql_database_manager.h"

#include "scanSimulator.h"

#include "debug.h"

void DBaseUtilDlg::init()
{
	mDBMgr = graspItGUI->getIVmgr()->getDBMgr();
}

void DBaseUtilDlg::destroy()
{

}

void DBaseUtilDlg::runPlannerOnCGDBButton_clicked()
{
	mHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if(!mDBMgr->ModelList(&mAllModel,db_planner::FilterList::NONE)){
		std::cout << "Model list retrieval failed" << std::endl;
		return;
	}
	FILE *fp = fopen("c:/tester/run.txt","r");
	fscanf(fp,"%d %d",&mCurrentIndex, &mNumGrasps);
	fclose(fp);
	DBGA("opened file\n");
	std::cout << "begin from " << mCurrentIndex << std::endl;

#define BY_NAME

#ifdef BY_NAME
	//get model names from the file
	FILE *fp_name = fopen("c:/tester/obj2.txt", "r");
	char name[30];
	std::vector<string> modelNames;
	while( fscanf(fp_name,"%s\n",&name ) != EOF )
	{
		modelNames.push_back(std::string(name));
	}
#endif

	//orgnize the models to plan
#ifdef BY_TYPE
	std::vector<string> modelType;
	modelType.push_back("laptop");
	modelType.push_back("tool");
	modelType.push_back("bottle");
#endif

#ifdef BY_TYPE
	for(size_t j = 0; j < modelType.size(); ++j)
	{
		for(size_t i = 0; i < mAllModel.size(); ++i)
		{
			if(mAllModel[i]->Tags().find(modelType[j]) != mAllModel[i]->Tags().end()) // find one
				mModelList.push_back(mAllModel[i]);
		}
	}
#endif

#ifdef BY_NAME
	for(size_t j = 0; j < modelNames.size(); ++j)
	{
		for(size_t i = 0; i < mAllModel.size(); ++i)
		{
			if( !strcmp(modelNames[j].c_str(), mAllModel[i]->ModelName().c_str()) )
				mModelList.push_back(mAllModel[i]);
		}

	}
#endif
	//start the first plan by hand
	complete();
}

void DBaseUtilDlg::update()
{
	std::cout << mPlanner->getListSize() << std::endl;
	if(mPlanner->getListSize() < mNumGrasps)
		return;
	storeGrasps();
	//stop the planner in 2 seconds
	int mMaxTime = 2;// this is a trick, maybe not good
	DBGP("begin to leave...");
	mPlanner->setMaxTime(mMaxTime);
}

void DBaseUtilDlg::complete()
{
	mNext = true;
	//schedule the sensor to give us the exit signal in 3 seconds
	//this should give the planner time to finish
	if(mTimerSensor) mTimerSensor->unschedule();
	delete mTimerSensor;
	mTimerSensor = new SoTimerSensor(sensorCB, this);
	mTimerSensor->setInterval( SbTime( 3.0 ));
	mTimerSensor->schedule();
}

void DBaseUtilDlg::sensorCB(void *data, SoSensor*)
{
	//go to next object
	DBaseUtilDlg* dlg = (DBaseUtilDlg*)data;
	if(dlg->readyForNext()){
		dlg->nextObject();
		DBGP("Go to next object");
	}
}

void DBaseUtilDlg::plan()
{
	std::cout << "begin to plan" << std::endl;
	//do not go to the next until this is done
	mNext = false;
	//initialize a planner
	mPlanner = new GuidedPlanner(mHand);
	//set parameters
	mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
	mPlanner->setContactType(CONTACT_PRESET);
	mPlanner->setMaxSteps(70000);

	mHandObjectState = new GraspPlanningState(mHand);
	mHandObjectState->setObject(mCurrentModel->getGraspableBody());
	//mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
	//mHandObjectState->setRefTran(mCurrentModel->getGraspableBody()->getTran());
	mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
	mRefTran = mHand->getTran();//mCurrentModel->getGraspableBody()->getTran();
	mHandObjectState->setRefTran(mRefTran, false);
	mHandObjectState->reset();
	((GuidedPlanner*)mPlanner)->setModelState(mHandObjectState);

	QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(update()));
	QObject::connect(mPlanner,SIGNAL(complete()),this,SLOT(complete()));
	mPlanner->resetPlanner();
	mPlanner->startPlanner();
}

void DBaseUtilDlg::loadNewObject()
{
	std::cout << "num: " << mModelList.size() << std::endl;
	dynamic_cast<GraspitDBModel*>(mModelList[mCurrentIndex])->load(graspItGUI->getIVmgr()->getWorld());
	mCurrentModel = dynamic_cast<GraspitDBModel*>(mModelList[mCurrentIndex]);
	//adds the object to the collision detection system
	mCurrentModel->getGraspableBody()->addToIvc();
	//this adds the object to the graspit world so that we can see it
	graspItGUI->getIVmgr()->getWorld()->addBody(mCurrentModel->getGraspableBody());
	mCurrentModel->getGraspableBody()->setTransparency(0.0);

	////transf current = mCurrentModel->getGraspableBody()->getTran();
	////transf rot = transf(Quaternion(0, 1, 0, 0),vec3(0,0,0));
	////transf transl = transf(Quaternion(), vec3(0,0,200));
	////mCurrentModel->getGraspableBody()->setTran(current * rot * transl);
}

void DBaseUtilDlg::loadAnotherHand()
{
	QString filename;
	QString graspitRoot = QString(getenv("GRASPIT"));
	//filename = graspitRoot + QString("/models/robots/tri_gripper/tri_gripper_7dof.xml");
	filename = graspitRoot + QString("/models/robots/NewBarrett/NewBarrett.xml");
	//filename = graspitRoot + QString("/models/robots/Barrett/Barrett.xml");
	mHand = (Hand*)graspItGUI->getIVmgr()->getWorld()->importRobot(filename);

	transf current = mHand->getBase()->getTran();
	transf rot = transf(Quaternion(0, 1, 0, 0),vec3(0,0,0));
	transf transl = transf(Quaternion(), vec3(0,0,200));
	mHand->setTran(current * rot * transl);
	
}

void DBaseUtilDlg::cleanUp()
{
	delete mHandObjectState;
	delete mPlanner;
	graspItGUI->getIVmgr()->getWorld()->destroyElement(mHand,true);
	if (mCurrentModel) {
		//remove the previously loaded model, and delete it from the world
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentModel->getGraspableBody(), true);
		mCurrentModel = NULL;
	}
}

void DBaseUtilDlg::storeGrasps()
{
	// for storing it into CGDB
	std::vector<db_planner::Grasp*> gpList;

	for(int i = 0; i < mPlanner->getListSize()-1; i+=2){
		gpList.push_back(synthesize(const_cast<GraspPlanningState*>(mPlanner->getGrasp(i)), const_cast<GraspPlanningState*>(mPlanner->getGrasp(i+1))));
	}

	if(mDBMgr)
	{
		mDBMgr->SaveGrasps(gpList);
		for(int i = 0; i < mPlanner->getListSize()/2; ++i){

			delete gpList[i];
		}
		gpList.clear();
	}
	//log
	mLog = fopen("c:/tester/log_run.txt","a");
	fprintf(mLog, "%d: %s\n",mCurrentIndex, mCurrentModel->ModelName().c_str());
	fclose(mLog);
}

GraspitDBGrasp* DBaseUtilDlg::synthesize(GraspPlanningState* pre, GraspPlanningState* fin)
{
	//synthesize a new graspit_db_grasp
	// store it into CGDB
	GraspitDBGrasp* gp;

	gp = new GraspitDBGrasp(mHand);
	gp->SetHandName(GraspitDBGrasp::getHandDBName(mHand).toStdString());
	db_planner::Model* m = mHand->getGrasp()->getObject()->getDBModel();
	gp->SetSourceModel(*m);

	//the pre-grasp's position is not in eigengrasp space, so we must save it in DOF space
	//these are only used for the representation of pregrasp in final grasp's format
	//convert it's tranform to the Quaternion__Translation format
	//make sure you pass it sticky=true, otherwise information is lost in the conversion
	pre->setPositionType(SPACE_COMPLETE,true);
	//we will want to save exact DOF positions, not eigengrasp values
	//again, make sure sticky=true
	pre->setPostureType(POSE_DOF,true);
	pre->setRefTran(mRefTran, true);
	gp->setPreGraspPlanningState(new GraspPlanningState(pre));

	//start analyzing and generate the final grasp
	SearchEnergy* se = new SearchEnergy();
	bool legal;
	double energy;
	se->analyzeState(legal,energy,fin,false);

	//save the qualities
	fin->setEpsilonQuality(se->getEpsQual());
	fin->setVolume(se->getVolQual());

	//Contacts is not copied in copy constructor
	GraspPlanningState * fin_tmp = new GraspPlanningState(fin);
	//the contacts
	std::vector<double> tempArray;
	tempArray.clear();
	for(int i = 0; i < mHand->getGrasp()->getNumContacts(); ++ i){
		Contact * c = mHand->getGrasp()->getContact(i);
		fin_tmp->getContacts()->push_back(c->getPosition());
	}

	gp->setFinalGraspPlanningState(fin_tmp);
	gp->SetSource("OBJECT_RECOGNITION");
	//gp->SetSource("EIGENGRASPS");

	delete se;
	return gp;
}


void DBaseUtilDlg::runPlannerButton_clicked()
{
	char ** argv;
	int argc = (parametersLine->text().count(QString(" ")) + 2);
	argv = (char**) malloc(sizeof(char*) * argc);
	argv[0] = "graspit";
	for(int i = 1; i < argc; ++i){
		argv[i] = (char*) malloc(sizeof(char) * 100);
		strcpy(argv[i],(parametersLine->text().section(" ",i-1,i-1).toStdString().c_str()));
		std::cout << argv[i] << std::endl;
	}
	DBaseBatchPlanner *dbp = new DBaseBatchPlanner(graspItGUI->getIVmgr(), graspItGUI);
	dbp->processArguments(argc, argv);
	dbp->startPlanner();
}

void DBaseUtilDlg::takeScansButton_clicked()
{
	FILE *fp;
	fp = fopen("c:\\test_ranger.txt","w");
	ScanSimulator sim;

	//TODO: need to be adjusted to SwissRanger
	sim.setOptics(-60.0 , 60.0 , 144 ,
		-60.0 , 60.0 , 176);
	sim.setType(ScanSimulator::SCANNER_COORDINATES);

	std::vector<position> cloud;
	std::vector<RawScanPoint> rawData;


	transf handLoc = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran();

	double t[4][4];
	handLoc.toColMajorMatrix(t);
	vec3 dir (t[2][0],t[2][1],t[2][2]);
	vec3 up(t[0][0],t[0][1],t[0][2]);
	vec3 loc(handLoc.translation().x() + 975.0*t[2][0],
		handLoc.translation().y() + 975.0*t[2][1],
		handLoc.translation().z() + 975.0*t[2][2]);

	std::cout << "dir: " << dir.x() << " " << dir.y() << " " << dir.z() << "\n"  << 
		"up: " << up.x() << " " << up.y() << " " << up.z() << "\n" <<
		"loc: " << loc.x() << " " << loc.y() << " " << loc.z() << std::endl;

	//looking towards the object
	sim.setPosition(loc,dir,up);

	cloud.clear();
	rawData.clear();

	sim.scan(&cloud, &rawData);

	//writeCloudToFile(i, j, cloud);
	for (int k=0; k<(int)cloud.size(); k++) {
		fprintf(fp,"%f %f %f\n",cloud[k].x(), cloud[k].y(), cloud[k].z());
	}
	fclose(fp);
}
