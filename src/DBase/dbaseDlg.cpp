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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: dbaseDlg.cpp,v 1.24 2009/10/01 00:11:42 cmatei Exp $
//
//######################################################################

/*! \file 
\brief Defines the %DBaseDlg class
*/
#include "dbaseDlg.h"
#include "searchEnergy.h"
#include <utility>
#include <QFileDialog>
#include <QDir>
#include <QComboBox>
#include <fstream>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "world.h"
#include "body.h"
#include "searchState.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "mainWindow.h"
#include "matvec3D.h"
#include "dbaseUtilDlg.h"
#include "SensorInterface.h"
#include "scan_manager.h"
#include "scanSimulator.h"
#include "bBox.h"
#include "quality.h"

#define RENDER true

#ifdef SEMANTIC_PLANNER_ENABLED
#include "SemanticPlanner/semanticPlannerDlg.h"
#endif

#include "DBPlanner/sql_database_manager.h"
#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "dbasePlannerDlg.h"
#include "graspCaptureDlg.h"

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

#include "../ui/taskControlDlg.h"

/*! Initializes the dialog and also gets the one and only manager from the
GraspitGUI. If this manager is already set, it also loads the model 
list from the database and initializes it.
*/
void DBaseDlg::init()
{
	mModelList.clear();
	mGraspList.clear();
	mGraspList.clear();
	browserGroup->setEnabled(FALSE);
	graspsGroup->setEnabled(FALSE);
	mDBMgr = graspItGUI->getIVmgr()->getDBMgr();
	if (mDBMgr) {
		getModelList();
	}

#ifdef JARED_AUTO_RUNNING
    DBGA("JARED_AUTO_RUNNING is set");
    connectButton_clicked();

    classesComboBox->setCurrentIndex(26);
    modelsComboBox->setCurrentIndex(0);

    loadModelButton_clicked();
    DBGA("Db loaded!");

    typesComboBox->setCurrentIndex(24);
    loadGraspButton_clicked();

    //setWindowState(windowState() ^ Qt::WindowMinimized);
#endif
}

void DBaseDlg::destroy()
{
	//we do not delete the dbmgr because it is now set in the ivmgr for the rest of
	//graspit to use. The ivmgr deletes it on its exit.
	delete mModelScene;
}

void DBaseDlg::exitButton_clicked(){
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
	}
	//delete and release all the memories occupied by the grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();
	//delete and release all the memories occupied by the models
	deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
    QDialog::accept();
}

void DBaseDlg::getModelList()
{
	//clear the modelList
	deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
	mModelList.clear();
	//load the models from database manager
	if(!mDBMgr->ModelList(&mModelList,db_planner::FilterList::NONE)){
		DBGA("Model list retrieval failed");
		return;
	}
	//display the retrieved models, put their names into the combobox
	displayModelList();
	//check that there are valid number of models
	if(!mModelList.empty()){
		browserGroup->setEnabled(TRUE);
		connectButton->setEnabled(FALSE);
	}
	std::vector<std::string>graspTypes;
	if(!mDBMgr->GraspTypeList(&graspTypes)){
		DBGA("Grasp Types not loaded");
		return;
	}
	//display the types
	displayGraspTypeList(graspTypes);
}

/*! Deletes the old connection to the database and creates a new one based
on the current settings in the dialog box. The new connection is then
set as the one an only Database Manager that the rest of GraspIt had
acces to.

After connecting, it also reads the model list from the database and
displays it.
*/
void DBaseDlg::connectButton_clicked()
{
    delete mDBMgr;
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	//h = setHand(h, i);
	graspItGUI->getIVmgr()->getWorld()->setCurrentHand(h);
	//Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	mDBMgr = new db_planner::SqlDatabaseManager(hostLineEdit->text().toStdString(),
		atoi(portLineEdit->text().latin1()),
		usernameLineEdit->text().toStdString(),
		passwordLineEdit->text().toStdString(),
		databaseLineEdit->text().toStdString(),
		new GraspitDBModelAllocator(),
		new GraspitDBGraspAllocator(h));
	if (mDBMgr->isConnected()) {
		//delete and release all the memories occupied by the models
		deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
		mCurrentLoadedModel = NULL;	
		getModelList();
	}
	else {
		DBGA("DBase Browser: Connection failed");
		delete mDBMgr;
		mDBMgr = NULL;
	}
	graspItGUI->getIVmgr()->setDBMgr(mDBMgr);
    DBGA("==========================> dbmgr set!")
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), true);
	}
	//	reRunButton_clicked();
	//delete and release all the memories occupied by the grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();
	//}
}

PROF_DECLARE(GET_GRASPS);
PROF_DECLARE(GET_GRASPS_CALL);

void DBaseDlg::loadGraspButton_clicked(){
	PROF_RESET_ALL;
	PROF_START_TIMER(GET_GRASPS);
	//get the current hand and check its validity
	Hand *hand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if (!hand) {
		DBGA("Load and select a hand before viewing grasps!");
		return;
	}
	//check the currently loaded model
	if(!mCurrentLoadedModel){
		DBGA("Load model first!");
		return;
	}
	//clear the previously loaded grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();
	mCurrentFrame = 0;
	//get new grasps from database manager
	PROF_START_TIMER(GET_GRASPS_CALL);
	if(!mDBMgr->GetGrasps(*mCurrentLoadedModel,GraspitDBGrasp::getHandDBName(hand).toStdString(), &mGraspList)){
		DBGA("Load grasps failed");
		mGraspList.clear();
		return;
	}
	PROF_STOP_TIMER(GET_GRASPS_CALL);

    std::stringstream out;
    out << typesComboBox->currentIndex();
    DBGA("Type = " + out.str());
	for(std::vector<db_planner::Grasp*>::iterator it = mGraspList.begin(); it != mGraspList.end(); ){
		if( QString((*it)->GetSource().c_str()) == typesComboBox->currentText() ||
			typesComboBox->currentText() == "ALL"){
				++it;
		}
		else{
			delete (*it);
			mGraspList.erase(it);
		}
	}
	//set corresponding indices and show the grasp
	QString numTotal, numCurrent;
	numTotal.setNum(mGraspList.size());
	if(!mGraspList.empty()){
		numCurrent.setNum(mCurrentFrame + 1);
		graspsGroup->setEnabled(TRUE);
		showGrasp(0);
	} else{
		numCurrent.setNum(0);
		graspsGroup->setEnabled(FALSE);
	}
	graspIndexLabel->setText(numCurrent + "/" + numTotal);
	PROF_STOP_TIMER(GET_GRASPS);
	PROF_PRINT_ALL;
}

void DBaseDlg::loadModelButton_clicked(){
	if (mCurrentLoadedModel) {
		//remove the previously loaded model, but don't delete it
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
		mCurrentLoadedModel = NULL;
	}
	if(mModelList.empty()){
		DBGA("No model loaded...");
		return;
	}

	//check out the model in the modelList
    GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]);
    DBGA("Model text = " + modelsComboBox->currentText().toStdString());
    std::stringstream out;

    out << modelsComboBox->currentIndex() << "x" << classesComboBox->currentIndex();
    DBGA("Model id = " + out.str());
    if(!model){
		DBGA("Cannot recognize the model type");
		return;
	}
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
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
	mCurrentLoadedModel = model;	//model->getGraspableBody()->showAxes(false);
	model->getGraspableBody()->setTransparency(0.0);
	graspsGroup->setEnabled(FALSE);

	//delete the previously loaded grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	mGraspList.clear();	
	//initialize the grasp information for the new model
	initializeGraspInfo();

	position cog = model->getGraspableBody()->getCoG();
	std::cout << "cog: " << cog[0] << ", " << cog[1] << ", " << cog[2] << std::endl;
}

// go to see the next grasp
void DBaseDlg::nextGraspButton_clicked(){
	nextGrasp();
}

// go back to the previous grasp
void DBaseDlg::previousGraspButton_clicked(){
	previousGrasp();
}

// pop up the new window for the grasp planner
void DBaseDlg::plannerButton_clicked(){
	//check the existance of database manager
	if(!mDBMgr){
		DBGA("No dbase manager.");
		return;
	}
	//check the hand
	Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if(!h){
		DBGA("No hand found currently");
		return;
	}
	//check the current model
	if(!mCurrentLoadedModel){
		DBGA("No object loaded");
		return;
	}
	//instantialize a new dialogue of type DBasePlannerDlg and pop it up
	DBasePlannerDlg *dlg = new DBasePlannerDlg(this, mDBMgr, mCurrentLoadedModel, h);
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show();

	//delete the grasps loaded, release the memories, and reset the grasp information
	graspsGroup->setEnabled(FALSE);
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
	initializeGraspInfo();
}

//a shortcut for the GWS display
void DBaseDlg::createGWSButton_clicked(){
	graspItGUI->getMainWindow()->graspCreateProjection();
}

//go to the utility dialog
void DBaseDlg::utilButton_clicked(){
	DBaseUtilDlg *dlg = new DBaseUtilDlg();
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show();
}

void DBaseDlg::semanticPlannerButton_clicked(){
#ifdef SEMANTIC_PLANNER_ENABLED
	SemanticPlannerDlg *dlg = new SemanticPlannerDlg();
	dlg->setCurrentLoadedModel(mCurrentLoadedModel);
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show();
#endif
}

//trigger when the selection in the model list combo box is changed, display the corresponding new image
void DBaseDlg::modelChanged(){
	if(inModelConstruction) return;
	QString psbModelThumbPath = QString( mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]->ThumbnailPath().c_str() );
	if(mModelScene) delete mModelScene;
	mModelScene = new QGraphicsScene;
	QPixmap lPixmap;
	lPixmap.load(psbModelThumbPath);
	//resize so that it will fit in window
	if (lPixmap.width() > 160) {
		lPixmap = lPixmap.scaledToWidth(160);
	}
	if (lPixmap.height() > 120) {
		lPixmap = lPixmap.scaledToHeight(120);
	}
	mModelScene->addPixmap(lPixmap);
	this->objectGraph->setScene(mModelScene);
	this->objectGraph->show();
}

//trigger when the grasp type is changed between pregrasp and final grasp 
void DBaseDlg::graspTypeChanged(){
	showGrasp(mCurrentFrame);
}

//trigger when the model class is changed, reconstruct the model list combo box
void DBaseDlg::classChanged(){
	inModelConstruction = true;
	modelsComboBox->clear();
	for(size_t i = 0; i < mModelList.size(); ++i){
		if(mModelList[i]->Tags().find(classesComboBox->currentText().toStdString()) != mModelList[i]->Tags().end()
			|| classesComboBox->currentText() == "ALL")
			modelsComboBox->addItem(mModelList[i]->ModelName().c_str());
	}
	inModelConstruction = false;
	modelChanged();
}

//synthesize the model list combo box
void DBaseDlg::displayModelList(){
	std::set<string> tags;
	mModelMap.clear();
	for(int i = 0; i < (int)mModelList.size(); ++i){
		modelsComboBox->insertItem(QString(mModelList[i]->ModelName().c_str()));
		tags.insert(mModelList[i]->Tags().begin(), mModelList[i]->Tags().end());
		mModelMap.insert(std::make_pair<std::string, int>(mModelList[i]->ModelName(), i));
	}
	classesComboBox->clear();
	classesComboBox->insertItem("ALL");
	for(std::set<string>::iterator i = tags.begin(); i != tags.end(); ++i){
		classesComboBox->insertItem(QString((*i).c_str()));
	}
}

void DBaseDlg::displayGraspTypeList(std::vector<std::string> list){
	typesComboBox->clear();
	typesComboBox->insertItem("ALL");
	for(size_t i = 0; i < list.size(); ++i){
		typesComboBox->insertItem(QString(list[i].c_str()));
	}
}

//core routine that shows the i-th loaded grasp
void DBaseDlg::showGrasp(int i)
{
	//gotoGrasp(i);
	//return;

	if (mGraspList.empty()) return;
	assert( i>=0 && i < (int)mGraspList.size() );
	//put the model in correct place
	mCurrentLoadedModel->getGraspableBody()->setTran(transf::IDENTITY);
	//show the pregrasp or final grasp
	if(showPreGraspRadioButton->isChecked()){
		if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState())//NULL grasp, return
			return;
		static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState()->execute();
		if(!resolveCollision())
		{
			DBGA("cannot solve collision");
		}
	}
	else{
		if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState())//NULL grasp, return
			return;
		static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState()->execute();
		if(!resolveCollision())
		{
			DBGA("cannot solve collision");
		}
		if(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->isA("Barrett")){
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(RENDER);
		}
	}
	//update the world and grasp information
	//	DBGA("Find contacts");
	graspItGUI->getIVmgr()->getWorld()->findAllContacts();
	//	DBGA("Update grasps");
	graspItGUI->getIVmgr()->getWorld()->updateGrasps();

	//update sensor and recolor it
	World * w = graspItGUI->getIVmgr()->getWorld();
	double maxVal = 0;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		w->getSensor(sensorInd)->updateSensorModel();
		double v = w->getSensor(sensorInd)->getNormalForce() ;
		if(v > maxVal)
			maxVal = v;
	}
	//no sensor has value
	if(maxVal < 1e-6)
	{
		maxVal = 1e10;
	}
	//set the color of each sensor
	//std::cout << this << std::endl;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		w->getSensor(sensorInd)->setColor(maxVal);
	}

	//	for(int i = 0; i < w->getCurrentHand()->getNumChains(); ++i)
	//	{
	//		KinematicChain* kc = w->getCurrentHand()->getChain(i);
	//		for(int j = 0; j < kc->getNumLinks(); ++j)
	//		{
	//			Link* l = kc->getLink(j);
	//			if(l->getType() == SENSORLINK)
	//			{
	//				std::cout << "update sensor" << std::endl;
	//				((SensorLink*)l)->updateSensors();
	//			}
	//		}
	//	}

	mCurrentFrame = i;
	updateGraspInfo();
	DBGA("Show grasp done");
}
/*gotoGrasp will move the hand to the grasp position and orientation,
but the hand will back up 1000mm from the object.  This is used for
simulating the scanner inside GraspIt!  This is not ideal!*/
void DBaseDlg::gotoGrasp(int i)
{
	if (mGraspList.empty()) return;
	assert( i>=0 && i < (int)mGraspList.size() );
	//put the model in correct place
	mCurrentLoadedModel->getGraspableBody()->setTran(transf::IDENTITY);
	if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState())//NULL grasp, return
		return;
	static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState()->execute();

	//turn off the axis shown
	mCurrentLoadedModel->getGraspableBody()->showAxes(false);
	//---------------------move back
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	transf handLoc = h->getBase()->getTran();

	double t[4][4];
	handLoc.toColMajorMatrix(t);
	vec3 dir (t[2][0],t[2][1],t[2][2]);
	vec3 loc(handLoc.translation().x() - 1000.0*t[2][0],
		handLoc.translation().y() - 1000.0*t[2][1],
		handLoc.translation().z() - 1000.0*t[2][2]);

	std::cout << "before: " << handLoc.translation().x() << " " << handLoc.translation().y() << " " << handLoc.translation().z() << std::endl;
	transf newHandLoc(handLoc.rotation(),loc);
	h->setTran(newHandLoc);
	//-----------------------

	//---------------------open it
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(RENDER,-1);

	//update the world and grasp information
	graspItGUI->getIVmgr()->getWorld()->updateGrasps();
	mCurrentFrame = i;
}

//go to see the next grasp and show the corresponding image
void DBaseDlg::nextGrasp() {
	if (mGraspList.empty()) return;
	mCurrentFrame ++;
	if (mCurrentFrame == mGraspList.size()) mCurrentFrame = 0;
	showGrasp(mCurrentFrame);
}

//go to see the previous grasp and show the corresponding image
void DBaseDlg::previousGrasp() {
	if (mGraspList.empty()) return;
	mCurrentFrame --;
	if (mCurrentFrame < 0) mCurrentFrame = mGraspList.size() - 1;
	showGrasp(mCurrentFrame);
}

//go to a specific grasp
void DBaseDlg::specificGoButton_clicked(){
	mCurrentFrame = indexEdit->text().toInt();
	showGrasp(mCurrentFrame);
	std::cout << "viewing the " << mCurrentFrame << " grasp" << std::endl;
}

//update the information of current grasp, including indices, epsilon qualities, and volume qualities
void DBaseDlg::updateGraspInfo(){
	QString numTotal, numCurrent;
	numTotal.setNum(mGraspList.size());
	if(!mGraspList.empty())
		numCurrent.setNum(mCurrentFrame + 1);
	else
		numCurrent.setNum(0);
	graspIndexLabel->setText(numCurrent + "/" + numTotal);

	QString eq, vq;
	eq.setNum(mGraspList[mCurrentFrame]->EpsilonQuality());
	vq.setNum(mGraspList[mCurrentFrame]->VolumeQuality());

	epsilonQualityLabel->setText(QString("Epsilon Quality: " + eq));
	volumeQualityLabel->setText(QString("Volume Quality: " + vq));

	std::cout << mGraspList[mCurrentFrame]->GraspId() << std::endl;
}

//reset the grasp information displayed
void DBaseDlg::initializeGraspInfo(){
	graspIndexLabel->setText("0/0");
	epsilonQualityLabel->setText(QString("Epsilon Quality: 0.0"));
	volumeQualityLabel->setText(QString("Volume Quality: 0.0"));
}

//helper function that deletes the vector of type vectorType, but treating every elements as type treatAsType
template <class vectorType, class treatAsType>
inline void DBaseDlg::deleteVectorElements(std::vector<vectorType>& v){
	for(size_t i = 0; i < v.size(); ++i){
		delete (treatAsType)v[i];
	}
	v.clear();
}

Hand * setHand(Hand * currentHand, int handInd){
	std::vector<QString> handVector;
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/Barrett/Barrett.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/Barrett/Barrett.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/Barrett/Barrett.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/HumanHand/HumanHand20DOF.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/pr2_gripper/pr2_gripper.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/McHand/McHand.xml"));
	handVector.push_back(QString(getenv("GRASPIT")) + QString("/models/robots/cobra_gripper/cobra_gripper.xml"));
	// if a hand exists
	if(currentHand){
		currentHand->getWorld()->removeElementFromSceneGraph(currentHand);
		currentHand->getWorld()->removeRobot(currentHand);
	}
	currentHand = static_cast<Hand *>(graspItGUI->getIVmgr()->getWorld()->importRobot(handVector[handInd - 1]));
	if(handInd <= 3){
		//set the whole hand to the right material
		int matIdx;
		if(handInd == 1)
			matIdx = currentHand->getWorld()->getMaterialIdx("rubber");
		if(handInd == 2)
			matIdx = currentHand->getWorld()->getMaterialIdx("wood");
		if(handInd == 3)
			matIdx = currentHand->getWorld()->getMaterialIdx("plastic");

		for(int kind = 0; kind < currentHand->getNumChains(); kind++)
			for(int lind = 0; lind < currentHand->getChain(kind)->getNumLinks(); lind++)
				currentHand->getChain(kind)->getLink(lind)->setMaterial(matIdx);
		currentHand->getPalm()->setMaterial(matIdx);		
	}
	return currentHand;

}

void DBaseDlg::recomputeGraspQualitiesInCGDB(){

	SearchEnergy searchEnergy;
	searchEnergy.setType(ENERGY_STRICT_AUTOGRASP);
	searchEnergy.disableRendering(false);
	std::vector<db_planner::Grasp*> graspList;
	FILE * fp, *fp_start;
	int start;
	fp_start = fopen("start.txt","r");
	fscanf(fp_start, "%d", &start);
	std::cout << "Start at: " << start << std::endl;

	for(size_t i = start; i < modelsComboBox->count(); ++i){ // for each model 0
		modelsComboBox->setCurrentIndex(i);
		loadModelButton_clicked();
		loadGraspButton_clicked();
		bool legal;
		fp = fopen("rerun.txt","a");
		for(size_t j = 0; j < mGraspList.size(); ++j){ // for each grasp
			mCurrentLoadedModel->getGraspableBody()->setTran(transf::IDENTITY);
			GraspPlanningState state(static_cast<GraspitDBGrasp*>(mGraspList[j])->getPreGraspPlanningState());
			state.setObject(mCurrentLoadedModel->getGraspableBody());

			double energy;
			searchEnergy.analyzeState(legal, energy, &state, false);

			Hand *currentHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
			GraspPlanningState newState(currentHand);
			newState.setRefTran(transf::IDENTITY);
			newState.setObject(mCurrentLoadedModel->getGraspableBody());
			newState.setPostureType(POSE_DOF, false);
			newState.setPositionType(SPACE_COMPLETE, false);

			newState.saveCurrentHandState();

			//collect the contacts
			newState.getContacts()->clear();
			if (legal) {
				newState.setEpsilonQuality(searchEnergy.getEpsQual());
			} else {
				DBGA("Illegal pre-grasp");
				newState.setEpsilonQuality(-2.0);

			}
			newState.setVolume(searchEnergy.getVolQual());
			if (legal) {
				for (int c=0; c<currentHand->getGrasp()->getNumContacts(); c++) {
					newState.getContacts()->push_back
						(currentHand->getGrasp()->getContact(c)->getPosition() );
				}
			}

			//------------- for database storage -----------------------
			//synthesize the GraspitDBGrasp
			db_planner::Grasp* grasp = new GraspitDBGrasp(currentHand);
			grasp->SetGraspId(mGraspList[j]->GraspId());
			grasp->SetEpsilonQuality(newState.getEpsilonQuality());
			grasp->SetVolumeQuality(newState.getVolume());

			std::vector<double> tempArray;
			//the posture
			for(int dofi = 0; dofi < newState.getPosture()->getNumVariables(); ++dofi){
				tempArray.push_back(newState.getPosture()->getVariable(dofi)->getValue());
			}
			//grasp->SetPregraspJoints(tempArray);
			grasp->SetFinalgraspJoints(tempArray);

			//the position
			tempArray.clear();
			for(int posi = 0; posi < newState.getPosition()->getNumVariables(); ++posi){
				tempArray.push_back(newState.getPosition()->getVariable(posi)->getValue());
			}
			//grasp->SetPregraspPosition(tempArray);
			grasp->SetFinalgraspPosition(tempArray);

			//the contacts
			std::list<position> *contacts;
			tempArray.clear();
			contacts = newState.getContacts();
			std::list<position>::iterator itContact;
			for(itContact = contacts->begin(); itContact != contacts->end();
				++itContact){
					tempArray.push_back((*itContact).x());
					tempArray.push_back((*itContact).y());
					tempArray.push_back((*itContact).z());
			}
			grasp->SetContacts(tempArray);
			//store this
			graspList.push_back(grasp);
			//modify database
			if(graspList.empty()) continue;
			if(! mDBMgr->UpdateGrasp(graspList, fp))
				std::cout << "update unsuccessfully\n";
			deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(graspList);
			if(!legal){
				std::cout<<"Illegal Grasp- Object: " << i << " Grasp Num: "<<j << std::endl;
			}

		}
		// one model is done
		fprintf(fp, "%i %s\n", i, modelsComboBox->text(i).toStdString().c_str());
		fclose(fp);
	}
}


//re-run the database
//loop through all the grasps on objects within ranger start and end and do something at each time
void DBaseDlg::reRunButton_clicked(){
	Hand *currentHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	FILE * fp, *fresult, *flog, *flog_joint, *f_start_end, *f_object_name;
	int start, end;

	//filtering based on the grasp type
	std::cout << typesComboBox->findText(QString("STABILITY_LEARNING")) << std::endl;
	typesComboBox->setCurrentIndex( typesComboBox->findText(QString("STABILITY_LEARNING")) );

	/*
	The following uses the indices inside start_end.txt and loop through them in the model_list
	This is the most regular version
	*/
#define REGULAR_DB
#ifdef REGULAR_DB
	f_start_end = fopen("start_end.txt","r");
	while(fscanf(f_start_end,"%d %d\n",&start, &end) != EOF)
	{
		//writeOutExp(start, end);
		writeOutTactileFromCGDB(start, end);
	}
	fclose(f_start_end);
#endif

//#define STAND_ALONE
#ifdef STAND_ALONE
	/*
	The following code runs through the objects in object.txt and uses TaskControlDlg to output the tacitle experiences
	Stand-alone version of implementation
	*/
	TaskControlDlg* tcg = NULL;
	f_object_name = fopen("object.txt", "r");
	char name[100];
	while(fscanf(f_object_name, "%s\n", name) > 0) // negative value is returned when no matching found
	{
		std::cout << name << std::endl;
		for(size_t i = 0; i < mModelList.size(); ++i)
		{
			if( !strcmp(mModelList[i]->ModelName().c_str(), name) )
			{
				//writeOutExp(i,i+1);
				std::cout << "working on" << mModelList[i]->ModelName().c_str() << " expected: " << name << std::endl;
				modelsComboBox->setCurrentIndex(i);
				loadModelButton_clicked();
				delete(tcg);
				tcg = new TaskControlDlg(this);
				tcg->goButton_clicked();
			}
		}
	}
	fclose(f_object_name);
#endif

}

void DBaseDlg::writeOutTactileFromCGDB(int start, int end)
{
	std::cout << "start: " << start << " end: " << end << std::endl;

	FILE * fp, *flog_handpose, *flog_tactile;

	//for the number characters
	char num[20];

	for(size_t i = start; i < end; ++i){
		//load a model
		modelsComboBox->setCurrentIndex(i);
		loadModelButton_clicked();
		loadGraspButton_clicked();
		for(size_t j = 0; j < mGraspList.size(); ++j){ // for each grasp
			if(mGraspList[j]->GetGraspTactileContactList().size() == 0 || mGraspList[j]->EpsilonQuality() < 0.1)
				continue;
			flog_handpose = fopen("handpose_cgdb.txt", "a");
			std::vector<double> handpose = mGraspList[j]->GetFinalgraspPosition();
			fprintf(flog_handpose, "%s_%d %lf %lf %lf %lf %lf %lf %lf ", mCurrentLoadedModel->ModelName().c_str(), j, handpose[0], handpose[1], handpose[2],
				handpose[3], handpose[4], handpose[5], handpose[6]);


			//spread angle
			std::vector<double> jnts;
			jnts = mGraspList[j]->GetFinalgraspJoints();
			fprintf(flog_handpose, "%lf\n", jnts[0]);

			fclose(flog_handpose);

			flog_tactile = fopen("experience_cgdb.txt", "a");
			std::vector<double> tactile_exp = mGraspList[j]->GetGraspTactileContactList();
			fprintf(flog_tactile, "%s_%d %lf %lf %d ", mCurrentLoadedModel->ModelName().c_str(), j, mGraspList[j]->EpsilonQuality(), mGraspList[j]->VolumeQuality(),
				tactile_exp.size()/8);//every entry has 8 numbers
			for(size_t k = 0; k < tactile_exp.size(); ++k)
			{
				fprintf(flog_tactile, "%lf ", tactile_exp[k]);
			}
			fprintf(flog_tactile,"\n");
			fclose(flog_tactile);
		}//one grasp is finished

	}//one object is finished

}

void DBaseDlg::writeOutExp(int start, int end)
{
	std::cout << "start: " << start << " end: " << end << std::endl;

	FILE * fp, *fp_start, *fp_end, *fresult, *flog_qualities, *flog_joint, *flog_contact, *flog_dof, *flog_sensor, *flog_dexterity;

	//for the number characters
	char num[20];

	for(size_t i = start; i < end; ++i){
		//load a model
		modelsComboBox->setCurrentIndex(i);
		loadModelButton_clicked();
		loadGraspButton_clicked();
		for(size_t j = 0; j < mGraspList.size(); ++j){ // for each grasp

#define TACTILE_OUTPUT
#ifdef TACTILE_OUTPUT
			//generate file name
			std::string fname("C:\\data\\new_barrett_150000_coke_gillette_canteen_uniform_sampling_tactile_experience\\tactile_sensor_reading\\");
			fname.append(mCurrentLoadedModel->ModelName());
			fname.append("_");
			sprintf(num, "%d", j);
			//itoa(j,num,10);
			fname.append(num);
			fname.append(".txt");

			fresult = fopen(fname.c_str(),"w");
			QTextStream qts(fresult);
			writeOutTactileInfo(qts);
			fclose(fresult);
#endif

#define QUALITIES_OUTPUT
#ifdef QUALITIES_OUTPUT
			flog_qualities = fopen("qualities.txt","a");
			while(!flog_qualities)
			{
				std::cout << "cannot open, try it again\n";
				flog_qualities = fopen("qualities.txt", "a");
			}
			/*
			//record the grasp info inside log file
			fprintf(flog_qualities,"%s_%d %lf %lf\n",mCurrentLoadedModel->ModelName().c_str(), j,
			mGraspList[j]->EpsilonQuality(), mGraspList[j]->VolumeQuality());
			*/

			//record the grasp info
			// if there is no collision, then begin computation
			Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
			if(mEpsQual){
				delete mEpsQual;
			}
			mEpsQual = new QualEpsilon( h->getGrasp(), ("Examine_dlg_qm"),"L1 Norm");
			if(mVolQual){
				delete mVolQual;
            }
			mVolQual = new QualVolume( h->getGrasp(), ("Examine_dlg_qm"),"L1 Norm");
			h->getWorld()->findAllContacts();
			h->getWorld()->updateGrasps();

			double eq = mEpsQual->evaluate();
			double vq = mVolQual->evaluate();
			fprintf(flog_qualities,"%s_%d %lf %lf\n",mCurrentLoadedModel->ModelName().c_str(), j,
				eq, vq);
			fclose(flog_qualities);
#endif

#define JOINT_OUTPUT
#ifdef JOINT_OUTPUT
			//log file is the result file as well
			flog_joint = fopen("joints.txt","a");
			//for outputing the joint angles
			//std::vector<double> jnts = mGraspList[j]->GetFinalgraspJoints();

			std::vector<double> jnts;
			jnts.resize(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getNumJoints());
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getJointValues(&jnts[0]);

			fprintf(flog_joint,"%s_%d ",  mCurrentLoadedModel->ModelName().c_str(), j);
			for(int jIndex = 0; jIndex < jnts.size(); ++jIndex){
				fprintf(flog_joint,"%lf ", jnts[jIndex]);
			}
			fprintf(flog_joint,"\n");
			fclose(flog_joint);
#endif

			//#define DOF_OUTPUT
#ifdef DOF_OUTPUT
			flog_dof = fopen("dofs.txt", "a");

			std::vector<double> dofs;
			dofs.resize(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getNumDOF());
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getDOFVals(&dofs[0]);

			fprintf(flog_dof, "%s_%d ", mCurrentLoadedModel->ModelName().c_str(), j);
			for(int dofIndex = 0; dofIndex < dofs.size(); ++dofIndex){
				fprintf(flog_dof,"%lf ", dofs[dofIndex]);
			}
			fprintf(flog_dof,"\n");
			fclose(flog_dof);
#endif

#define SENSOR_LOCATION
#ifdef SENSOR_LOCATION
			fname = std::string("C:\\data\\new_barrett_150000_coke_gillette_canteen_uniform_sampling_tactile_experience\\tactile_sensor_location\\");
			fname.append(mCurrentLoadedModel->ModelName());
			fname.append("_");
			sprintf(num, "%d", j);
			//itoa(j,num,10);
			fname.append(num);
			fname.append(".txt");
			flog_sensor = fopen(fname.c_str(),"w");

			World * w = graspItGUI->getIVmgr()->getWorld();
			for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
				//w->getSensor(sensorInd)->updateSensorModel();
				transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
				transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
				transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

				fprintf(flog_sensor,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",sensorInHand.translation().x(), sensorInHand.translation().y(), sensorInHand.translation().z(),
					sensorInHand.rotation().w, sensorInHand.rotation().x, sensorInHand.rotation().y, sensorInHand.rotation().z);
			}
			fclose(flog_sensor);
#endif

			//#define CONTACT_OUTPUT
#ifdef CONTACT_OUTPUT
			std::string fname2("c:\\contact_output\\");
			fname2.append(mCurrentLoadedModel->ModelName());
			fname2.append("_");
			sprintf(num, "%d", j);
			//itoa(j,num,10);
			fname2.append(num);
			fname2.append(".txt");
			flog_contact = fopen(fname2.c_str(),"w");

			if(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getNumContacts() == 0) return;
			Body *b = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getContact(0)->getBody2();
			std::fstream file;
			file.open(fname2.c_str(), std::iostream::app);
			std::list<Contact *>::iterator cp;
			std::list<Contact *> contactList;
			contactList = b->getContacts();
			for(cp = contactList.begin(); cp != contactList.end(); ++cp){
				Contact* ct = *(cp);
				file << 
					//mGraspList[mCurrentFrame]->GetSource() << "_" << mCurrentFrame << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().translation().x() << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().translation().y() << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().translation().z() << "\t" <<

					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().rotation().w << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().rotation().x << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().rotation().y << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getBase()->getTran().rotation().z << "\t" <<

					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getCoG().x() << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getCoG().y() << "\t" <<
					graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->getCoG().z() << "\t" <<

					mGraspList[j]->EpsilonQuality() << "\t" <<
					mGraspList[j]->VolumeQuality() << "\t" <<

					ct->getFrame().translation().x() << "\t" <<
					ct->getFrame().translation().y() << "\t" <<
					ct->getFrame().translation().z() << "\t" <<

					ct->getNormal().x() << "\t" <<
					ct->getNormal().y() << "\t" <<
					ct->getNormal().z() << "\t" <<

					ct->getCof() << "\n";
			}
			file << "-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\n";
			file.close();
#endif

			nextGrasp();
		}// one model is done
		fp = fopen("rerun.txt","a");
		fprintf(fp, "%d\n", i);
		fclose(fp);
	}
}

//for exporting tactile information
void DBaseDlg::writeOutTactileInfo(QTextStream & qts){
	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		qts << sensorInd << " ";
		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);
	}
}

void DBaseDlg::collectScans(std::string filename_suffix)
{
#define BROUGHT_UP_NEW_
#ifdef BROUGHT_UP_NEW_
	//std::string fn("c:\\output_data\\mug50x_1.25_scans\\"), fn_model, fn_raw;
	//char modelNumber[20], graspNumber[20];
	//itoa(modelsComboBox->currentIndex(),modelNumber,10);
	//itoa(mCurrentFrame,graspNumber,10);
	//fn.append(modelNumber);
	//fn.append("_");
	//fn.append(graspNumber);

	std::string fn_model, fn_raw;
	fn_model = filename_suffix;
	fn_model.append("_m.txt");
	fn_raw = filename_suffix;
	fn_raw.append("_r.txt");

	ScanManager sm;
	//this defines half the fov
	double fov_horizontal = 19.8;
	double fov_vertical = 23.75;
	sm.setOptics(fov_horizontal, fov_vertical, 144, 176);

	transf pos = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	double t[4][4];
	pos.toColMajorMatrix(t);
	vec3 dir (t[2][0],t[2][1],t[2][2]);
	//up direction is barrett's y axis
	vec3 up(t[1][0],t[1][1],t[1][2]);
	vec3 loc;

	// look for bounding box
	std::vector<BoundingBox> bvs;
	vec3 center, corner;
	for(int i = 5000; i > 0; i -= 5) //one step back
	{
		vec3 tmp(pos.translation().x() + i*dir[0],
			pos.translation().y() + i*dir[1],
			pos.translation().z() + i*dir[2]);
		graspItGUI->getIVmgr()->getWorld()->getBvs(mCurrentLoadedModel->getGraspableBody(), 0, &bvs);
		center = bvs[0].getTran().translation();
		//std::cout << "center is: " << center.x() << " " << center.y() << " " << center.z() << std::endl;
		//std::cout << "rotation is: " << bvs[0].getTran().rotation().x << " " << bvs[0].getTran().rotation().y << " " << bvs[0].getTran().rotation().z << std::endl;
		//std::cout << "halfsize is: " << bvs[0].halfSize.x() << " " << bvs[0].halfSize.y() << " " << bvs[0].halfSize.z() << std::endl;

		vec3 size = bvs[0].halfSize;// * bvs[0].getTran();

		bool isWithin = true;
		//check all the corneres of the bounding box to see if they are within the field of view
		for(int x = -1; x <2; x += 2){
			for(int y = -1; y <2; y += 2){
				for(int z = -1; z <2; z += 2)
				{
					corner = vec3(center.x() + x * size.x(), center.y() + y * size.y(), center.z() + z * size.z());
					if(!checkWithin(tmp,corner,fov_vertical,fov_horizontal,up,dir))
						//if it is not within the fov continue to move backwards
						isWithin = false;
				}
			}
		}
		if(isWithin){
			loc = tmp;
			break;
		}
	}

	//std::cout << " " << "dir: " << dir.x() << " " << dir.y() << " " << dir.z() << "\n"  << 
	//"up: " << up.x() << " " << up.y() << " " << up.z() << "\n" <<
	//"loc: " << loc.x() << " " << loc.y() << " " << loc.z() << std::endl;

	//looking towards the object and save the scan
	sm.setupCameraPose(loc,dir,up);
	//sm.scanToFile(fn_raw, fn_model);

#define SAVE_PARAM
#ifdef SAVE_PARAM
	FILE *fp;
	std::string fn;
	fn.append("c:\\output_data\\task_");
	fn.append(mCurrentLoadedModel->ModelName().c_str());
	fn.append(".txt");
	//std::cout << fn.c_str() << std::endl;
	fp = fopen(fn.c_str(),"a");

	//print the output name
	fprintf(fp,"%s\n",filename_suffix.c_str());
	//print the scaling factor
	fprintf(fp,"%lf\n",mCurrentLoadedModel->RescaleFactor());
	//print out the center of gravity
	position p = mCurrentLoadedModel->getGraspableBody()->getCoG();
	fprintf(fp,"%lf %lf %lf\n",p.x(), p.y(), p.z());

	sm.saveParamsToFile(fp);
	fclose(fp);
#endif

#endif
}

bool DBaseDlg::checkWithin(vec3 origin, vec3 corner, double vertical, double horizontal, vec3 up, vec3 approach)
{
	//std::cout << "corner is: " << corner.x() << " " << corner.y() << " " << corner.z() << std::endl;
	vec3 origin2Corner = (corner - origin);
	origin2Corner = origin2Corner/origin2Corner.len();

	double approachProj = approach % origin2Corner;
	double upProj = up % origin2Corner;
	vec3 horizon = up * approach;
	double horizonProj = horizon % origin2Corner;

	if(approachProj < 0)
		return false;

	double v = atan(fabs(upProj/approachProj));
	if(v > vertical * 3.14159265 / 180.0)
		return false;

	double h = atan(fabs(horizonProj/approachProj));
	if(h > horizontal * 3.14159265 / 180.0)
		return false;
	return true;
}

void DBaseDlg::scanButton_clicked()
{
	sampleScans();
}

void DBaseDlg::sampleScans()
{
	position p = mCurrentLoadedModel->getGraspableBody()->getCoG();
	transf initP = transf(Quaternion(), vec3(p[0], p[1], -5000)), pose;

	//double error_theta, error_phi;
	//error_theta = 0.0 * M_PI / 180;
	//error_phi = 0.0 * M_PI / 180;

	int theta_step, phi_step;
	int theta_begin, theta_end, phi_begin, phi_end;
	double offset_theta, offset_phi;

	//theta_begin = 0.0;
	//theta_end = 360.0;
	//phi_begin = -90.0;
	//phi_end = 90.0;

	theta_begin = 0;
	theta_end = 360;
	phi_begin = 0;
	phi_end = 90;
	theta_step = 2;
	phi_step = 2;

	offset_theta = 0.0;
	offset_phi = 0.0;


	for(int theta_degree = theta_begin; theta_degree < theta_end; theta_degree += theta_step)
	{
		double theta = (theta_degree + offset_theta) * M_PI / 180.0;
		for(int phi_degree = phi_begin; phi_degree < phi_end; phi_degree += phi_step)
		{
			double phi = (phi_degree + offset_phi) * M_PI / 180.0;
			vec3 y =  vec3(0,1.0,0);
			vec3 l = vec3(sin(theta), 0, cos(theta));
			transf Rot_y = transf(Quaternion(theta,y),
				vec3(0,0,0));
			transf Rot_k = transf(Quaternion(phi, y*l),
				vec3(0,0,0));
			transf Transl_cog = transf(Quaternion(),
				vec3(p[0], p[1], p[2]));
			pose = initP * Transl_cog.inverse() * Rot_y * Rot_k * Transl_cog;
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(pose);

			std::string fn("c:\\output_data\\drill\\");
			//std::string fn;
			fn.append(mCurrentLoadedModel->ModelName());
			fn.append("_");
			char thetaN[20], phiN[20];
			sprintf(thetaN,"%d",theta_degree);
			//itoa(theta_degree,thetaN,10);
			sprintf(phiN,"%d",phi_degree);
			//itoa(phi_degree,phiN,10);
			fn.append(thetaN);
			fn.append("_");
			fn.append(phiN);
			collectScans(fn);
		}
	}

}

bool DBaseDlg::resolveCollision()
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
	h->autoGrasp(RENDER, -1.0);
	//move forward
	return h->approachToContact(500, false);
}
