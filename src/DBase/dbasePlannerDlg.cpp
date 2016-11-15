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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: dbasePlannerDlg.cpp,v 1.14 2009/07/06 20:35:25 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %DBasePlannerDlg class
 */

#include "dbasePlannerDlg.h"
#include <algorithm>

#include "body.h"
#include "robot.h"
#include "world.h"
#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "searchState.h"

#include "graspitGUI.h"
#include "mainWindow.h"

#include "DBPlanner/caching_neighbor_finder.h"
#include "DBPlanner/caching_aligner.h"
#include "graspit_db_planner.h"

#include "debug.h"

using std::string;

//helper functor that decide the order of two grasps based on their average test score
bool biggerInTestScores(db_planner::Grasp* g1, db_planner::Grasp* g2){
	return static_cast<GraspitDBGrasp*>(g1)->getTestAverageScore() > static_cast<GraspitDBGrasp*>(g2)->getTestAverageScore();
}

//helper functor that decide the order of two grasps based on their tested epsilon quality
bool biggerInEpsilonQuality(db_planner::Grasp* g1, db_planner::Grasp* g2){
	return static_cast<GraspitDBGrasp*>(g1)->getPreGraspPlanningState()->getEpsilonQuality() > 
		static_cast<GraspitDBGrasp*>(g2)->getPreGraspPlanningState()->getEpsilonQuality();
}

//initialize the dialogue
void DBasePlannerDlg::init(){
	//get the distance functions and display their names
	std::vector<string> distFuncList;
	if(!mDBMgr->DistanceFunctionList(&distFuncList)){
		DBGA("Distance function names retrival failed");
		return;
	}
	initializeDistanceComboBox(distFuncList);
	//instantialize the grasp ranker
	mGraspRanker = new db_planner::GraspRanker("Talk to Corey");
	//disable the unnecessary groups at current time
	setGroupBoxEnabled(true, false, false, false, false);
}

void DBasePlannerDlg::destroy(){
	// release the memories occupied during the grasp planning
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mOriginalGrasps);
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mTestedGrasps);
}

void DBasePlannerDlg::exitButton_clicked(){
	//recover the display to the original scene
	show3DObject();
	QDialog::accept();
}

void DBasePlannerDlg::getNeighborButton_clicked(){
	// get the neighbors from neighbor finder
	mNeighbors.clear();
	mNeighborFinder = new db_planner::CachingNeighborFinder(*mDBMgr, false, distanceFunctionComboBox->currentText().toStdString());
	mNeighborFinder->Find(*static_cast<GraspitDBModel*>(mPlanningModel), numOfNeighborsSpinBox->value(), &mNeighbors);
	if(mNeighbors.empty()) return;
	//update the neighbor combo box
	updateNeighborList();
	//show the neighbor's image
	updateModelImage(mNeighbors[neighborComboBox->currentIndex()].first);
	//disable the unnecessary groups at current time
	setGroupBoxEnabled(true, true, true, false, false);
}
// execute a kind of test, test current one or all AND dynamic test or static test
void DBasePlannerDlg::executeButton_clicked(){
	if(graspRangeComboBox->currentText() == QString("Test current")){
		if(executionTypeComboBox->currentText() == QString("Static"))
			mPlanner->testCurrentGrasp(GraspitDBPlanner::STATIC);
		else{
			GraspitDBPlanner::DynamicCode dynCode ;
			mPlanner->testCurrentGrasp(GraspitDBPlanner::DYNAMIC, &dynCode);
		}
	}
	else if(graspRangeComboBox->currentText() == QString("Test all")){
		if(executionTypeComboBox->currentText() == QString("Static"))
			mPlanner->testGrasps(GraspitDBPlanner::STATIC, mOriginalGrasps, &mTestedGrasps);
		else{
			mPlanner->testGrasps(GraspitDBPlanner::DYNAMIC, mOriginalGrasps, &mTestedGrasps);
		}
		std::sort(mTestedGrasps.begin(), mTestedGrasps.end(), biggerInEpsilonQuality);
		mCurrentTestedGrasp = 0;
	}
	else{
		DBGA("Execution failed\n");
		return;
	}
	float eq = 0, vq = 0;
	if(graspRangeComboBox->currentText() == QString("Test current"))
		mPlanner->computeQuality(eq, vq);
	testedEpsilonQualityLabel->setText(QString("Epsilon Quality: ") + QString::number(eq));
	testedVolumeQualityLabel->setText(QString("Volume Quality: ") + QString::number(vq));
}

// retrieve all the grasps from the neighbors, without doing alignment here
void DBasePlannerDlg::retrieveGraspsButton_clicked(){
	//first clean the vector to store the grasps
	deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mOriginalGrasps);
	mOriginalGrasps.clear();
	mCurrentOriginalGrasp = 0;
	mCurrentTestedGrasp = 0;
	//go through all the neighbors and get their original grasps
	for(int i = 0; i < (int)mNeighbors.size(); ++i){
		std::vector<db_planner::Grasp*> grasps;
		if(!mDBMgr->GetGrasps(*(mNeighbors[i].first), GraspitDBGrasp::getHandDBName(mHand).toStdString(), &grasps)){
			DBGA("Cannot retrieve grasps from neighbor:" << mNeighbors[i].first->ModelName().c_str());
			return;
		}
		mOriginalGrasps.insert(mOriginalGrasps.end(), grasps.begin(), grasps.end());
		grasps.clear();
	}
	updateOriginalGraspInfo();
}
// rank the retrieved grasps based on the selected test on their pre-grasps
void DBasePlannerDlg::rankGraspsButton_clicked(){
	//instantialize an aligner based ont he alignment method
	if(mAligner)
		delete mAligner;
	mAligner = new db_planner::CachingAligner(*mDBMgr, false, false, alignmentMethodComboBox->currentText().toStdString());
	//instantialize a planner to do the ranking
	mPlanner = new GraspitDBPlanner(mHand, mPlanningModel, mDBMgr, mAligner);
	if(mOriginalGrasps.size() == 0){
		DBGA("No grasps to rank");
		return;
	}
	//assemble the test model's list
	std::vector<db_planner::Model*> neighborList;
	for(size_t i = 0; i < mNeighbors.size(); ++i){
		neighborList.push_back(mNeighbors[i].first);
	}
	//decide whether to use cross correlation to rank
	if(rankingMethodComboBox->currentText() == QString("Across neighbors")){
		mPlanner->crossCorrelate(neighborList, mOriginalGrasps);
		std::stable_sort(mOriginalGrasps.begin(), mOriginalGrasps.end(), biggerInTestScores);
	}
	//update the original grasp information retrieved from the database and show it
	updateOriginalGraspInfo();
	//disable the unnecessary groups at current time
	setGroupBoxEnabled(true, true, true, true, true);
	//show the first grasp of the retrieved list
	mCurrentOriginalGrasp = 0;
	if(mCurrentOriginalGrasp < (int)mOriginalGrasps.size())
		showGrasp(mOriginalGrasps[mCurrentOriginalGrasp]);
	updateOriginalGraspInfo();
}

// show the previous pre-grasp and do corresponding display of thumbnail and model if needed
void DBasePlannerDlg::previousGraspButton_clicked(){
	if(originalGraspRadioButton->isChecked()){
		previousGrasp(mCurrentOriginalGrasp, mOriginalGrasps);
		updateOriginalGraspInfo();
	}else if(testedGraspRadioButton->isChecked()){
		previousGrasp(mCurrentTestedGrasp, mTestedGrasps);
		updateTestedGraspInfo();
	}
	testedEpsilonQualityLabel->setText("Epsilon Quality: -1.0");
	testedVolumeQualityLabel->setText("Volume Quality: -1.0");
	show3DObject(seeNeighborCheckBox->isChecked());
}

// show the next pre-grasp and do corresponding display of thumbnail and model if needed
void DBasePlannerDlg::nextGraspButton_clicked(){
	if(originalGraspRadioButton->isChecked()){
		nextGrasp(mCurrentOriginalGrasp, mOriginalGrasps);
		updateOriginalGraspInfo();
	}else if(testedGraspRadioButton->isChecked()){
		nextGrasp(mCurrentTestedGrasp, mTestedGrasps);
		updateTestedGraspInfo();
	}
	testedEpsilonQualityLabel->setText("Epsilon Quality: -1.0");
	testedVolumeQualityLabel->setText("Volume Quality: -1.0");
	show3DObject(seeNeighborCheckBox->isChecked());
}

// shortcut for the GWS display
void DBasePlannerDlg::createGWSButton_clicked(){
	graspItGUI->getMainWindow()->graspCreateProjection();
}

// put the entries into the distance function combo box
void DBasePlannerDlg::initializeDistanceComboBox(std::vector<string> entries){
	for(int i = 0; i < (int)entries.size(); ++i){
		distanceFunctionComboBox->insertItem(QString(entries[i].c_str()));
	}
}

// update the original grasp information retrieved from the database
void DBasePlannerDlg::updateOriginalGraspInfo(){
	graspIndexLabel->setText(QString::number(mCurrentOriginalGrasp + 1) + QString("/") + QString::number(mOriginalGrasps.size()));
	if(mOriginalGrasps.empty()) return;
	originalEpsilonQualityLabel->setText(QString("Epsilon Quality: ") + QString::number(mOriginalGrasps[mCurrentOriginalGrasp]->EpsilonQuality()));
	originalVolumeQualityLabel->setText(QString("Volume Quality: ") + QString::number(mOriginalGrasps[mCurrentOriginalGrasp]->VolumeQuality()));
	int i;
	for( i = 0; i < (int)mNeighbors.size(); ++i){
		if(mNeighbors[i].first->ModelName() == mOriginalGrasps[mCurrentOriginalGrasp]->SourceModel().ModelName())
			break;
	}
	if(i < (int)mNeighbors.size())
		neighborComboBox->setCurrentIndex(i);
}

// update the tested grasp information
void DBasePlannerDlg::updateTestedGraspInfo(){
	graspIndexLabel->setText(QString::number(mCurrentTestedGrasp + 1) + QString("/") + QString::number(mTestedGrasps.size()));
	if(mTestedGrasps.empty()) return;
	float eq, vq;
	mPlanner->computeQuality(eq, vq);
	originalEpsilonQualityLabel->setText(QString("Epsilon Quality: ") + QString::number(eq));
	originalVolumeQualityLabel->setText(QString("Volume Quality: ") + QString::number(vq));
	int i;
	for(i = 0; i < (int)mNeighbors.size(); ++i){
		if(mNeighbors[i].first->ModelName() == mTestedGrasps[mCurrentTestedGrasp]->SourceModel().ModelName())
			break;
	}
	if(i < (int)mNeighbors.size())
		neighborComboBox->setCurrentIndex(i);
}

// given a grasp list and a current position index i, show the previous grasp before i
void DBasePlannerDlg::previousGrasp(int& i, std::vector<db_planner::Grasp*> graspList) {
	if (graspList.empty()) return;
	i --;
	if (i < 0) i = graspList.size() - 1;
	showGrasp(graspList[i]);
}

// given a grasp list and a current position index i, show the next grasp after i
void DBasePlannerDlg::nextGrasp(int& i, std::vector<db_planner::Grasp*> graspList){
	if (graspList.empty()) return;
	i ++;
	if (i == graspList.size()) i = 0;
	showGrasp(graspList[i]);
}

/* show the original pre-grasp, aligned it before shown to the screen,
   this alignment does not influence the grasp by copying grasp to another
   grasp g for the transformation
*/
void DBasePlannerDlg::showGrasp(db_planner::Grasp* grasp){
	if(!mAligner){
		DBGA("Aligner is not available\n");
		return;
	}
	if (!grasp) return;
	GraspitDBGrasp *g = new GraspitDBGrasp(*static_cast<GraspitDBGrasp*>(grasp));
	if(!testedGraspRadioButton->isChecked()){
		float elmts[16];
		if(mAligner->Align(g->SourceModel(), *mPlanningModel, elmts))
			g->Transform(elmts);
	}
	static_cast<GraspitDBModel*>(mPlanningModel)->getGraspableBody()->setTran(transf::IDENTITY);
	g->getPreGraspPlanningState()->execute();
	if(mHand->isA("Barrett") && testedGraspRadioButton->isChecked()){
		graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(true);
	}
	mHand->getWorld()->findAllContacts();
	mHand->getWorld()->updateGrasps();
}

// update the neighbor list by synthesizing the new list
void DBasePlannerDlg::updateNeighborList(){
	neighborComboBoxInReconstruction = true;
	neighborComboBox->clear();
	for(int i = 0; i < (int)mNeighbors.size(); ++i){
		neighborComboBox->insertItem(mNeighbors[i].first->ModelName().c_str());
	}
	neighborComboBoxInReconstruction = false;
}

// update the model's thumbnail
void DBasePlannerDlg::updateModelImage(db_planner::Model* model){
	QString psbModelThumbPath = QString(model->ThumbnailPath().c_str());
	if(mModelScene) delete mModelScene;
	mModelScene = new QGraphicsScene;
	QPixmap lPixmap;
	lPixmap.load(psbModelThumbPath);
	mModelScene->addPixmap(lPixmap);
	this->objectGraph->setScene(mModelScene);
	this->objectGraph->show();
}

/* trigger when the selction of neighbor combo box is changed, show the right
   thumbnail of the neighbor
*/
void DBasePlannerDlg::modelChanged(){
	if(neighborComboBoxInReconstruction)
		return;
	updateModelImage(mNeighbors[neighborComboBox->currentIndex()].first);
}

// trigger when the checkbox is changed between display neighbor or not
void DBasePlannerDlg::neighborCheckBoxChanged(){
	show3DObject(seeNeighborCheckBox->isChecked());
	executeButton->setEnabled(!seeNeighborCheckBox->isChecked());
}

// trigger when we choose to show the original grasp
void DBasePlannerDlg::originalGraspRadioButton_clicked(){
	if(mCurrentOriginalGrasp < (int)mOriginalGrasps.size())
		showGrasp(mOriginalGrasps[mCurrentOriginalGrasp]);
	updateOriginalGraspInfo();
}

// trigger when we choose to show the tested grasp
void DBasePlannerDlg::testedGraspRadioButton_clicked(){
	if(mCurrentTestedGrasp < (int) mTestedGrasps.size())
		showGrasp(mTestedGrasps[mCurrentTestedGrasp]);
	updateTestedGraspInfo();
}

/* show the 3D object, if isNbr is true, we will show the neighbor
   after applying the alignment to the neighbor which transforms it 
   to the current model's coordinate system
*/
void DBasePlannerDlg::show3DObject(bool isNbr){
	db_planner::Model* m = mPlanningModel;
	if(isNbr){
		GraspitDBModel* dbm = static_cast<GraspitDBModel*>(mNeighbors[neighborComboBox->currentIndex()].first); 
		// load in the test model
		if (!dbm->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
			static_cast<GraspitDBModel*>(dbm)->load(mHand->getWorld());
		}
		m = dbm;
	}
	if(m == mModelShown)
		return;
	if(isNbr){
		//make necessary transformation on the model to be shown
		transf tr = transf::IDENTITY;
		float elmts[16];
		if(mAligner->Align(*m, *mPlanningModel, elmts)){
			mat3 mat;
			vec3 v(elmts[3],elmts[7],elmts[11]);
			mat[0] = elmts[0];
			mat[1] = elmts[1];
			mat[2] = elmts[2];
			mat[3] = elmts[4];
			mat[4] = elmts[5];
			mat[5] = elmts[6];
			mat[6] = elmts[8];
			mat[7] = elmts[9];
			mat[8] = elmts[10];
			tr.set(mat,v);
		}
		static_cast<GraspitDBModel*>(m)->getGraspableBody()->setTran( tr );
	}
	//destroy the current one, not delete from the memory
	mHand->getWorld()->destroyElement(static_cast<GraspitDBModel*>(mModelShown)->getGraspableBody(), false);
	static_cast<GraspitDBModel*>(m)->getGraspableBody()->addToIvc();
	//todo: where does dynamic information come from?
	//static_cast<GraspitDBModel*>(m)->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world
	mHand->getWorld()->addBody(static_cast<GraspitDBModel*>(m)->getGraspableBody());
	mModelShown = m;
}

// helper function that enables/disables the groups
void DBasePlannerDlg::setGroupBoxEnabled(bool neighborGenerator, bool alignment, bool ranking, bool grasp, bool execute){
	neighborGeneratorGroup->setEnabled(neighborGenerator);
	alignmentMethodGroup->setEnabled(alignment);
	graspRankingGroup->setEnabled(ranking);
	graspsGroup->setEnabled(grasp);
	executionGroup->setEnabled(execute);
}

// trigger when the alignment method is changed, in need of re-planning the grasps from the beginning
void DBasePlannerDlg::alignmentChanged(){
	setGroupBoxEnabled(true, true, true, false, false);
}

// helper function that deltes the vector elements of type vectorType as treatAsType
template <class vectorType, class treatAsType>
inline void DBasePlannerDlg::deleteVectorElements(std::vector<vectorType>& v){
	for(size_t i = 0; i < v.size(); ++i){
		delete (treatAsType)v[i];
	}
	v.clear();
}
