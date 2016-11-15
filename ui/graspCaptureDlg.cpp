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
// Author(s): Matei T. Ciocarlie
//
// $Id: graspCaptureDlg.cpp,v 1.15 2009/10/08 16:24:10 cmatei Exp $
//
//######################################################################

#include "graspCaptureDlg.h"

#include <QFileDialog>

#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "quality.h"
#include "searchState.h"
#include "contact.h"
#include "quality.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "mytools.h"

#include "debug.h"

#ifdef CGDB_ENABLED
#include "DBase/DBPlanner/grasp.h"
#include "DBase/graspit_db_grasp.h"
#include "DBase/graspit_db_model.h"
#include "DBase/DBPlanner/sql_database_manager.h"
#endif

void 
GraspCaptureDlg::init(World *w) {
	mWorld = w;
	QObject::connect(mWorld, SIGNAL(graspsUpdated()), this, SLOT(updateQuality()));
	mIndicator = new QualityIndicator(this);
	mIndicator->setAttribute(Qt::WA_ShowModal, false);
	mIndicator->setAttribute(Qt::WA_DeleteOnClose, false);
	mIndicator->show();
	mQualEpsilon = NULL;
	mQualVolume = NULL;
	mCurrentHand = NULL;
	saveToDBaseButton->setEnabled(FALSE);

	if (graspItGUI->getIVmgr()->getDBMgr()) {
		saveToDBaseButton->setEnabled(TRUE);
	} else {
		QTWARNING("DBase connection not found; only Save to File possible.");
		saveToDBaseButton->setEnabled(FALSE);
	}
}

GraspCaptureDlg::~GraspCaptureDlg()
{
	mIndicator->close();
	delete mIndicator;
	clearListButtonClicked();
	delete mQualEpsilon;
	delete mQualVolume;
}

bool
GraspCaptureDlg::checkHandSelection()
{
	Hand *hand = mWorld->getCurrentHand();
	if (!hand) return false;
	if (mCurrentHand != hand) {
		mCurrentHand = hand;
		delete mQualEpsilon;
		mQualEpsilon = new QualEpsilon(hand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
		delete mQualVolume; 
		mQualVolume = new QualVolume(hand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
	}
	GraspableBody *body = hand->getGrasp()->getObject();
	if (!body) return false;
	return true;
}

void 
GraspCaptureDlg::captureButtonClicked()
{
	if (!checkHandSelection()) return;
	if (!mWorld->noCollision()) {
		DBGA("COLLISION");
		return;
	}
	mWorld->findAllContacts();
	mWorld->updateGrasps();
        double quality = mQualEpsilon->evaluate();
        if (!allowNonFCBox->isChecked() && quality < 0.0) {
		DBGA("NON FORCE CLOSURE");
		return;
	}
	GraspPlanningState *newState = new GraspPlanningState(mCurrentHand);
	newState->setPostureType(POSE_DOF, false);
	GraspableBody *body = mCurrentHand->getGrasp()->getObject();
	newState->setRefTran(body->getTran());
	newState->setObject(body);
	newState->setEpsilonQuality( std::max(0.0, quality) );
	newState->setVolume( std::max(0.0, quality) );
	newState->saveCurrentHandState();
	for (int i=0; i<mCurrentHand->getGrasp()->getNumContacts(); i++) {
		newState->getContacts()->push_back( mCurrentHand->getGrasp()->getContact(i)->getPosition() );
	}
	mGrasps.push_back(newState);
	updateNumGrasps();

	if (graspItGUI->getIVmgr()->getDBMgr()) {
		saveToDBaseButton->setEnabled(TRUE);
	} else {
		saveToDBaseButton->setEnabled(FALSE);
	}
}

void 
GraspCaptureDlg::saveToFileButtonClicked()
{
	DBGA("Foo");
	if (mGrasps.empty()) {
		DBGA("No recorded grasps to save");
		return;
	}
	QString fn( QFileDialog::getSaveFileName(this, QString(), 
				QString(getenv("GRASPIT")), "Text Files (*.txt)") );
	if ( fn.isEmpty() ) return;
    if (fn.section('.',1).isEmpty()) fn.append(".txt");
	FILE *fp = fopen(fn.ascii(), "a");
	if (!fp) {
		DBGA("Failed to open save file " << fn.ascii());
		return;
	}
	std::list<GraspPlanningState*>::iterator it;
	for (it=mGrasps.begin(); it!=mGrasps.end(); it++) {
		(*it)->writeToFile(fp);
	}
	fclose(fp);
	DBGA("Grasps saved.");
}

/*! Saves the current list of grasps in the CGDB, if an interface exists.
	However, when we save in the CGDB, we need more information than we
	currently store in the HandObjectState. 
*/
void 
GraspCaptureDlg::saveToDBaseButtonClicked()
{
#ifndef CGDB_ENABLED
	return;
#else
	db_planner::DatabaseManager *dbMgr = graspItGUI->getIVmgr()->getDBMgr();
	if (!dbMgr) return;
	std::list<GraspPlanningState*>::iterator it;
	std::vector<db_planner::Grasp*> graspList;
	for (it=mGrasps.begin(); it!=mGrasps.end(); it++) {
		GraspitDBModel* dbModel= (*it)->getObject()->getDBModel();
		if (!dbModel) {
			DBGA("Model not from database!");
			continue;
		}
		db_planner::Grasp* grasp = new GraspitDBGrasp(mCurrentHand);
		grasp->SetSourceModel( *(static_cast<db_planner::Model*>(dbModel)) );
		grasp->SetHandName(GraspitDBGrasp::getHandDBName(mCurrentHand).toStdString());
		grasp->SetEpsilonQuality((*it)->getEpsilonQuality());
		grasp->SetVolumeQuality((*it)->getVolume());
		grasp->SetEnergy( - 30*(*it)->getEpsilonQuality() - 100*(*it)->getVolume() );
		//Hard-coded source: human operator
		//should have a ComboBox here where the operator can select a source from what 
		//options the database gives us
		grasp->SetSource("HUMAN_REFINED");

		std::vector<double> tempArray;
		//the posture
		for(int i = 0; i < (*it)->getPosture()->getNumVariables(); ++i){
			tempArray.push_back((*it)->getPosture()->getVariable(i)->getValue());
		}
		grasp->SetPregraspJoints(tempArray);
		grasp->SetFinalgraspJoints(tempArray);

		//the position
		tempArray.clear();
		for(int i = 0; i < (*it)->getPosition()->getNumVariables(); ++i){
			tempArray.push_back((*it)->getPosition()->getVariable(i)->getValue());
		}
		grasp->SetPregraspPosition(tempArray);
		grasp->SetFinalgraspPosition(tempArray);

		//the contacts
		std::list<position> *contacts;
		tempArray.clear();
		contacts = (*it)->getContacts();
		std::list<position>::iterator itContact;
		for(itContact = contacts->begin(); itContact != contacts->end(); ++itContact){
			tempArray.push_back((*itContact).x());
			tempArray.push_back((*itContact).y());
			tempArray.push_back((*itContact).z());
		}
		grasp->SetContacts(tempArray);
		//store this
		graspList.push_back(grasp);
	}
	dbMgr->SaveGrasps(graspList);
	DBGA(graspList.size() << " grasps successfully saved to database");
	for(size_t i = 0; i < graspList.size(); ++i){
		delete graspList[i];
	}
	//just to make sure we can't click twice by mistake
	clearListButtonClicked();
#endif
}

void
GraspCaptureDlg::updateNumGrasps()
{
	QString num;
	num.setNum((int)mGrasps.size());
	recordedGraspsLabel->setText("Recorded grasps: " + num);
}

void 
GraspCaptureDlg::clearListButtonClicked()
{
	while (!mGrasps.empty()) {
		delete mGrasps.back();
		mGrasps.pop_back();
	}
	updateNumGrasps();
}

void
GraspCaptureDlg::updateQuality()
{
	double quality = 0.0;
	if (checkHandSelection()) {
		quality = mQualEpsilon->evaluate();
	}
	mIndicator->setBar( std::max(0.0, quality) );
}
