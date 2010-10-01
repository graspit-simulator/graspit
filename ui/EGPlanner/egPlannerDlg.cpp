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
// $Id: egPlannerDlg.cpp,v 1.9 2009/07/02 21:04:05 cmatei Exp $
//
//######################################################################

#include "egPlannerDlg.h"

#include <QGridLayout>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QValidator>
#include <QFileDialog>

#include "search.h"
#include "searchState.h"
#include "body.h"
#include "robot.h"
#include "egPlanner.h"
#include "eigenGrasp.h"
#include "grasp.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "contactExaminerDlg.h"
#include "onLinePlanner.h"
#include "timeTest.h"
#include "guidedPlanner.h"
#include "loopPlanner.h"


//#define GRASPITDBG
#include "debug.h"

void EigenGraspPlannerDlg::exitButton_clicked()
{
 QDialog::accept();
}

void EigenGraspPlannerDlg::init()
{ 
 energyBox->insertItem("Hand Contacts");
 energyBox->insertItem("Potential Quality");
 energyBox->insertItem("Contacts AND Quality");
 energyBox->insertItem("Autograsp Quality");
 energyBox->insertItem("Guided Autograsp");
 energyBox->setCurrentItem(0);

 plannerTypeBox->insertItem("Sim. Ann.");
 plannerTypeBox->insertItem("Loop");
 plannerTypeBox->insertItem("Multi-Threaded");
 plannerTypeBox->insertItem("Online");
 plannerTypeBox->insertItem("Time Test");
 plannerTypeBox->setCurrentItem(0);
 
 plannerInitButton->setEnabled(TRUE);
 plannerResetButton->setEnabled(FALSE);
 plannerStartButton->setEnabled(FALSE);
 instantEnergyButton->setEnabled(FALSE);

 useVirtualHandBox->setChecked(true);
 onlineDetailsGroup->setEnabled(FALSE);

 QString n;
 QIntValidator* vAnnSteps = new QIntValidator(1,500000,this);
 annStepsEdit->setValidator(vAnnSteps);
 n.setNum(70000);
 annStepsEdit->setText(n);

 spaceSearchBox->insertItem("Complete");
 spaceSearchBox->insertItem("Axis-angle");
 spaceSearchBox->insertItem("Ellipsoid");
 spaceSearchBox->insertItem("Approach");
 spaceSearchBox->setCurrentItem(1);

 prevGraspButton->setEnabled(FALSE);
 nextGraspButton->setEnabled(FALSE);
 bestGraspButton->setEnabled(FALSE);

 variableBox->setColumnLayout(0, Qt::Vertical);

 varGridLayout = new QGridLayout( variableBox->layout(),1,5 );
 varGridLayout->setSpacing(5);
 varGridLayout->setAlignment(Qt::AlignTop);
 varGridLayout->addMultiCellWidget(spaceSearchLabel,0,0,0,1);
 varGridLayout->addMultiCellWidget(spaceSearchBox,0,0,2,4);

 varGridLayout->addWidget( new QLabel("On", variableBox),1,0 );
 varGridLayout->addWidget( new QLabel("Name", variableBox),1,1 );
 varGridLayout->addWidget( new QLabel("Input", variableBox),1,2 );
 varGridLayout->addWidget( new QLabel("Target", variableBox),1,3 );
 varGridLayout->addWidget( new QLabel("Confidence", variableBox),1,4 );

 inputGloveBox->setEnabled(FALSE);
 inputLoadButton->setEnabled(FALSE);

 fprintf(stderr,"INIT DONE \n");
}

void EigenGraspPlannerDlg::destroy()
{
 delete mHandObjectState;
 if (mPlanner) delete mPlanner;

 //cleanup
 for (unsigned int i=0; i<varNames.size(); i++) {
//  varMainLayout->removeItem(varLayouts[i]);

  varGridLayout->remove(varNames[i]);
  varGridLayout->remove(varCheck[i]);
  varGridLayout->remove(varInput[i]);
  varGridLayout->remove(varTarget[i]);
  varGridLayout->remove(varConfidence[i]);

  delete varNames[i];
  delete varCheck[i];
  delete varInput[i];
  delete varTarget[i];
  delete varConfidence[i];

 }
 varNames.clear();
 varCheck.clear();
 varInput.clear();
 varConfidence.clear();
 varTarget.clear();
 varLayouts.clear();
}

void EigenGraspPlannerDlg::setMembers( Hand *h, GraspableBody *b )
{
 mPlanner = NULL;
 mHand = h;
 mObject = b;
 mHand->getGrasp()->setObjectNoUpdate(mObject);
// mHand->getGrasp()->setGravity(true);
 mHand->getGrasp()->setGravity(false);

 mHandObjectState = new GraspPlanningState(mHand);
 mHandObjectState->setObject(mObject);
 mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
 mHandObjectState->setRefTran(mObject->getTran());
 mHandObjectState->reset();
 setVariableLayout();

 if (mHand->getNumVirtualContacts() > 0) {
	 setContactsBox->setChecked(TRUE);
 }

 updateVariableLayout();
 updateInputLayout();
}

// ----------------------------------- Search State and variable layout management -------------------------------

void EigenGraspPlannerDlg::setVariableLayout()
{
 //cleanup
 for (unsigned int i=0; i<varNames.size(); i++) {
  varGridLayout->remove(varNames[i]);
  varGridLayout->remove(varCheck[i]);
  varGridLayout->remove(varInput[i]);
  varGridLayout->remove(varTarget[i]);
  varGridLayout->remove(varConfidence[i]);
  delete varNames[i];
  delete varCheck[i];
  delete varInput[i];
  delete varTarget[i];
  delete varConfidence[i];
 }
 varNames.clear();
 varCheck.clear();
 varInput.clear();
 varTarget.clear();
 varConfidence.clear();
 varLayouts.clear();

 int size = mHand->getEigenGrasps()->getSize() + 7;

 for (int i=0; i<size; i++) {
  QLabel *name = new QLabel("foo",variableBox);
  QCheckBox *check = new QCheckBox(variableBox);
  connect(check, SIGNAL(clicked()), this, SLOT(variableCheckBoxChanged()));
  QCheckBox *inputCheck = new QCheckBox(variableBox);
  connect(inputCheck, SIGNAL(clicked()), this, SLOT(variableInputChanged()));
  QLabel *target = new QLabel("N/A",variableBox);
  QSlider *slider = new QSlider(0,100,10,0,Qt::Horizontal,variableBox);
  connect(slider, SIGNAL(sliderReleased()), this, SLOT(variableInputChanged()));

  slider->setLineStep(1);
  slider->setMaximumWidth(50);
  varGridLayout->addWidget(check,2+i,0);
  varGridLayout->addWidget(name,2+i,1);
  varGridLayout->addWidget(inputCheck,2+i,2);
  varGridLayout->addWidget(target,2+i,3);
  varGridLayout->addWidget(slider,2+i,4);
  

  varCheck.push_back( check );
  varNames.push_back( name );
  varInput.push_back( inputCheck );
  varTarget.push_back( target );
  varConfidence.push_back( slider );
 }
}

void EigenGraspPlannerDlg::updateVariableLayout()
{
	int i;
 for (i=0; i<mHandObjectState->getNumVariables(); i++) {
  varNames[i]->setEnabled(TRUE);
  varNames[i]->setText( mHandObjectState->getVariable(i)->getName() );
  varCheck[i]->setEnabled(TRUE);
  if (mHandObjectState->getVariable(i)->isFixed()) varCheck[i]->setChecked(false);
  else varCheck[i]->setChecked(true);
 }

 for (i=mHandObjectState->getNumVariables(); i < mHand->getEigenGrasps()->getSize() + 7; i++) {
  varNames[i]->setEnabled(FALSE);
  varNames[i]->setText( "n/a" );
  varCheck[i]->setChecked(false);
  varCheck[i]->setEnabled(FALSE);
 }
}

void EigenGraspPlannerDlg::updateInputLayout()
{
	int i;
 for (i=0; i<mHandObjectState->getNumVariables(); i++) {
   if ( !mPlanner || !(mPlanner->isReady() || mPlanner->isActive()) ) {
   varInput[i]->setEnabled(FALSE);
   varInput[i]->setChecked(false);
   varTarget[i]->setText("N/A");
   varTarget[i]->setEnabled(FALSE);
   varConfidence[i]->setValue( 0 );
   varConfidence[i]->setEnabled(FALSE);
  } else {
   GraspPlanningState *t = mPlanner->getTargetState();
   varInput[i]->setEnabled(TRUE);
   QString n;
   n.setNum(t->getVariable(i)->getValue(),'f',3);
   varTarget[i]->setText(n);
   varConfidence[i]->setValue( t->getVariable(i)->getConfidence() * 100 );
   if ( t->getVariable(i)->isFixed() ) {
    varInput[i]->setChecked(TRUE);
    varTarget[i]->setEnabled(TRUE);
    varConfidence[i]->setEnabled(TRUE);
   } else {
    varInput[i]->setChecked(FALSE);
    varTarget[i]->setEnabled(FALSE);
    varConfidence[i]->setEnabled(FALSE);
   }
  }
 }

 for (i=mHandObjectState->getNumVariables(); i < mHand->getEigenGrasps()->getSize() + 7; i++) {
  varInput[i]->setEnabled(FALSE);
  varInput[i]->setChecked(false);
  varTarget[i]->setText("N/A");
  varTarget[i]->setEnabled(FALSE);
  varConfidence[i]->setValue( 0 );
  varConfidence[i]->setEnabled(FALSE);
 }
}

void EigenGraspPlannerDlg::variableInputChanged()
{
	assert(mPlanner);
	GraspPlanningState *t = mPlanner->getTargetState();
	assert(t);
	for (int i=0; i<mHandObjectState->getNumVariables(); i++) {
		if (varInput[i]->isChecked()) {
			varTarget[i]->setEnabled(TRUE);
			varConfidence[i]->setEnabled(TRUE);
			t->getVariable(i)->setFixed(true);
			t->getVariable(i)->setConfidence( ((double)varConfidence[i]->value()) / 100.0);
			DBGP("Conf: " << ((double)varConfidence[i]->value()) / 100.0);
		}
		else {
			varTarget[i]->setEnabled(FALSE);
			t->getVariable(i)->setFixed(false);
			t->getVariable(i)->setConfidence(0);
			varConfidence[i]->setValue(0);
			varConfidence[i]->setEnabled(FALSE);
		}
	}
}

void EigenGraspPlannerDlg::variableCheckBoxChanged()
{
 for (int i=0; i<mHandObjectState->getNumVariables(); i++) {
  if (varCheck[i]->isChecked()) mHandObjectState->getVariable(i)->setFixed(false);
  else mHandObjectState->getVariable(i)->setFixed(true);
 }
 //force a reset of the planner
 plannerStartButton->setEnabled(FALSE);
}

void EigenGraspPlannerDlg::spaceSearchBox_activated( const QString &s )
{
 if ( s==QString("Complete") ) {
  mHandObjectState->setPositionType(SPACE_COMPLETE);
  mHandObjectState->setRefTran( mObject->getTran() );
 }  else if ( s==QString("Axis-angle") ) {
  mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
  mHandObjectState->setRefTran( mObject->getTran() );
 } else if ( s==QString("Ellipsoid") ) {
  mHandObjectState->setPositionType(SPACE_ELLIPSOID);
  mHandObjectState->setRefTran( mObject->getTran() );
 } else if ( s==QString("Approach") ) {
  mHandObjectState->setPositionType(SPACE_APPROACH);
  mHandObjectState->setRefTran( mHand->getTran() );
 } else {
  fprintf(stderr,"WRONG SEARCH TYPE IN DROP BOX!\n");
 }
 mHandObjectState->reset();
 updateVariableLayout();
 //force a reset of the planner
 if (mPlanner) mPlanner->invalidateReset();
 updateStatus();
}

//------------------------------------- Show Results stuff ---------------------------------

void EigenGraspPlannerDlg::prevGraspButton_clicked()
{
 mDisplayState--;
 updateResults(true);
}

void EigenGraspPlannerDlg::bestGraspButton_clicked()
{
 if (!mPlanner) return;
 mDisplayState = 0;
 updateResults(true);
}

void EigenGraspPlannerDlg::nextGraspButton_clicked()
{
 mDisplayState++;
 updateResults(true);
}

void EigenGraspPlannerDlg::plannerUpdate()
{
 assert(mPlanner);
 mDisplayState = 0;
 updateResults(false);
 //if we are using the CyberGlove for input this updates the target values
 if (inputGloveBox->isChecked()) {
	 updateInputLayout();
 }
}

void EigenGraspPlannerDlg::updateResults(bool render)
{
 assert(mPlanner);

 QString nStr;
 nStr.setNum(mPlanner->getCurrentStep());
 currentStepLabel->setText(QString("Current step: ") + nStr);

 nStr.setNum(mPlanner->getRunningTime(),'f',0);
 timeLabel->setText(QString("Time used: ") + nStr + QString(" sec."));

 int d = mPlanner->getListSize();
 int rank, size, iteration; double energy;

 if (d==0) {
  mDisplayState = rank = size = energy = iteration = 0; render = false;
 } else if (mDisplayState < 0){
  mDisplayState = 0;
 } else if ( mDisplayState >= d) {
  mDisplayState = d-1;
 } 
 
 if ( d!=0 ){
  const GraspPlanningState *s = mPlanner->getGrasp( mDisplayState);
  rank = mDisplayState+1;
  size = d;
  iteration = s->getItNumber();
  energy = s->getEnergy();
 }

 /*
 FILE *f = fopen("foo.txt","w");
 for (int i=0; i<mPlanner->getListSize(); i++) {
	 for(int j=i+1; j<mPlanner->getListSize(); j++) {
		 float d = mPlanner->getGrasp(i)->distance( mPlanner->getGrasp(j) );
		 fprintf(stderr,"%d -- %d: %f\n",i+1,j+1,d);
	 }
	 fprintf(stderr,"\n");
	 mPlanner->getGrasp(i)->writeToFile(f);
 }
 fclose(f);
 */

 QString n1,n2;
 n1.setNum(rank);
 n2.setNum(size);
 rankLabel->setText("Rank: " + n1 + "/" + n2);
 n1.setNum(iteration);
 iterationLabel->setText("Iteration: " + n1);
 n1.setNum(energy,'f',3);
 energyLabel->setText("Energy: " + n1);

 if (render) {
  mPlanner->showGrasp(mDisplayState);
  //mHand->getWorld()->findAllContacts();
  //mHand->getGrasp()->update();
  mPlanner->getGrasp(mDisplayState)->printState();
  //mHand->autoGrasp(true);
 }
}

// ----------------------------- Settings management ---------------------------


void EigenGraspPlannerDlg::updateStatus()
{
	PlannerState s = DONE;
	if (mPlanner) s = mPlanner->getState();
	DBGP("Update Layout");
	switch(s) {
		case INIT:
			plannerInitButton->setEnabled(FALSE);
			plannerResetButton->setEnabled(TRUE);
			plannerStartButton->setEnabled(FALSE);
			plannerStartButton->setText(">");

			prevGraspButton->setEnabled(FALSE);
			nextGraspButton->setEnabled(FALSE);
			bestGraspButton->setEnabled(FALSE);
			mObject->showFrictionCones(true);

			inputGloveBox->setEnabled(FALSE);
			inputLoadButton->setEnabled(FALSE);
			onlineDetailsGroup->setEnabled(FALSE);
			break;
		case READY:
			{
			plannerInitButton->setEnabled(FALSE);
			plannerResetButton->setEnabled(TRUE);
			plannerStartButton->setEnabled(TRUE);
			plannerStartButton->setText(">");

			prevGraspButton->setEnabled(TRUE);
			nextGraspButton->setEnabled(TRUE);
			bestGraspButton->setEnabled(TRUE);
			mObject->showFrictionCones(true);

			inputGloveBox->setEnabled(TRUE);
			inputLoadButton->setEnabled(TRUE);
			if (mPlanner->getType() == PLANNER_ONLINE) onlineDetailsGroup->setEnabled(TRUE);
			else onlineDetailsGroup->setEnabled(FALSE);
			}
			break;
		case RUNNING:
			plannerInitButton->setEnabled(FALSE);
			plannerResetButton->setEnabled(FALSE);
			plannerStartButton->setEnabled(TRUE);
			plannerStartButton->setText("||");

			prevGraspButton->setEnabled(FALSE);
			nextGraspButton->setEnabled(FALSE);
			bestGraspButton->setEnabled(FALSE);
			mObject->showFrictionCones(false);
			break;
		default:
			plannerInitButton->setEnabled(TRUE);
			plannerResetButton->setEnabled(FALSE);
			plannerStartButton->setEnabled(FALSE);
			plannerStartButton->setText(">");

			prevGraspButton->setEnabled(FALSE);
			nextGraspButton->setEnabled(FALSE);
			bestGraspButton->setEnabled(FALSE);
			mObject->showFrictionCones(true);

			inputGloveBox->setEnabled(FALSE);
			inputLoadButton->setEnabled(FALSE);
			onlineDetailsGroup->setEnabled(FALSE);
			break;
	}
	updateInputLayout();
}

void EigenGraspPlannerDlg::energyBox_activated( const QString & )
{
 //force a reset of the planner
	if (mPlanner) mPlanner->invalidateReset();
	updateStatus();
}

void EigenGraspPlannerDlg::setContactsBox_toggled( bool checked)
{
 if (checked) {
  if ( mHand->getNumVirtualContacts() == 0 ) {
   //if we are asking to use pre-set contacts, but none are available, pop up the dialog
   //for loading virtual contacts
   ContactExaminerDlg dlg(this);
   dlg.exec();
  }
  if (mHand->getNumVirtualContacts() == 0) {
   //if we still have no virtual contacts, un-check the box
   setContactsBox->setChecked(false);
  }
 }
 if (mPlanner) mPlanner->invalidateReset();
 updateStatus();
}

void EigenGraspPlannerDlg::readPlannerSettings()
{
 assert(mPlanner);
 //energy type
 QString s = energyBox->currentText();
 if ( s == QString("Hand Contacts") ) {
  mPlanner->setEnergyType(ENERGY_CONTACT);
 } else if ( s == QString("Potential Quality") ) {
  mPlanner->setEnergyType(ENERGY_POTENTIAL_QUALITY);
 } else if ( s == QString("Autograsp Quality") ) {
  mPlanner->setEnergyType(ENERGY_AUTOGRASP_QUALITY);
 } else if ( s == QString("Contacts AND Quality") ) {
  mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
 } else if ( s == QString("Guided Autograsp") ) {
  mPlanner->setEnergyType(ENERGY_GUIDED_AUTOGRASP);
 } else {
  fprintf(stderr,"WRONG ENERGY TYPE IN DROP BOX!\n");
 }

 //contact type
 if ( setContactsBox->isChecked() ) {
  mPlanner->setContactType(CONTACT_PRESET);
 } else {
  mPlanner->setContactType(CONTACT_LIVE);
 }

 //steps
 int steps = annStepsEdit->text().toInt();
 mPlanner->setMaxSteps(steps);
}

void EigenGraspPlannerDlg::plannerComplete()
{
 updateStatus();
 bestGraspButton_clicked();
}
//----------------------------------- Planner start / stop control stuff ---------------------------

void EigenGraspPlannerDlg::plannerInit_clicked()
{
 QString s = plannerTypeBox->currentText();
 if (s == QString("Sim. Ann.")) {
  if (mPlanner) delete mPlanner;
  mPlanner = new SimAnnPlanner(mHand);
  ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);
  energyBox->setEnabled(TRUE);
 } else if (s == QString("Loop")) {
  if (mPlanner) delete mPlanner;
  mPlanner = new LoopPlanner(mHand);
  ((LoopPlanner*)mPlanner)->setModelState(mHandObjectState);
  energyBox->setEnabled(TRUE);
 } else if (s == QString("Multi-Threaded")) {
  if (mPlanner) delete mPlanner;
  mPlanner = new GuidedPlanner(mHand);
  ((GuidedPlanner*)mPlanner)->setModelState(mHandObjectState);
  energyBox->setCurrentItem(2);
  energyBox->setEnabled(FALSE);
 } else if (s == QString("Online") ) {
  if (mPlanner) delete mPlanner;
  mPlanner = new OnLinePlanner(mHand);
  ((OnLinePlanner*)mPlanner)->setModelState(mHandObjectState);
  energyBox->setEnabled(TRUE);
  energyBox->setCurrentItem(2);
  QString n;
  n.setNum(2000);
  annStepsEdit->setText(n);
  QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(onlinePlannerUpdate())); 
 } else if ( s == QString("Time Test") ) {
  if (mPlanner) delete mPlanner;
  mPlanner = new MTTester(mHand);
 } else {
  fprintf(stderr,"Unknown planner type requested\n");
  return;
 }

 QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(plannerUpdate()));
 QObject::connect(mPlanner,SIGNAL(complete()),this,SLOT(plannerComplete()));

 updateStatus();
 plannerReset_clicked();
}

void EigenGraspPlannerDlg::plannerReset_clicked() 
{
	assert(mPlanner);
	readPlannerSettings();
	mPlanner->resetPlanner();
	updateStatus();
}

void EigenGraspPlannerDlg::startPlanner()
{
	mPlanner->startPlanner();
	updateStatus();
}

void EigenGraspPlannerDlg::stopPlanner()
{
	mPlanner->pausePlanner();
	updateStatus();
}

void EigenGraspPlannerDlg::plannerStart_clicked()
{	
	if (!mPlanner->isActive()){
		startPlanner();
	} else {
		stopPlanner();
	}
}

void EigenGraspPlannerDlg::plannerTypeBox_activated( const QString & )
{
	if (mPlanner) {
		delete mPlanner;
		mPlanner = NULL;
	}
	updateStatus();
}

//----------------------------------- Dedicated on-line planner control ---------------------------

void EigenGraspPlannerDlg::autoGraspBox_clicked()
{

}

//this slot does the updating that's specific to the online planner
void EigenGraspPlannerDlg::onlinePlannerUpdate()
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 QString num;

 double objDist = op->getObjectDistance();
 num.setNum(objDist,'f',0);
 objDistLabel->setText("Object distance: "  + num);
/*
 double solDist = op->getSolutionDistance();
 if (solDist >= 0) num.setNum(solDist,'f',0);
 else num.setAscii("N/A");
 solDistLabel->setText("Solution distance: " + num);
*/
 ActionType s = op->getAction();
 switch (s) {
	 case ACTION_PLAN:
		 num.setAscii("PLANNING");
		 break;
	 case ACTION_GRASP:
		 num.setAscii("GRASPING");
		 break;
	 case ACTION_OPEN:
		 num.setAscii("OPEN");
		 break;
	 default:
		 num.setAscii("N/A");
		 break;
 }
 onlineStatusLabel->setText("Status: " + num);

 num.setNum( op->getSABufferSize() );
 saBufferLabel->setText("SimAnn buffer: " + num);
 num.setNum( op->getFCBufferSize() );
 fcBufferLabel->setText("FC Thread buffer: " + num); 
}

void EigenGraspPlannerDlg::onlineGraspButton_clicked()
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->action(ACTION_GRASP);
 onlinePlannerUpdate();
}

void EigenGraspPlannerDlg::onlineReleaseButton_clicked()
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->action(ACTION_OPEN);
 onlinePlannerUpdate();
}


void EigenGraspPlannerDlg::onlinePlanButton_clicked()
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->action(ACTION_PLAN);
 onlinePlannerUpdate(); 
}


void EigenGraspPlannerDlg::instantEnergyButton_clicked()
{
 assert(mPlanner);
// mPlanner->instantEnergy();
}


void EigenGraspPlannerDlg::showCloneBox_toggled( bool c)
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->showClone(c);  
}


void EigenGraspPlannerDlg::showSolutionBox_toggled( bool c)
{	
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->showSolutionClone(c);
}

void EigenGraspPlannerDlg::useVirtualHandBox_clicked()
{
	/*
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 bool c = useVirtualHandBox->isChecked();
 op->controlVirtualHand(c);
 */

}

void EigenGraspPlannerDlg::useRealBarrettBox_toggled( bool s)
{
 assert( mPlanner->getType() == PLANNER_ONLINE);
 OnLinePlanner *op = (OnLinePlanner*)mPlanner;
 op->useRealBarrettHand(s);
}

//---------------------------------------- Input selection -----------------------------------------

void EigenGraspPlannerDlg::inputGloveBox_toggled( bool on)
{
	assert(mPlanner);
	mPlanner->setInput(INPUT_GLOVE, on);
}

void EigenGraspPlannerDlg::inputLoadButton_clicked()
{
	assert(mPlanner);
	QString fn = QFileDialog::getOpenFileName(this, QString(),  QString(getenv("GRASPIT"))+QString("/models/grasps"),
		"Grasp Files (*.txt)" );

	if ( fn.isEmpty() ) {
		return;
	}

	FILE *fp = fopen(fn.latin1(), "r");
	bool success = true;
	if (!fp) {
		DBGA("Failed to open input file!");
		success = false;
	}else if (!mPlanner->getTargetState()->readFromFile(fp)) {
		DBGA("Failed to read target from input file!");
		success = false;
	} else {
		DBGA("Target values loaded successfully");
	}
	fclose(fp);
	mPlanner->setInput(INPUT_FILE, success);
	updateInputLayout();
}
