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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: barrettHandDlg.cpp,v 1.5 2009/04/21 16:21:57 cmatei Exp $
//
//#######################################################################include "barrettHandDlg.h"

#include <QValidator>
#include <QPushButton>
#include <QLineEdit>

#include "world.h"
#include "barrett.h"
#include "barrettHandDlg.h"

#ifdef HARDWARE_LIB
#include "BarrettHand.h"
#include "BarrettHandThread.h"
#endif

void BarrettHandDlg::init()
{
	QLineEdit *stepSize = (QLineEdit *) this->child("stepSize", "QLineEdit");
	stepSize->setValidator(new QIntValidator(0, 180, stepSize));
	withinSimulationUpdate = false;
	simulatedHandForContinuousOperation = NULL;
	userInteractionOngoing = false;
}

/*! Sets the simulation world used by this dialog, then takes the current 
	selected hand in the World and checks if it is a Barrett. If so, it 
	becomes the simulated hand, which is queried for its interface to a
	real hand. Right now, each simulated Barrett holds its own instance of
	an interface to a real hand.
*/
bool BarrettHandDlg::setWorld( World *w )
{
#ifndef HARDWARE_LIB
	return false;
#endif
	world = w;
	if (!world->getCurrentHand()) return false;
	if (!world->getCurrentHand()->isA("Barrett")) return false;
	mSimBarrett = (Barrett*)world->getCurrentHand();
	mRealBarrett = mSimBarrett->getRealHand();
	if (!mRealBarrett) return false;
	return true;
}

void BarrettHandDlg::initializeHand()
{
#ifdef HARDWARE_LIB
	mRealBarrett->HandInitialize(BARRETT_ALL);
	//while(mRealBarrett->isBusy());
	//simulationFromRealHand();
#endif
}

/*! Queries the real hand for finger posture and breakaway status and
	replicates them on the simulated hand.
*/
void BarrettHandDlg::simulationFromRealHand()
{
#ifdef HARDWARE_LIB
	if(withinSimulationUpdate || userInteractionOngoing) return;
	withinSimulationUpdate = true;

	int realVals[4];
	realVals[0] = mRealBarrett->GetFingerPosition(BARRETT_SPREAD);
	realVals[1] = mRealBarrett->GetFingerPosition(BARRETT_FINGER1);
	realVals[2] = mRealBarrett->GetFingerPosition(BARRETT_FINGER2);
	realVals[3] = mRealBarrett->GetFingerPosition(BARRETT_FINGER3);

	if(mRealBarrett->GetBreakawayDetected(BARRETT_FINGER1)) {
		int breakawayPosition = mRealBarrett->GetBreakawayPosition(BARRETT_FINGER1);
		fprintf(stderr, "Finger 1 breakaway at %i\n", breakawayPosition);
	}
	if(mRealBarrett->GetBreakawayDetected(BARRETT_FINGER2)) {
		int breakawayPosition = mRealBarrett->GetBreakawayPosition(BARRETT_FINGER2);
		fprintf(stderr, "Finger 2 breakaway at %i\n", breakawayPosition);
	}
	if(mRealBarrett->GetBreakawayDetected(BARRETT_FINGER3)) {
		int breakawayPosition = mRealBarrett->GetBreakawayPosition(BARRETT_FINGER3);
		fprintf(stderr, "Finger 2 breakaway at %i\n", breakawayPosition);
	}

	double dofVals[4];
	dofVals[0] = mRealBarrett->InternalToRadians(BARRETT_SPREAD, realVals[0]);
	dofVals[1] = mRealBarrett->InternalToRadians(BARRETT_FINGER1, realVals[1]);
	dofVals[2] = mRealBarrett->InternalToRadians(BARRETT_FINGER2, realVals[2]);
	dofVals[3] = mRealBarrett->InternalToRadians(BARRETT_FINGER3, realVals[3]);
	mSimBarrett->forceDOFVals(dofVals);

	withinSimulationUpdate = false;
#endif
}

/*! Sends the joints of the real hand to the positions of the joints
	of the simulated hand. Breakaway is not always replicated, as it 
	would need a physical obstacle to stop the movement of the real hand.
*/
void BarrettHandDlg::realHandFromSimulation()
{
#ifdef HARDWARE_LIB
	if(withinSimulationUpdate || userInteractionOngoing) return;
	withinSimulationUpdate = true;

	double dofVals[4];
	mSimBarrett->getDOFVals(dofVals);

/*
	QLineEdit *stepSize = (QLineEdit *) this->child("stepSize", "QLineEdit");
	bool stepSizeValid;
	double degrees = stepSize->text().toInt(&stepSizeValid, 10);
	if(!stepSizeValid) degrees = 15;
	double radians = degrees * 3.14159 / 180.0;
	mRealBarrett->MoveMultiple(dofVals, radians);
*/
	mRealBarrett->MoveTogether(dofVals);
	withinSimulationUpdate = false;
#endif
}

/*! When continous opperation is on, the real Barrett hand automatically
	mimics all movement of the simulated hand that is done by the user.
*/
void BarrettHandDlg::toggleContinuousOperation()
{
	QString *text;

	if(!simulatedHandForContinuousOperation) {
		QObject::connect(mSimBarrett, SIGNAL(configurationChanged()), this, SLOT(realHandFromSimulation()));
		QObject::connect(mSimBarrett, SIGNAL(userInteractionStart()), this, SLOT(userInteractionStart()));
		QObject::connect(mSimBarrett, SIGNAL(userInteractionEnd()), this, SLOT(userInteractionEnd()));
		simulatedHandForContinuousOperation = mSimBarrett;
		text = new QString("End continuous operation");
	} else {
		QObject::disconnect(simulatedHandForContinuousOperation, 0, this, 0);
		simulatedHandForContinuousOperation = NULL;
		text = new QString("Begin continuous operation");
	}

	QPushButton *button = (QPushButton *) this->child("continuousOperationButton", "QPushButton");
	button->setText(*text);
	delete text;
}

/*! Called when the user has clicked a dragger on the simulated hand
	and has started draggin it around. In continous update mode, we 
	actually wait until the user has released the dragger to set the
	real hand, so that we don't send a lots of tiny commands to the 
	real motors while the user is dragging the mouse.
*/
void BarrettHandDlg::userInteractionStart()
{
	userInteractionOngoing = true;
}

void BarrettHandDlg::userInteractionEnd()
{
	userInteractionOngoing = false;
	realHandFromSimulation();
}

/*! Sets a set of parameters to the real Barrett hand which are intended
	to produce smoother acceleration in the spread DOF. Needs to be called
	each time after the hand is initialized, as initialization sets the
	default values which produce jerky motion.
*/
void BarrettHandDlg::smoothButton_clicked()
{
#ifdef HARDWARE_LIB
	mRealBarrett->smoothSpreadParams();
#endif
}

void BarrettHandDlg::initSpreadButton_clicked()
{
#ifdef HARDWARE_LIB
	mRealBarrett->HandInitialize(BARRETT_SPREAD);
#endif
}
