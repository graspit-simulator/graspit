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
// $Id: barrettHandDlg.h,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _barretthanddlg_h_
#define _barretthanddlg_h_

#include "ui_barrettHandDlg.h"

#include <QDialog>

class Barrett;
class BarrettHandThread;
class BarrettHand;
class World;
class Robot;

//! Interfaces GraspIt with a real Barrett hand
/*! This dialog provides an interface between a simulated Barrett hand
	in GraspIt and a real Barrett hand. The interface goes in both ways:
	the real Barrett hand can me instructed to move like the simulated
	one, or the simulated hand can replicate the posture of the real
	one.
*/
class BarrettHandDlg : public QDialog, public Ui::BarrettHandDlgUI
{
	Q_OBJECT
private:
	//! Shows if the user is currently interacting with the simulated hand
    bool userInteractionOngoing;
	//! Shows if we are currently in the process of interacting with the real hand
	bool withinSimulationUpdate;

    Robot *simulatedHandForContinuousOperation;
	//! The interface to the real Barrett hand
    BarrettHand *mRealBarrett;
	//! For multi-threaded interface with a real hand. This is not complete.
    BarrettHandThread *mBHT;
	//! The simulated Barrett hand in GraspIt
    Barrett *mSimBarrett;
	//! The world that the simulated hand belongs to 
    World *world;

	//! Dialog initialization and button connections
	void init();
	//! Called when the user starts interacting with the simulated hand
	void userInteractionStart();
	//! Called when the user stops interacting with the simulated hand
	void userInteractionEnd();

public:
	//! Initializes the dialog
	BarrettHandDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}

	//! Sets the world and extracts from it all the members of this dialog
	bool setWorld( World *w );
	
public slots:
	//! Calls the initialization procedure on the real hand
	void initializeHand();
	//! Makes the simulated hand mimic the posture of the real hand
	void simulationFromRealHand();
	//! Updates the real hand based on the posture of the simulated hand
	void realHandFromSimulation();
	//! In continous operation, the real hand follows all motion of the simulated hand
	void toggleContinuousOperation();
	//! Sets motor parameters on the real hand to produce smoother motion in the spread DOF
	void smoothButton_clicked();
	//! Re-initializes the spread angle DOF of the real hand
	void initSpreadButton_clicked();
};

#endif
