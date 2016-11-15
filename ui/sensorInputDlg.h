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
// $Id: sensorInputDlg.h,v 1.3 2009/05/11 13:58:26 cmatei Exp $
//
//######################################################################

#ifndef _sensorinputdlg_h_
#define _sensorinputdlg_h_

#include <QDialog>

#include "matvec3D.h"

class SoSensor;
class SoTimerSensor;
class Flock;
class CyberGlove;
class World;

#include "ui_sensorInputDlg.h"

//! A dialog that allows the world elements to be controlled by a Cyberglove of Flock of Birds
/*! This is the main interface between the GraspIt world and external sensors
	such as the Flock of Birds or Cyberglove. It uses an internal timer to
	wake up periodically and query the sensors that have been enabled by the
	user. It then sets the positions or postures of the world elements that
	are controlled by the sensor.

	The Flock of Birds has multiple modes of operation and for some of those
	we must keep track of base transforms that all other transforms are computed
	relative to. This is also the job of this dialog. See the \a FlockMode enum
	for possible running modes for the Flock.
*/
class SensorInputDlg : public QDialog, public Ui::SensorsDlgUI
{
	Q_OBJECT
public:
	//! The possible running modes of the Flock of Birds
	/*! ABSOLUTE: each object controlled by a bird gets exactly the transform of 
		the bird read from the sensor. The base of the Flock of Birds becomes the
		origin of the world coordinate system.

		RELATIVE: one of the birds in the Flock is considered to be the master bird.
		The objects run by the master bird do not change their positions when we
		turn on the flock. Rather, all subsequent movement of the bird is interpreted
		as relative to the starting position. All objects run by other birds than 
		the master are repositioned relative to the master. In this way, distances
		between different birds in the real world are kept in the GraspIt! world.

		CAMERA: just as in RELATIVE mode, except that the master bird controls the
		position of the camera in GraspIt.
	*/
	enum FlockMode{FLOCK_RELATIVE, FLOCK_ABSOLUTE, FLOCK_CAMERA};
private:
	//! The world that this dialog is connected to
	World *mWorld;
	//! Shows if the flock is currently being used
	bool mFlockRunning;
	//! Shows if the glove is currently being used
	bool mGloveRunning;
	//! The timer that fires at discrete intervals so we can update the readings
	SoTimerSensor *mTimerSensor;
	//! Interface to the raw Flock of Birds
	Flock *mFlock;
	//! Interface to raw Cyberglove
	CyberGlove *mGlove;
	//! The current running mode of the Flock
	FlockMode mFlockMode;
	//! Keeps track of the camera flock transform, for the CAMERA flock mode of operation
	FlockTransf mCameraFlockTran;
	//! The number of birds in the Flock. For now, it's hard-coded in.
	int mNumBirds;
	//! Which bird is the master bird, the one that evertyhing else is set relative to
	int mMasterBird;

	//! Acts as a constructor, called on initialization
	void init(World *w);
	//! Initialized the flock of birds
	bool initFlock();
	//! Initializes the Cyberglove
	bool initGlove();

	//! The static version of the timer callback, just calls the internal version
	static void timerStaticCB(void *data, SoSensor*);
	//! Called when the timer sensor fires
	void timerInternalCB();

	//! The actual processing of a new glove reading
	void processGlove();

	//! Moves bodies to the position set by the Flock, resolving collisions and contacts
	void processFlockBodies(std::vector<transf> &birdTransf);
	//! If the mode of operation requires it, sets the camera based on the flock
	void processFlockCamera(std::vector<transf> &birdTransf);
	//! Moves robots to the position set by the Flock, resolving collisions and contacts
	void processFlockRobots(std::vector<transf> &birdTransf);

	//! Attempts to find a legal grasp in the vicinity of the state set by glove and flock
	void flockAndGloveFuzzyGrasp(std::vector<transf> &birdTransf);

	//! Helper function that reads in a raw bird transform and returns it as a GraspIt transf.
	transf SensorInputDlg::getBirdTran(int b);

private slots:
	//! Resets all relative positions that new flock positions are computed relative to
	void resetFlock();

public:
	SensorInputDlg(World *w, QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButton_clicked()));
		QObject::connect(gloveStartButton, SIGNAL(clicked()), this, 
						 SLOT(gloveStartButton_clicked()));
		QObject::connect(flockStartButton, SIGNAL(clicked()), this, 
						 SLOT(flockStartButton_clicked()));
		QObject::connect(resetFlockButton, SIGNAL(clicked()), this, 
						 SLOT(resetFlock()));
		init(w);
	}

public slots:
	//! Stops the timers and cleans up
	void exitButton_clicked();
	//! Starts or stops the Cyberglove use
	void gloveStartButton_clicked();
	//! Starts or stops the Flock of Birds use
	void flockStartButton_clicked();
};

#endif
