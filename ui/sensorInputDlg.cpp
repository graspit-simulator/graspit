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
// $Id: sensorInputDlg.cpp,v 1.5 2009/05/12 14:16:15 cmatei Exp $
//
//######################################################################

#include "sensorInputDlg.h"

#include <assert.h>
#include <Inventor/sensors/SoTimerSensor.h>

#include "robot.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "body.h"
#include "barrett.h"
#include "Flock.h"
#include "CyberGlove.h"
#include "gloveInterface.h"
#include "collisionStructures.h"

//#define GRASPITDBG
#include "debug.h"

/*! Acts as a contructor. Sets the default Flock mode (RELATIVE) and 
	default master bird (1).
*/
void 
SensorInputDlg::init(World *w)
{
	mWorld = w;
	mGloveRunning = false;
	mFlockRunning = false;
	mFlock = NULL;
	mGlove = NULL;
	mTimerSensor = new SoTimerSensor(timerStaticCB, this);
	//timer interval, in seconds
	mTimerSensor->setInterval( SbTime(0.05) );
	mNumBirds = 1;
	flockModeBox->insertItem("Relative");
	flockModeBox->insertItem("Camera");
	flockModeBox->insertItem("Absolute");
	masterFlockBox->setMinimum(1);
	masterFlockBox->setMaximum(9);
	masterFlockBox->setValue(1);
	mMasterBird = 1;
	mFlockMode = FLOCK_RELATIVE;
}

void 
SensorInputDlg::timerStaticCB(void *data, SoSensor*)
{
	SensorInputDlg *dlg = static_cast<SensorInputDlg*>(data);
	dlg->timerInternalCB();
}

/*! Refreshes all sensor readings, so that is only done once. Then, is
	calls the appropriate processing routines, based on the operation mode
	selected by the user. If any sensor is active it also reschedules the 
	timer.
*/
void 
SensorInputDlg::timerInternalCB()
{
	bool needed = false;
	if (mGloveRunning) {
		if (!mGlove->instantRead()) {
			DBGA("Error reading CyberGlove\n");
			gloveStartButton_clicked();
		} else {
			if (!mFlockRunning) processGlove();
			needed = true;
		}
	}
	if (mFlockRunning) {
		bool process = true;
		// If the Flock is attached to a real Barrett, we ignore readings
		// while the motors of the Barrett are active, as they corrupt the bird.
		if ( mWorld->getCurrentHand() && mWorld->getCurrentHand()->isA("Barrett") ) {
			if ( ((Barrett*)mWorld->getCurrentHand())->isBusy() ) process = false;
		}
		if (process) {
			//read in transforms from all the birds in the system
			std::vector<transf> birdTransf(mNumBirds+1);
			for (int i=1; i<=mNumBirds; i++) {
				birdTransf[i] = getBirdTran(i);
			}
			processFlockCamera(birdTransf);
			processFlockBodies(birdTransf);
			if (!mGloveRunning) {
				processFlockRobots(birdTransf);
			} else {
				flockAndGloveFuzzyGrasp(birdTransf);
			}
		}
		needed = true;
	}
	if (!needed) {
		mTimerSensor->unschedule();
	}
}

/*! Initialized the Flock of Birds. For now, no check is done to see
	if the initialization was successful.
*/
bool
SensorInputDlg::initFlock()
{
	mFlock = new Flock(1);
	return true;
}

/*! Inits the raw Glove, checks if initialization was successful. Then,
	it passes the raw glove to all the cyberglove interfaces of individual
	robots. The interfaces of each robot know how to translate glove
	information to DOF values.
*/
bool
SensorInputDlg::initGlove()
{
	mGlove = new CyberGlove();
	if (!mGlove->testGlove()) {
		delete mGlove; mGlove = NULL;
		return false;
	}
	for ( int i=0; i<mWorld->getNumRobots(); i++) {
		if (mWorld->getRobot(i)->useCyberGlove()) {
			mWorld->getRobot(i)->setGlove(mGlove);
		}
	}	
	return true;
}

void 
SensorInputDlg::exitButton_clicked()
{
	if (mFlock) delete mFlock;
	if (mGlove) delete mGlove;
	delete mTimerSensor;
	QDialog::accept();
}

/*! If the raw glove has not yet been initialized, it initializes it.
	Also schedules the timer.
*/
void 
SensorInputDlg::gloveStartButton_clicked()
{
	if (!mGloveRunning) {
		if (!mGlove) {
			if (!initGlove()) {
				DBGA("Glove init failed");
				return;
			}
		}
		mGloveRunning = true;
		if (!mTimerSensor->isScheduled()) {
			mTimerSensor->schedule();
		}
		gloveStartButton->setText("Stop");
	} else {
		mGloveRunning = false;
		gloveStartButton->setText("Start");
	}
}

/*! If the raw flock has not yet been initialized, it initializes it.
	When this is called *for the first time* it calls the reset function 
	of this dialog to set base positions and reference transforms. 
	Subsequent calls no longer trigger a reset, allowing the user to reset
	only when he/she wants to. This not ideal from a UI perspective, but
	it is what we have so far.
*/
void 
SensorInputDlg::flockStartButton_clicked()
{
	if (!mFlockRunning) {
		if (!mFlock) {
			if (!initFlock()) {
				DBGA("Flock init failed");
				return;
			}
			//on first execution we also reset the flock which sets the reference
			//transforms for all the bodies in the world
			resetFlock();
		}
		mFlockRunning = true;
		if (!mTimerSensor->isScheduled()) {
			mTimerSensor->schedule();
		}
		flockStartButton->setText("Stop");
		flockModeBox->setEnabled(FALSE);
		masterFlockBox->setEnabled(FALSE);
		resetFlockButton->setEnabled(FALSE);
	} else {
		mFlockRunning = false;
		flockStartButton->setText("Start");
		flockModeBox->setEnabled(TRUE);
		masterFlockBox->setEnabled(TRUE);
		resetFlockButton->setEnabled(TRUE);
	}
}

transf 
SensorInputDlg::getBirdTran(int b)
{
	if (!mFlock->instantRead(b) ) {
		DBGA("Error reading Flock!");
		mFlockRunning = false;
		flockStartButton->setText("Start");
		return transf::IDENTITY;
	}
	
	double r[9], t[3];
	mFlock->getRotationMatrix(r);
	mFlock->getPosition(t);

	transf birdTran;
	birdTran.set(mat3(r), vec3(t));
	return birdTran;
}

/*!	Computes and sets the "base" tranforms for each object, which is 
	the transform that each later Bird transform will be computed
	relatiev to. This allows for multiple modes of operation for the
	Flock, see the definition of \a FlockMode for details.

	The Flock must be initialized and operational when this is called,
	as readings from the Flock are needed to set the base transforms.
*/
void
SensorInputDlg::resetFlock()
{
	assert(mFlock);

	//get the current transform of the master bird
	mMasterBird = masterFlockBox->value();
	if (flockModeBox->currentText()=="Relative") {
		mFlockMode = FLOCK_RELATIVE;
	} else if (flockModeBox->currentText()=="Camera") {
		mFlockMode = FLOCK_CAMERA;
	} else if (flockModeBox->currentText()=="Absolute") {
		mFlockMode = FLOCK_ABSOLUTE;
	};

	transf masterFlockTran = getBirdTran(mMasterBird);

	//for each body, set the base transform that it will be relative to
	if (mFlockMode == FLOCK_CAMERA) {
		//everything gets rearranged relative to the camera
		transf mount(mat3( vec3(0,1,0), vec3(0,0,-1), vec3(-1,0,0) ), vec3(0,0,0));
		mCameraFlockTran.setMount(mount.inverse());
		//the base is set based on current camera position
		transf cameraBaseTran = graspItGUI->getIVmgr()->getCameraTransf();
		mCameraFlockTran.setFlockBase(masterFlockTran);
		mCameraFlockTran.setObjectBase(cameraBaseTran);
		cameraBaseTran = mount.inverse() * cameraBaseTran;
		//process the bodies
		for (int i=0; i<mWorld->getNumBodies(); i++) {
			if (!mWorld->getBody(i)->usesFlock() || 
				mWorld->getBody(i)->getOwner() != mWorld->getBody(i) ) continue;
			//each body will be re-arranged relative to the camera transform
			mWorld->getBody(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getBody(i)->getFlockTran()->setObjectBase( cameraBaseTran );
		}
		//process the robots
		for (int i=0; i<mWorld->getNumRobots(); i++) {
			if ( !mWorld->getRobot(i)->usesFlock() ) continue;
			mWorld->getRobot(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getRobot(i)->getFlockTran()->setObjectBase( cameraBaseTran );
		}
	} else if (mFlockMode == FLOCK_RELATIVE) {
		//bodies run by the master bird remain relative to themselves
		//those that are not, will be relative to one of the bodies run by the master bird
		//which one of the bodies run by the master bird? for now, we just pick one at random
		//first, do the ones that are run by the master bird
		transf objectBaseTran = transf::IDENTITY;
		bool baseSet = false;
		//set the bodies
		for (int i=0; i<mWorld->getNumBodies(); i++) {
			if (!mWorld->getBody(i)->usesFlock() || 
				mWorld->getBody(i)->getOwner() != mWorld->getBody(i) ) continue;
			if (mWorld->getBody(i)->getBirdNumber()!=mMasterBird) continue;
			//pick the first body run by the master as a reference
			if (!baseSet) {
				objectBaseTran = mWorld->getBody(i)->getTran();
				baseSet = true;
			}
			//body will be relative to itself
			mWorld->getBody(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getBody(i)->getFlockTran()->setObjectBase( mWorld->getBody(i)->getTran() );
		}
		//set the robots
		for (int i=0; i<mWorld->getNumRobots(); i++) {
			if ( !mWorld->getRobot(i)->usesFlock() ) continue;
			if ( mWorld->getRobot(i)->getBirdNumber() != mMasterBird ) continue;
			if (!baseSet) {
				objectBaseTran = mWorld->getRobot(i)->getTran();
				baseSet = true;
			}
			mWorld->getRobot(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getRobot(i)->getFlockTran()->setObjectBase( mWorld->getRobot(i)->getTran() );
		}
		if (!baseSet) {
			DBGA("WARNING: nobody is run by master bird!");
		}
		//now do all those that are not run by the master bird
		for (int i=0; i<mWorld->getNumBodies(); i++) {
			if (!mWorld->getBody(i)->usesFlock() || 
				mWorld->getBody(i)->getOwner() != mWorld->getBody(i) ) continue;
			if (mWorld->getBody(i)->getBirdNumber()==mMasterBird) continue;
			mWorld->getBody(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getBody(i)->getFlockTran()->setObjectBase( objectBaseTran );
		}
		for (int i=0; i<mWorld->getNumRobots(); i++) {
			if ( !mWorld->getRobot(i)->usesFlock() ) continue;
			if ( !mWorld->getRobot(i)->getBirdNumber() == mMasterBird ) continue;
			mWorld->getRobot(i)->getFlockTran()->setFlockBase( masterFlockTran );
			mWorld->getRobot(i)->getFlockTran()->setObjectBase( objectBaseTran );
		}
	} else if (mFlockMode == FLOCK_ABSOLUTE) {
		//we have nothing to do here; each body will get the bird tran in absolute terms
	} else {
		DBGA("Unknown Flock mode requested!");
		assert(0);
	}
}

void
SensorInputDlg::processFlockCamera(std::vector<transf> &birdTransf)
{
	if (mFlockMode == FLOCK_CAMERA) {
		graspItGUI->getIVmgr()->setCameraTransf( mCameraFlockTran.get(birdTransf[mMasterBird]) );
	}
}

/*! For each body, computes an absolute transform in the GraspIt world 
	based on it. This computation depends on the mode of operation of the 
	Flock, and uses the	base transforms that have been previously set by 
	\a resetFlock()
*/
void
SensorInputDlg::processFlockBodies(std::vector<transf> &birdTransf)
{
	for (int i=0; i<mWorld->getNumBodies(); i++) {
		if (mWorld->getBody(i)->usesFlock() && 
			mWorld->getBody(i)->getOwner() == mWorld->getBody(i) ) {
			transf bodyTran;
			transf birdTran = birdTransf[mWorld->getBody(i)->getBirdNumber()];
			if (mFlockMode == FLOCK_ABSOLUTE) {	
				bodyTran = mWorld->getBody(i)->getFlockTran()->getAbsolute(birdTran);
			} else {
				bodyTran = mWorld->getBody(i)->getFlockTran()->get(birdTran);
			}
			mWorld->getBody(i)->moveTo(bodyTran , WorldElement::ONE_STEP, 
									   WorldElement::ONE_STEP );
		}
	}
}

void
SensorInputDlg::processFlockRobots(std::vector<transf> &birdTransf)
{
	for (int i=0; i<mWorld->getNumRobots(); i++) {
		if ( mWorld->getRobot(i)->usesFlock() ) {
			transf bodyTran;
			transf birdTran = birdTransf[mWorld->getRobot(i)->getBirdNumber()]; 
			if (mFlockMode == FLOCK_ABSOLUTE) {
				bodyTran = mWorld->getRobot(i)->getFlockTran()->getAbsolute( birdTran );
			} else {
				bodyTran = mWorld->getRobot(i)->getFlockTran()->get( birdTran );
			}
			mWorld->getRobot(i)->moveTo( bodyTran, WorldElement::ONE_STEP, 
										 WorldElement::ONE_STEP );
		}
	}
}

void
SensorInputDlg::processGlove()
{
	for ( int i=0; i<mWorld->getNumRobots(); i++) {
		if (mWorld->getRobot(i)->useCyberGlove()) {
			mWorld->getRobot(i)->processCyberGlove();
		}		
	}
}

void
SensorInputDlg::flockAndGloveFuzzyGrasp(std::vector<transf> &birdTransf) 
{
	for (int i=0; i<mWorld->getNumRobots(); i++) {
		Robot *robot = mWorld->getRobot(i);
		if (!robot->usesFlock() || !robot->useCyberGlove()) continue;
		//compute robot transform, same as in processFlockRobots
		transf robotTran;
		transf birdTran = birdTransf[mWorld->getRobot(i)->getBirdNumber()]; 
		if (mFlockMode == FLOCK_ABSOLUTE) {
			robotTran = mWorld->getRobot(i)->getFlockTran()->getAbsolute( birdTran );
		} else {
			robotTran = mWorld->getRobot(i)->getFlockTran()->get( birdTran );
		}
		//force hand in position
		robot->setTran(robotTran);
		//compute collisions only for the palm
		CollisionReport colReport, originalColReport;
		std::vector<Body*> interestList;
		interestList.push_back(robot->getBase());
		//if palm collision, back up until out of it
		transf collisionTran;
		bool collision = false;
		int steps = 0, maxSteps = 3;
		//we need to remember the original collision report for later when we interpolate
		int numCollisions = mWorld->getCollisionReport(&originalColReport, &interestList);
		while ( numCollisions && steps < maxSteps) {
			steps++;
			collision = true;
			collisionTran = robot->getTran();
			transf newTran = translate_transf(vec3(0,0,-10.0) * 
							 robot->getApproachTran()) * robot->getTran();
			robot->setTran(newTran);
			numCollisions = mWorld->getCollisionReport(&colReport, &interestList);
		}
		bool palmSuccess;
		if (steps == maxSteps) {
			//we have not managed to resolve the collision
			palmSuccess = false;
			robot->setTran(robotTran);
			DBGP("Can not resolve collision");
		} else if (collision) {
			//if we have resolved a collision, go back in until contact
			if (!robot->interpolateTo(robot->getTran(), collisionTran, originalColReport)) {
				DBGA("Palm interpolation failed");
				palmSuccess = false;
			} else {
				mWorld->findContacts(originalColReport);
				palmSuccess = true;
			}
		} else {
			//no collision from the beginning
			palmSuccess = true;
		}

		//read and prepare the glove dof values
		std::vector<double> dofVals(robot->getNumDOF(), 0.0);
		robot->getDOFVals(&dofVals[0]);
		for (int i=0; i<robot->getNumDOF(); i++) {
			if ( robot->getGloveInterface()->isDOFControlled(i) ) {
				dofVals[i] = robot->getGloveInterface()->getDOFValue(i);
			} 
		}
		robot->checkSetDOFVals(&dofVals[0]);
		//force the required dof values, no collision check
		robot->forceDOFVals(&dofVals[0]);

		if (!palmSuccess) {
			//we have failed with the palm; leave the chains like this and we are done
			DBGA("Palm failure");
			//mark the palm in red
			continue;
		}
		
		numCollisions = mWorld->getCollisionReport(&colReport);
		//for each chain, if collision snap to contacts
		bool success = true;
		for (int c=0; c<robot->getNumChains(); c++) {
			success = success && robot->snapChainToContacts(c, colReport);
		}
		if (success) {
			mWorld->updateGrasps();
		} else {
			//mark colliding bodies
			DBGP("Chain failure");
		}
	}
}