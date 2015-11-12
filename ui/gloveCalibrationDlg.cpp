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
// Author(s): Matei T. Ciocarlie
//
// $Id: gloveCalibrationDlg.cpp,v 1.3 2009/03/30 20:42:12 cmatei Exp $
//
//######################################################################

#include "gloveCalibrationDlg.h"

#include <QFileDialog>

#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "gloveInterface.h"

void GloveCalibrationDlg::init()
{
//	calibrateButton->setEnabled(false);

	World *w=graspItGUI->getIVmgr()->getWorld();
	mInterface = w->getCurrentHand()->getGloveInterface();

	mInterface->saveRobotPose();

	calibrationTypeBox->insertItem( "Fist" );
	calibrationTypeBox->insertItem( "Simple thumb" );
	calibrationTypeBox->insertItem( "Complex thumb" );
	calibrationTypeBox->insertItem( "Abd. - Add." );
	calibrationTypeBox->insertItem( "Mean Pose" );
	calibrationTypeBox->setCurrentItem(2);
    mInterface->initCalibration( GloveInterface::COMPLEX_THUMB );

	poseDistanceBox->insertItem("0");
	poseDistanceBox->insertItem("17");
	poseDistanceBox->insertItem("61");
	poseDistanceBox->insertItem("82");
	poseDistanceBox->setCurrentItem(0);

	update();
}

void GloveCalibrationDlg::prevPose()
{
	mInterface->nextPose(-1);
	mInterface->showCurrentPose();
	update();
}


void GloveCalibrationDlg::nextPose()
{
	mInterface->nextPose( 1);
	mInterface->showCurrentPose();
	update();
}


void GloveCalibrationDlg::record()
{
	int d = poseDistanceBox->currentText().toInt();
	mInterface->recordPoseFromGlove(d);
	mInterface->showCurrentPose();
	update();
}

void GloveCalibrationDlg::update()
{
	if ( mInterface->poseSet() ){
		textLabel2->setText("Pose recorded");

	} else {
		textLabel2->setText("No values recorded");
	}

	if ( mInterface->readyToCalibrate() ) {
		calibrateButton->setEnabled(true);
	} else
		calibrateButton->setEnabled(false);

	numberPosesLabel->setNum( mInterface->getNumPoses() );
}

void GloveCalibrationDlg::calibrate()
{
	mInterface->performCalibration();
	update();
}


void GloveCalibrationDlg::save()
{
	QString fn( QFileDialog::getSaveFileName( this, QString(), QString(getenv("GRASPIT"))+QString("/models/CyberGlove"),
				"Glove Pose Files (*.txt)") );
    if ( !fn.isEmpty() ) {
	    if (fn.section('.',1).isEmpty())
			fn.append(".txt");
		mInterface->saveCalibration( fn.latin1() );
	}
}


void GloveCalibrationDlg::done()
{
// mInterface->revertRobotPose();
	QDialog::accept();
}


void GloveCalibrationDlg::savePoses()
{
	QString fn( QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT"))+QString("/models/CyberGlove"),
				"Glove Pose Files (*.pos)" ) );
    if ( !fn.isEmpty() ) {
	    if (fn.section('.',1).isEmpty())
			fn.append(".pos");
		mInterface->saveCalibrationPoses( fn.latin1() );
	}
}


void GloveCalibrationDlg::loadPoses()
{
	QString fn( QFileDialog::getOpenFileName(this, QString(), QString(getenv("GRASPIT"))+QString("/models/CyberGlove"),
			    "Glove Pose Files (*.pos)") );
	if ( !fn.isEmpty() ) {
		mInterface->loadCalibrationPoses( fn.latin1() );
	}
	update();
}

void GloveCalibrationDlg::loadCalibration()
{
	QString fn( QFileDialog::getOpenFileName(this, QString(), QString(getenv("GRASPIT"))+QString("/models/CyberGlove"),
		        "Glove Pose Files (*.txt)") );
    if ( !fn.isEmpty() ) {
		mInterface->loadCalibration( fn.latin1() );
	}
}


void GloveCalibrationDlg::initCalibration()
{
	switch( calibrationTypeBox->currentItem() ) {
	case 0:
		mInterface->initCalibration( GloveInterface::FIST );
		break;
	case 1:
		mInterface->initCalibration( GloveInterface::SIMPLE_THUMB );
		break;
	case 2:
		mInterface->initCalibration( GloveInterface::COMPLEX_THUMB );
		break;
	case 3:
		mInterface->initCalibration( GloveInterface::ABD_ADD );
		break;
	case 4:
		mInterface->initCalibration( GloveInterface::MEAN_POSE );
		break;
	default:
		break;
	}
}


void GloveCalibrationDlg::clearPoses()
{
	mInterface->clearPoses();
	update();
}


void GloveCalibrationDlg::startGlove()
{
	mInterface->startGlove();
}
