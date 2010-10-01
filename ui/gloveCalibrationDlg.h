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
// $Id: gloveCalibrationDlg.h,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _glovecalibrationdlg_h_
#define _glovecalibrationdlg_h_

#include "ui_gloveCalibrationDlg.h"

#include <QDialog>

class GloveInterface;
class CalibrationPose;

//! Allows the user to record hand postures using the CyberGlove that can be used for calibration
/*! The GloveCalibrationDlg provides a UI to the calibration part of the 
	GloveInterface class. The calibration routines are very specific, 
	and a more general calibration interface would be desirable. For now,
	more details can be found in the documentation to the GloveInterface 
	class.

	Allows 5 types of computations:
	- simple palm extended vs. fist calibration for finger flexion
	- a simple or a more complex calibration routine for thumb joints
	- a simple calibration routine for finger abduction / adduction
	- computation of the "mean" of a set of postures

	Can also be used to save and load the result of the calibration. A 
	saved calibration can be specified in a robot .cfg file to be used
	later with the CyberGlove. Can also save just the calibration
	postures (as opposed to the result of the calibration itself), so
	that calibration can be debugged without having to record poses
	each time.
*/
class GloveCalibrationDlg : public QDialog, public Ui::GloveCalibrationDlgUI
{
	Q_OBJECT
private:
    GloveInterface *mInterface;
    CalibrationPose *mCurrentPose;

	void init();
	void update();

public:
	GloveCalibrationDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}

public slots:
	void prevPose();
	void nextPose();
	void record();
	void calibrate();
	void save();
	void done();
	void savePoses();
	void loadPoses();
	void loadCalibration();
	void initCalibration();
	void clearPoses();
	void startGlove();
};

#endif
