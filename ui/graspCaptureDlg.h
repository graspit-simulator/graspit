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
// $Id: graspCaptureDlg.h,v 1.12 2009/07/02 21:28:14 cmatei Exp $
//
//######################################################################

#ifndef _graspcapturedlg_h_
#define _graspcapturedlg_h_

#include <QDialog>
#include <list>

class World;
class GraspPlanningState;
class QualEpsilon;
class QualVolume;
class Hand;

#include "ui_graspCaptureDlg.h"
#include "ui_qualityIndicator.h"

//! A small progress bar that can show grasp quality on the screen
/*! This is a tiny window that only contains a progress bar; it is used
	to show grasp quality in a simple and unobtrusive way, particularly
	when doing grasp recording studies with human subjects.
*/
class QualityIndicator : public QDialog, public Ui::QualityIndicatorUI
{
public:
	QualityIndicator(QWidget *parent=0) : QDialog(parent) {
		setupUi(this);
		qualityBar->setMinimum(0);
		qualityBar->setMaximum(100);
	}
	void setBar(double value) {
		qualityBar->setValue((value/0.25)*100);
		if (value > 0.0){
			fcLabel->setEnabled(TRUE);
		} else {
			fcLabel->setEnabled(FALSE);
		}
	}
};

//! Allows the user to record grasps and save them
/*! This dialog allows the user to record grasps that have been created
	in the main GraspIt! window and then save them. For now, grasps are
	recorded as HandObjectState. Grasps can be saved to files (no dialog
	for loading them back and showing them exists yet though). If the
	CGDB is enabled, grasps can also be saved to the CGDB, with a 
	tag that will distinguish them from grasps created in different
	ways. They can then be played back with the usual CGDB browser.

	The CGDB save option will be enabled only if a sucessful connection
	to the CGDB has already been opened (use the Connect and Browse menu
	for that). If not, only the option to save grasps to a file, in some
	arbitrary ascii format, is available.

	This is still very crude, and some of the parameters of the grasps
	that are stored in the CGDB are hard-coded in rather than input by
	the user through the dialog. However, it can serve as a starting
	point if you need to write better code for saving grasps to the 
	database.

	The class manages its own quality measures so that we make sure
	we have consistency between runs, and also to make sure we have the
	exact same data that we store in the CGDB for other grasps.
*/
class GraspCaptureDlg : public QDialog, public Ui::GraspCaptureDlgUI
{
	Q_OBJECT
private:
	//! The GraspIt world that this dialog is connected to
	World *mWorld;
	//! The hand that the current quality metrics point to
	Hand *mCurrentHand;
	//! The little window that shows the grasp quality
	QualityIndicator *mIndicator;
	//! Stores the grasps captured so far
	std::list<GraspPlanningState*> mGrasps;

	//! The epsilon quality measure used for grasps
	QualEpsilon *mQualEpsilon;
	//! The volume quality measure used for grasps
	QualVolume *mQualVolume;

	//! Updates the label that shows the number of grasps captured
	void updateNumGrasps();
	//! Called by the constructor
	void init(World *w);
	//! Checks which hand is currently selected and updates the quality measures accordingly
	/*! If no hand, or no object are selected, returns false. If a hand is
		selected and an object is set as the target of its grasp, it makes
		sure the quality metrics refer to the correct hand and returns 
		true.
	*/
	bool checkHandSelection();
private slots:
	void captureButtonClicked();
	void saveToFileButtonClicked();
	void saveToDBaseButtonClicked();
	void clearListButtonClicked();
	void exitButtonClicked(){QDialog::accept();}
public:
	GraspCaptureDlg(World *w, QWidget *parent=0) : QDialog(parent) {
		setupUi(this);
		init(w);
		QObject::connect(captureButton, SIGNAL(clicked()), this, SLOT(captureButtonClicked()));
		QObject::connect(saveToFileButton, SIGNAL(clicked()), this, SLOT(saveToFileButtonClicked()));
		QObject::connect(saveToDBaseButton, SIGNAL(clicked()), this, SLOT(saveToDBaseButtonClicked()));
		QObject::connect(clearListButton, SIGNAL(clicked()), this, SLOT(clearListButtonClicked()));
		QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButtonClicked()));
	}
	~GraspCaptureDlg();
public slots:
	void updateQuality();
};

#endif
