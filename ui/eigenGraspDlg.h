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
// $Id: eigenGraspDlg.h,v 1.4 2009/03/30 20:42:12 cmatei Exp $
//
//######################################################################

#ifndef _eigengraspdlg_h_
#define _eigengraspdlg_h_

#include "ui_eigenGraspDlg.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QDialog>
#include <vector>

class Hand;
class EigenGraspInterface;
class World;
class QLabel;
class QScrollBar;
class QCheckBox;
class QVBoxLayout;

//! The eigengrasp dialog allows the user to see hand motion along eigengrasp dimensions
/*! This is the ui for eigengrasp movement of a hand. The main functionality
	is that the user can inspect hand motion along any eigengrasp dimension.
	However, collisions are NOT checked, either with the hand itself or with
	external object.

	The principal interface is a set of sliders, one for each eigengrasp 
	dimension, that the user can manipulate. The user can also decide if the
	eigengrasp interface should be "rigid" or not, see the EigenGraspInterface
	class documentation for details.

	The user can also manipulate individual dof's as usual in GraspIt and see the
	projection of the set pose in eg space. It is also possible to make the eg
	space rigid, in which case if an individual dof is moved, the dialog will 
	project the resulting pose in eg space, then project the result back in dof
	space and use it to set robot posture. In this mode, even if the user 
	manipulates individual dof's, the dialog will try to find the closest
	posture in eg space that matches the desired dof posture.

	Whenever the posture of the robot is changed, this will also attempt to
	re-compute the min and max values along each eigengrasp and set slider
	limits accordingly. This unfortunately has never worked very well.

	In this dialog it is also possible to load a different set of eigengrasps
	from a file, or to set the trivial (identity) eigengrasp set. Therefore, the
	sliders themselves are not shown in the main window, but rather in a "slave"
	dialog window which is killed and reconstructed each time the eigengrasp
	interface changes.
*/
class EigenGraspDlg : public QDialog, public Ui::EigenGraspDlgUI
{
	Q_OBJECT
private:
	//! The resolution of the sliders, set through Qt
    int SLIDER_STEPS;
	//! Converts slider units to eigengrasp amplitudes
    double mSliderConversion;
	//! The number of eigengrasps in the interface, and thus the number of sliders in the dialog
	int mNumberGrasps;

    double *mLegalAmplitudes;
	//! The hand that this dialog is controlling
    Hand* mHand;
	//! The eigengrasp interface of the current hand
    EigenGraspInterface* mEigenGrasps;
	//! The world that the hand belongs to
    World *mWorld;

	//! Holds all the eg sliders and any controls that must exist for each eg.
	QDialog *mSlave;
    std::vector<QLabel*> mValueList;
    std::vector<QScrollBar*> mBarList;
    std::vector<QCheckBox*> mCheckList;
    QVBoxLayout *mainLayout;

	void init();
	void destroy();
	//! Kills and reconstructs the slave dialog, when the eigengrasp interface changes
	void resetSlave();
	//! Populates the slave with the right number of sliders, controls, etc.
	void setSlaveLayout( int nGrasps );

	//! Attempts to compute the mSliderConversion factor based on the legal range of motion of all eg's
	void adjustSliders();
	//! Sets the sliders to match the set of amplitudes given in \a amp
	void setAmplitudes(double *amp);

	//! Sets the hand posture to the origin of the eg subspace
	void goToOrigin();
public:
	EigenGraspDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	~EigenGraspDlg(){destroy();}
	int setWorld( World *w );

public slots:
	void eigenGraspChanged();
	void fixBoxChanged();
	void saveButton_clicked();
	void loadButton_clicked();
	void identityButton_clicked();
	void exitButton_clicked();
	void show();
	//! Called when the user changes a dof value directly
	void handConfigurationChanged();
	//! Sets the current dof posture to be the origin of the eg subspace
	void setOriginButton_clicked();
	//! Sets the "rigid" mode of the EigenGraspInterface; see that class for details
	void rigidCheckBox_clicked();
	//! Obsolete; has been removed
	void closeHandButton_clicked();
	void goToOriginButton_clicked();
};

#endif
