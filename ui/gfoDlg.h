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
// $Id: gfoDlg.h,v 1.3 2009/08/17 22:17:32 cmatei Exp $
//
//######################################################################

#ifndef _gfodlg_h_
#define _gfodlg_h_

#include "ui_gfoDlg.h"
#include <QDialog>

class Hand;
class MainWindow;

//! Provides an interface for running Grasp Force Optimization routines
/*! This dialog allows running some of the Grasp Force Optimization routines
	and display the results for visual inspection. The user can select the 
	type of optimization to be performed; see the Grasp class for more 
	details about the differences between these routines. If the optimization
	is enabled, the dialog will monitor the configuration of the hand and 
	run the optimization after each change.

	The contact forces that are computed through the optimization, as well
	as the resultant object wrench, are displayed visually through contact
	force indicators at each contat plus an object wrench at the object
	origin. The contacts, as well as their forces, are also displayed in
	the contact details area in the main window. The user can click on a 
	contact in this are to cause it's associated force indicator to blink.
*/
class GFODlg : public QDialog, public Ui::GFODlgUI
{
	Q_OBJECT
private:
	//! The hand that is the target of the optimization
	Hand *mHand;
	//! A pointer to the main window used to ask for an update of the contact list
	MainWindow *mMainWindow;
	//! Calls the GFO routine that computes both contact wrenches and joint torques
	void graspForceOptimization();
	//! Calls the routine that computes contact forces that balance compliant joints
	void compliantEquilibriumOptimization(bool useDynamicForce);
	//! Calls the tendon route optimization specific to the McGrip hand
	void tendonRouteOptimization();
	//! Checks if analytical tendon model correctly predicts equilibrium for McGrip
	void mcgripEquilibrium();

	//! Calls the ivmgr to display the forces and wrenched set by the optimization
	void displayResults(int result);
	//! Decides which optimization routine to run bases on what is selected in the combo box
	void runOptimization();
public:
	GFODlg(MainWindow *mw, Hand *h, QWidget *parent=0);
	~GFODlg();
public slots:
	void exitButtonClicked(){QDialog::accept();}
	void optimizationOnBoxClicked();
	//! Performs the optimization routine after each hand config change
	void handConfigurationChanged();
};


#endif