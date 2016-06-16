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
// $Id: optimizerDlg.cpp,v 1.3 2009/07/02 22:06:55 cmatei Exp $
//
//######################################################################

#include "optimizerDlg.h"

#include "eigenTorques.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"

#include "debug.h"

OptimizerDlg::OptimizerDlg(World *w, QWidget *parent) : QDialog(parent), mWorld(w) 
{
	setupUi(this);
	QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButtonClicked()));
	QObject::connect(torqueButton, SIGNAL(clicked()), this, SLOT(torqueButtonClicked()));
}

void
OptimizerDlg::torqueButtonClicked()
{
#ifdef CGDB_ENABLED
	if (!mWorld->getCurrentHand()) {
		DBGA("No hand selected");
		return;
	}
	if (!graspItGUI->getIVmgr()->getDBMgr()) {
		DBGA("Connect to database first");
		return;
	}
	CGDBGraspProcessor eigenTorques(mWorld->getCurrentHand());
	DBGA("CGDBGraspProcessor constructed");
	eigenTorques.run();
	DBGA("CGDBGraspProcessor done");
#else
	DBGA("Optimizer needs the CGDB");
#endif
}
