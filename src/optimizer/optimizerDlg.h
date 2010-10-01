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
// $Id: optimizerDlg.h,v 1.1 2009/06/17 17:50:28 cmatei Exp $
//
//######################################################################

#ifndef _optimizerdlg_h_
#define _optimizerdlg_h_

#include "ui_optimizerDlg.h"
#include <QDialog>

class World;

class OptimizerDlg : public QDialog, public Ui::OptimizerDlgUI
{
	Q_OBJECT
private:
	World *mWorld;
public:
	OptimizerDlg(World *w, QWidget *parent=0);
public slots:
	void torqueButtonClicked();
	void exitButtonClicked(){QDialog::accept();}
};

#endif