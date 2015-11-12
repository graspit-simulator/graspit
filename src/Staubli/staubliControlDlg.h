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
// Author(s):  Hao Dang
//
// $Id: staubliControlDlg.h,v 1.1 2009/09/14 18:04:41 hao Exp $
//
//######################################################################

/*! \file 
\brief Defines the %StaubliControlDlg class
*/

#pragma once
#ifndef STAUBLICONTROLDLG_H_
#define STAUBLICONTROLDLG_H_

#include "ui_staubliControlDlg.h"

#include <utility>
#include <QDialog>

class Robot;
class TX60L;
class Barrett;
class BarrettHand;

class StaubliControlDlg : public QDialog, public Ui::StaubliControlDlgUI
{
	Q_OBJECT

private:
	void init();
	void destroy();
	TX60L *mStaubliReal;
	BarrettHand *mBarrettReal;
	Robot *mStaubliSimulated;
	Barrett *mBarrettSimulated;

	void updateTX60L(std::vector<double> dofs);

public:
	StaubliControlDlg(QWidget *parent = 0) : QDialog(parent), mStaubliReal(NULL), mStaubliSimulated(NULL),
											 mBarrettReal(NULL), mBarrettSimulated(NULL){
		setupUi(this);
		QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButton_clicked()));
		QObject::connect(connectToStaubliButton, SIGNAL(clicked()), this, SLOT(connectToStaubliButton_clicked()));
		QObject::connect(getStaubliStatusButton, SIGNAL(clicked()), this, SLOT(getStaubliStatusButton_clicked()));
		QObject::connect(staubliGoButton, SIGNAL(clicked()), this, SLOT(staubliGoButton_clicked()));
		QObject::connect(initializeBarrettButton, SIGNAL(clicked()), this, SLOT(initializeBarrettButton_clicked()));
		QObject::connect(barrettGoButton, SIGNAL(clicked()), this, SLOT(barrettGoButton_clicked()));
		QObject::connect(getBarrettStatusButton, SIGNAL(clicked()), this, SLOT(getBarrettStatusButton_clicked()));
		QObject::connect(getLocButton, SIGNAL(clicked()), this, SLOT(getLocButton_clicked()));
		QObject::connect(barrettCloseButton, SIGNAL(clicked()), this, SLOT(barrettCloseButton_clicked()));
		QObject::connect(barrettOpenButton, SIGNAL(clicked()), this, SLOT(barrettOpenButton_clicked()));
		QObject::connect(getBarrettButton, SIGNAL(clicked()), this, SLOT(getBarrettButton_clicked()));
		init();
	}
	~StaubliControlDlg(){destroy();};

public slots:
	void exitButton_clicked();
	void connectToStaubliButton_clicked();
	void getStaubliStatusButton_clicked();
	void staubliGoButton_clicked();
	void initializeBarrettButton_clicked();
	void barrettGoButton_clicked();
	void getBarrettStatusButton_clicked();
	void getLocButton_clicked();
	void barrettCloseButton_clicked();
	void barrettOpenButton_clicked();
	void getBarrettButton_clicked();
};
#endif //STAUBLICONTROLDLG_H_