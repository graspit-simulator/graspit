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
// Author(s): Andrew T. Miller 
//
// $Id: settingsDlg.h,v 1.3 2009/04/21 14:53:09 cmatei Exp $
//
//######################################################################

#ifndef _settingsdlg_h_
#define _settingsdlg_h_

/*! \file
  \brief Implements the SettingsDlg, the user preferences dialog box. 
*/

#include "ui_settingsDlg.h"
#include <QObject>
#include <QDialog>

/*! \class SettingsDlg
  \brief Creates and controls the user preferences dialog box.

  This dialog box consists of three tabs: coefficients of friction,
  friction cones, and dynamics.  When the dialog box is opened, the values
  for the various parameters are read from the current world.  The COF tab
  allows the user to change material names and the values of the static and
  kinetic coefficients of friction.  The friction cones tab allows the user
  to change the number of vectors that are used to approximate friction
  cones, and the dynamics tab allows the user to change the dynamics time
  step length.
*/

class SettingsDlg : public QObject
{
	Q_OBJECT
private:
	double currCOFVal, currKCOFVal;
	QDialog *dlgImpl;
	void init();

private slots:
	void checkCOFEntry( int row, int col );
	void saveCurrentCOF( int row, int col );
	void saveCurrentKCOF( int row, int col );
	void checkKCOFEntry( int row, int col );
	void validateDlg();

public:
	Ui::SettingsDlgUI *dlgUI;
	SettingsDlg(QWidget * parent = 0, Qt::WFlags f = 0){
		dlgImpl = new QDialog(parent, f);
		dlgUI = new Ui::SettingsDlgUI;
		dlgUI->setupUi(dlgImpl);
		init();
	}
	~SettingsDlg();
	int exec(){return dlgImpl->exec();}
	void setAttribute(Qt::WidgetAttribute attribute, bool on = true) {
		dlgImpl->setAttribute(attribute,on);
	};

};

#endif
