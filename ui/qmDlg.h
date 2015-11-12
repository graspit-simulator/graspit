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
// $Id: 
//
//######################################################################

/*! \file
  \brief Implements the QMDlg, the quality measure dialog box. 
*/

#include "ui_qmDlg.h"
#include <quality.h>

/*! \class QMDlg
  \brief Creates and controls the quality measure dialog box.

  The dialog box contains a list of the currently defined quality measures
  for this grasp as well as a "New Quality Measure" item.  It also has
  text entry box for changing the name of a QM, a combo box for choosing
  the QM type, and a settings area for the settings
  of the individual types of QM's.  If the user selects a currently defined
  measure from the list, the QM type and settings area are updated using the
  current values from the chosen QM.  The user can change these values and
  click the "add/edit" button to make the changes, or click the "delete"
  button to remove the quality measure.  If the user selects "New Quality
  Measure", then chooses a name and a type for the QM and chooses settings
  for that QM, it can then be added to the list by clicking "add/edit".
*/

class QMDlg : public QDialog, public Ui::QMDlgUI
{
	Q_OBJECT
private:
	qmDlgDataT qmDlgData;

	void init();
public:
	QMDlg(QWidget *parent) : QDialog(parent) {
		setupUi(this);
		init();
	}
public slots:
	void selectQMType(const QString &typeStr);
	void updateSettingsBox();
	void addEditQM();
	void deleteQM();
	void selectQM( int which);
	void gravityBox_clicked();
};
