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
// Author(s): Andrew T. Miller
//
// $Id: settingsDlg.cpp,v 1.2 2009/03/25 22:10:14 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the SettingsDlg, the user preferences dialog box.
*/

#include "settingsDlg.h"
#include "graspit/world.h"
#include "graspit/graspitCore.h"
#include "graspit/ivmgr.h"

#include <QValidator>
#include <QMessageBox>

/*!
  Sets up the static and kinetic friction tables.  It gets the current
  materials list from the world and puts these values in the first row
  and column of each table.  The individual coefficients for each pair of
  materials are then read from the world, and entered into the appropriate
  cells.  Numerical validators are also created for the boxes on the other
  tabs to restrict the time step range and the number of possible friction
  cone vectors.
*/
void SettingsDlg::init()
{
  int i, j;
  QString val;
  World *w = graspitCore->getWorld();

  dlgUI->staticFrictionTable->horizontalHeader()->hide();
  dlgUI->staticFrictionTable->verticalHeader()->hide();
  dlgUI->kineticFrictionTable->horizontalHeader()->hide();
  dlgUI->kineticFrictionTable->verticalHeader()->hide();
  dlgUI->kineticFrictionTable->setContentsMargins(0, 0, 0, 0);


  dlgUI->staticFrictionTable->setRowCount(w->getNumMaterials() + 1);
  dlgUI->staticFrictionTable->setColumnCount(w->getNumMaterials() + 1);
  dlgUI->kineticFrictionTable->setRowCount(w->getNumMaterials() + 1);
  dlgUI->kineticFrictionTable->setColumnCount(w->getNumMaterials() + 1);

  dlgUI->staticFrictionTable->setItem(0, 0, new QTableWidgetItem(""));
  dlgUI->kineticFrictionTable->setItem(0, 0, new QTableWidgetItem(""));

  for (i = 0; i < w->getNumMaterials(); i++) {
    dlgUI->staticFrictionTable->setItem(0, i + 1, new QTableWidgetItem(QString(w->getMaterialName(i))));
    dlgUI->staticFrictionTable->setItem(i + 1, 0, new QTableWidgetItem(QString(w->getMaterialName(i))));
    dlgUI->kineticFrictionTable->setItem(0, i + 1, new QTableWidgetItem(QString(w->getMaterialName(i))));
    dlgUI->kineticFrictionTable->setItem(i + 1, 0, new QTableWidgetItem(QString(w->getMaterialName(i))));
  }

  for (i = 0; i < w->getNumMaterials(); i++) {
    for (j = 0; j < w->getNumMaterials(); j++) {
      dlgUI->staticFrictionTable->setItem(i + 1, j + 1, new QTableWidgetItem(val.setNum(w->getCOF(i, j))));
      dlgUI->kineticFrictionTable->setItem(i + 1, j + 1, new QTableWidgetItem(val.setNum(w->getKCOF(i, j))));
    }
  }

  dlgUI->timeStepLine->setText(QString::number(w->getTimeStep() * 1.0e+3));
  dlgUI->timeStepLine->setValidator(new QDoubleValidator(1.0e-2, 1.0e+4, 10, dlgImpl));

  QObject::connect(dlgUI->staticFrictionTable, SIGNAL(currentCellChanged(int, int, int, int)),
                   this, SLOT(saveCurrentCOF(int, int, int, int)));
  QObject::connect(dlgUI->kineticFrictionTable, SIGNAL(currentCellChanged(int, int, int, int)),
                   this, SLOT(saveCurrentKCOF(int, int, int, int)));

  QObject::connect(dlgUI->staticFrictionTable, SIGNAL(cellChanged(int, int)),
                   this, SLOT(checkCOFEntry(int, int)));
  QObject::connect(dlgUI->kineticFrictionTable, SIGNAL(cellChanged(int, int)),
                   this, SLOT(checkKCOFEntry(int, int)));

  QObject::connect(dlgUI->okButton, SIGNAL(clicked()), this, SLOT(validateDlg()));

}

SettingsDlg::~SettingsDlg()
{
  delete dlgImpl;
  delete dlgUI;
}

/*!
  When the user enters a new value for a coefficient of friction, this checks
  that the value is positive, and if not it resets it.  The change is also
  reflected in across the diagonal in the table (the same pair of materials).
  If the user changes a material name, the name is updated in the other table
  as well.
*/
void SettingsDlg::checkCOFEntry(int row, int col)
{
  double val;
  bool ok;

  if (row > 0 && col > 0) {
    val = dlgUI->staticFrictionTable->item(row, col)->text().toDouble(&ok);
    if (!ok || val < 0) {
      dlgUI->staticFrictionTable->setItem(row, col, new QTableWidgetItem(QString::number(currCOFVal)));
      return;
    }
  }
  else {
    dlgUI->kineticFrictionTable->setItem(row, col, new QTableWidgetItem(dlgUI->staticFrictionTable->item(row, col)->text()));
    dlgUI->kineticFrictionTable->setItem(col, row, new QTableWidgetItem(dlgUI->staticFrictionTable->item(row, col)->text()));
  }
  dlgUI->staticFrictionTable->setItem(col, row, new QTableWidgetItem(dlgUI->staticFrictionTable->item(row, col)->text()));
}


/*!
  Whenever the user selects a new box within the static table, this saves the
  current value there in case it needs to be reset when the user tries
  to enter an invalid number.
*/
void SettingsDlg::saveCurrentCOF(int row, int col, int prevRow, int prevCol)
{
  currCOFVal = dlgUI->staticFrictionTable->item(row, col)->text().toDouble();
}

/*!
  Whenever the user selects a new box within the kinetic table, this saves the
  current value there in case it needs to be reset when the user tries
  to enter an invalid number.
*/
void SettingsDlg::saveCurrentKCOF(int row, int col, int prevRow, int prevCol)
{
  currKCOFVal = dlgUI->kineticFrictionTable->item(row, col)->text().toDouble();
}


/*!
  When the user enters a new value for a coefficient of friction, this checks
  that the value is positive, and if not it resets it.  The change is also
  reflected in across the diagonal in the table (the same pair of materials).
  If the user changes a material name, the name is updated in the other table
  as well.
*/
void SettingsDlg::checkKCOFEntry(int row, int col)
{
  double val;
  bool ok;
  if (row > 0 && col > 0) {
    val = dlgUI->kineticFrictionTable->item(row, col)->text().toDouble(&ok);
    if (!ok || val < 0) {
      dlgUI->kineticFrictionTable->setItem(row, col, new QTableWidgetItem(QString::number(currKCOFVal)));
      return;
    }
  }
  else {
    dlgUI->staticFrictionTable->setItem(row, col, new QTableWidgetItem(dlgUI->kineticFrictionTable->item(row, col)->text()));
    dlgUI->staticFrictionTable->setItem(col, row, new QTableWidgetItem(dlgUI->kineticFrictionTable->item(row, col)->text()));
  }
  dlgUI->kineticFrictionTable->setItem(col, row, new QTableWidgetItem(dlgUI->kineticFrictionTable->item(row, col)->text()));
}


/*!
  This checks the validators to make sure the number of friction cone vectors
  and the dynamic time step are within their ranges.  If they aren't, a
  warning box explaining the problem is created.  Otherwise, the dialog
  is closed and accepted.
*/
void SettingsDlg::validateDlg()
{
  int zero = 0;
  QString tst = dlgUI->timeStepLine->text();
  QString msg;
  QDoubleValidator *tsv = (QDoubleValidator *)dlgUI->timeStepLine->validator();

  if (tsv->validate(tst, zero) != QValidator::Acceptable) {
    msg = QString("Dynamic time step must be between %1 and %2").arg(tsv->bottom()).arg(tsv->top());
  }

  if (!msg.isEmpty()) {
    QMessageBox::warning(NULL, "GraspIt!", msg, QMessageBox::Ok, Qt::NoButton, Qt::NoButton);
  } else {
    dlgImpl->accept();
  }
}
