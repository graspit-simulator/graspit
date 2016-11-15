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
// $Id: 
//
//######################################################################

#include <QHBoxLayout>
#include "qmDlg.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "grasp.h"
#include "list"
#include "world.h"
#include "robot.h"

//#define GRASPITDBG
#include "debug.h"

/*!
  This populates the quality measure list with the currently defined quality
  measures for this grasp.  Then it populates the QM comboBox with the
  all the possible quality measure types.  Next, it creates an empty 
  settings area widget to hold the settings for the individual types of
  quality measures, and adds it to the layout.
*/
void QMDlg::init()
{
  std::list<QualityMeasure *>::iterator qp;
  Grasp *g = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
  int i;
  
  qmListBox->insertItem("New quality measure");
  for (qp=g->qmList.begin(),i=1;qp!=g->qmList.end();qp++,i++)
    qmListBox->insertItem((*qp)->getName());
  
  for (i=0;QualityMeasure::TYPE_LIST[i];i++)
    qmTypeComboBox->insertItem(QString(QualityMeasure::TYPE_LIST[i]));

  qmSettingsBox->setColumnLayout(0, Qt::Vertical );
  QHBoxLayout *settingsBoxLayout = new QHBoxLayout(qmSettingsBox->layout());
  settingsBoxLayout->setAlignment( Qt::AlignTop );

  qmDlgData.settingsArea = new QWidget(qmSettingsBox);
  settingsBoxLayout->addWidget(qmDlgData.settingsArea);

  qmDlgData.grasp = g;
  qmDlgData.qmTypeComboBox = qmTypeComboBox;
  qmDlgData.qmName = qmName;
  gravityBox->setChecked( g->isGravitySet() );

  qmListBox->setCurrentItem(0);
}


/*!
  Reads the selected QM type and calls updateSettingsBox .
*/
void QMDlg::selectQMType(const QString &typeStr)
{
  qmDlgData.qmType = typeStr.latin1();
  updateSettingsBox();
}

/*!
  Deletes the current settings area widget, creates a new one, and
  calls the static quality meausre method to build the correct set of
  settings widgets for the currently selected QM type.
*/
void QMDlg::updateSettingsBox()
{
  delete qmDlgData.settingsArea;

  qmDlgData.settingsArea = new QWidget(qmSettingsBox);
  qmSettingsBox->layout()->setAlignment(Qt::AlignTop);
  qmSettingsBox->layout()->add(qmDlgData.settingsArea);
  QualityMeasure::buildParamArea(&qmDlgData);

  qmDlgData.settingsArea->show();
}

/*!
  Creates a new quality measure of the selected type with the given name from
  the name text area.  If "New Quality Measure" is selected in the list, it
  adds this QM to the grasp and to the quality measure list.  Otherwise it
  replaces the currently selected QM with the new one.
*/
void QMDlg::addEditQM()
{
  Grasp *g = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
  QualityMeasure *newQM;
  int selectedQM;

  newQM = QualityMeasure::createInstance(&qmDlgData);
  
  selectedQM = qmListBox->currentItem();
  if (selectedQM == 0) { // create a new quality measure    
    g->addQM(newQM);

    qmListBox->insertItem(qmName->text());
  }
  else { // replace an old quality measure with a new one
    g->replaceQM(selectedQM-1,newQM);
    qmListBox->changeItem(qmName->text(),selectedQM);
  }

  qmListBox->setCurrentItem(0);
  qmListBox->update();
  qmListBox->show();

}

/*!
  Removes the selected QM from the grasp and the quality measure list. Then
  it selects the next item in the list.
*/
void QMDlg::deleteQM()
{
  int selectedQM;
  int numItems;
  
  selectedQM = qmListBox->currentItem();
  graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp()->
    removeQM(selectedQM-1);
  qmListBox->removeItem(selectedQM);

  // select the next item in the list
  numItems = qmListBox->count();
  qmListBox->setCurrentItem(selectedQM < numItems ? selectedQM : 0);
  qmListBox->update();
}

/*!
  If "New Quality Measure" is selected, the first item is the QM type box
  is selected, and the delete button is disabled.  Otherwise, it updates
  the name in the text area, and sets the QM type in the comboBox to the
  type of thecurrently selected QM, and finally calls the updateSettingsBox
  to update the settings area.
*/
void QMDlg::selectQM( int which)
{
  if (which==0) {  // "New quality measure" selected
    qmDlgData.currQM = NULL;
    qmDlgData.qmType = QualityMeasure::TYPE_LIST[0];
    qmTypeComboBox->setCurrentItem(0);
    DeleteButton->setEnabled(false);
    qmName->setText(QString("New Quality Measure"));
  }
  else {
    DeleteButton->setEnabled(true);
    Grasp *g = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
    qmDlgData.currQM = g->getQM(which-1);

	for (int i=0;QualityMeasure::TYPE_LIST[i];i++) {
      if (!strcmp(QualityMeasure::TYPE_LIST[i],qmDlgData.currQM->getType())) {
		qmTypeComboBox->setCurrentItem(i);
		qmDlgData.qmType = QualityMeasure::TYPE_LIST[i];
		break;
      }
	}
    qmName->setText(qmListBox->text(which));
  }
  updateSettingsBox();
}

void QMDlg::gravityBox_clicked()
{
    Grasp *g = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
	g->setGravity( gravityBox->isChecked() );
	if ( gravityBox->isChecked() ) {
		fprintf(stderr,"Gravity on\n");
	} else {
		fprintf(stderr,"Gravity off\n");
	}
//	g->updateWrenchSpaces();
	g->update();
}
