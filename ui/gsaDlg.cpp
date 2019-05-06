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
// Author(s): Maximilian Haas-Heger
//
// $Id: gsaDlg.cpp,v 1.6 2009/09/19 00:35:12 cmatei Exp $
//
//######################################################################

#include "gsaDlg.h"

#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/graspSolver.h"
#include "mainWindow.h"

#include "graspit/robots/humanHand.h"

#include "graspit/debug.h"

#include <QDialog>
#include <QCheckBox>
#include <QPushButton>
#include <QStandardItemModel>

#define PROF_ENABLED
#include "graspit/profiling.h"

// Structure holding pointers to UI items
struct UIParamT {
  std::list<QLineEdit*> WrenchInput;
  std::list<QCheckBox*> WrenchPlaneInput;

  std::list<QLineEdit*> JointInput;
  std::list<QCheckBox*> PreloadInput;

  QLineEdit *angTolInput;
  QLineEdit *normUncertInput;

  QCheckBox *stepInput;
  QCheckBox *coneInput;
};

GSADlg::GSADlg(MainWindow *mw, Hand *h, QWidget *parent) : QDialog(parent), mMainWindow(mw), mHand(h)
{
  mParams = new UIParamT();
  mGraspSolver = new GraspSolver(mHand->getGrasp());
  setupUi(this);
  init();
}

GSADlg::~GSADlg()
{
  mMainWindow->clearContactsList();
  delete mParams;
  delete mGraspSolver;
}

void
GSADlg::init()
{
  solverTypeComboBox->insertItem(QString("Refinement"));
  solverTypeComboBox->insertItem(QString("Iterative"));

  solveForComboBox->insertItem(QString("Contact forces"));
  solveForComboBox->insertItem(QString("Maximum wrench"));
  solveForComboBox->insertItem(QString("Optimal preload"));
  solveForComboBox->insertItem(QString("Resistible wrenches"));

  mSettingsArea = new QWidget(gsaGroupBox);
  buildSettingsArea();
}

// Slot for selecting desired type of solver. Disable 'solve for'
// options that are not compatible with selected solver from 
// the solveForComboBox
void
GSADlg::solverTypeSelected(const QString &typeStr)
{
  QStandardItemModel *model = dynamic_cast< QStandardItemModel * >(solveForComboBox->model());
  QStandardItem *item;
  // Refinement solver can solve all queries currently implemented
  if (!strcmp(typeStr.latin1(), "Refinement")) {
    for (int i=0; i<model->rowCount(); i++) {
      item = model->item(i);
      item->setEnabled(true);      
    }
  }
  // Iterative solver cannot solve for optimal preloads
  else if (!strcmp(typeStr.latin1(), "Iterative")) {
    QStandardItem *item = model->item(2);
    item->setEnabled(false);
    if (!model->item(solveForComboBox->currentIndex())->isEnabled()) {
      solveForComboBox->setCurrentIndex(0);
    }
  }
  buildSettingsArea();
}

// Slot for selecting what to solve for
void
GSADlg::solveForSelected(const QString &typeStr)
{
  buildSettingsArea();
}

// Build settings area corresponding to selected solver type and
// query to solve for
void
GSADlg::buildSettingsArea()
{
  delete mParams;
  delete mSettingsArea;
  mParams = new UIParamT();
  mSettingsArea = new QWidget(gsaGroupBox);

  // Enable Solve button in case it was disabled by
  // 'Resistible wrenches' computation
  SolveButton->setEnabled(true);

  QHBoxLayout *hl = new QHBoxLayout(mSettingsArea);

  // Build part of settings area for query settings
  QString solveFor = solveForComboBox->currentText();
  if (!strcmp(solveFor.latin1(), "Contact forces")) {
    forcesSettingsArea(hl);
  } else if (!strcmp(solveFor.latin1(), "Maximum wrench")) {
    maxWrenchSettingsArea(hl);
  } else if (!strcmp(solveFor.latin1(), "Optimal preload")) {
    optPreloadSettingsArea(hl);
  } else if (!strcmp(solveFor.latin1(), "Resistible wrenches")) {
    // Disable Solve button until two axes have been selected
    SolveButton->setEnabled(false);
    resMapSettingsArea(hl);
  }

  // Build solver options part of settings area
  QString solverType = solverTypeComboBox->currentText();
  if (!strcmp(solverType.latin1(), "Refinement")) {
    refinementSettingsArea(hl);
  } else if (!strcmp(solverType.latin1(), "Iterative")) {
    iterativeSettingsArea(hl);
  }

  mSettingsArea->show();
}

void
GSADlg::forcesSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->addWidget(new QLabel(QString("Applied wrench:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  QString lab[] = {"Fx:", "Fy:", "Fz:", "Mx:", "My:", "Mz:"};
  std::list<QString> labels(lab, lab+6);
  std::list<QString>::iterator l_it = labels.begin();
  std::list<QLineEdit*>::iterator e_it = mParams->WrenchInput.begin();
  for ( ; l_it!=labels.end(); l_it++, e_it++) {
    wl->addWidget(new QLabel(*l_it));
    QLineEdit *line = new QLineEdit();
    line->setText(QString::number(0.0));
    mParams->WrenchInput.push_back(line);
    wl->addWidget(line);
  }

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  int preloadSize;
  if (mHand->inherits("HumanHand")) {
    jointLayout->addWidget(new QLabel(QString("Tendon Preloads:")));
    preloadSize = ((HumanHand*)mHand)->getNumTendons();
  } else {
    jointLayout->addWidget(new QLabel(QString("Preload torques:")));
    mHand->getGrasp()->collectContacts();
    std::list<Joint*> joints = mHand->getGrasp()->getJointsOnContactChains(); 
    preloadSize = joints.size();
  }

  QLayout *jl = new QGridLayout(jointLayout,4,2,0);
  mParams->JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  preload = getDefaultPreload();

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit *line = new QLineEdit();
    jl->addWidget(line);
    mParams->JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
  }
}

void
GSADlg::maxWrenchSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->addWidget(new QLabel(QString("Wrench direction:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  QString lab[] = {"Fx:", "Fy:", "Fz:", "Mx:", "My:", "Mz:"};
  std::list<QString> labels(lab, lab+6);
  std::list<QString>::iterator l_it = labels.begin();
  std::list<QLineEdit*>::iterator e_it = mParams->WrenchInput.begin();
  for ( ; l_it!=labels.end(); l_it++, e_it++) {
    wl->addWidget(new QLabel(*l_it));
    QLineEdit *line = new QLineEdit();
    line->setText(QString::number(0.0));
    mParams->WrenchInput.push_back(line);
    wl->addWidget(line);
  }

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  int preloadSize;
  if (mHand->inherits("HumanHand")) {
    jointLayout->addWidget(new QLabel(QString("Tendon Preloads:")));
    preloadSize = ((HumanHand*)mHand)->getNumTendons();
  } else {
    jointLayout->addWidget(new QLabel(QString("Preload torques:")));
    mHand->getGrasp()->collectContacts();
    std::list<Joint*> joints = mHand->getGrasp()->getJointsOnContactChains(); 
    preloadSize = joints.size();
  }

  QLayout *jl = new QGridLayout(jointLayout,4,2,0);
  mParams->JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  preload = getDefaultPreload();

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit *line = new QLineEdit();
    jl->addWidget(line);
    mParams->JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
  }
}

void
GSADlg::optPreloadSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->addWidget(new QLabel(QString("Applied wrench:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  QString lab[] = {"Fx:", "Fy:", "Fz:", "Mx:", "My:", "Mz:"};
  std::list<QString> labels(lab, lab+6);
  std::list<QString>::iterator l_it = labels.begin();
  std::list<QLineEdit*>::iterator e_it = mParams->WrenchInput.begin();
  for ( ; l_it!=labels.end(); l_it++, e_it++) {
    wl->addWidget(new QLabel(*l_it));
    QLineEdit *line = new QLineEdit();
    line->setText(QString::number(0.0));
    mParams->WrenchInput.push_back(line);
    wl->addWidget(line);
  }

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  int preloadSize;
  if (mHand->inherits("HumanHand")) {
    jointLayout->addWidget(new QLabel(QString("Tendon Preloads:")));
    preloadSize = ((HumanHand*)mHand)->getNumTendons();
  } else {
    jointLayout->addWidget(new QLabel(QString("Preload torques:")));
    mHand->getGrasp()->collectContacts();
    std::list<Joint*> joints = mHand->getGrasp()->getJointsOnContactChains(); 
    preloadSize = joints.size();
  }

  QLayout *jl = new QGridLayout(jointLayout,preloadSize+1,3,0);
  mParams->JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  preload = getDefaultPreload();

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit *line = new QLineEdit();
    jl->addWidget(line);
    mParams->JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
    QCheckBox *box = new QCheckBox();
    jl->addWidget(box);
    mParams->PreloadInput.push_back(box);
    QObject::connect(box, SIGNAL (toggled(bool)), line, SLOT (setDisabled(bool)));
  }
}

void
GSADlg::resMapSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->setAlignment(Qt::AlignTop);
  forceLayout->addWidget(new QLabel(QString("Select plane dimensions:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  QString lab[] = {"Fx:", "Fy:", "Fz:", "Mx:", "My:", "Mz:"};
  std::list<QString> labels(lab, lab+6);
  std::list<QString>::iterator l_it = labels.begin();
  std::list<QLineEdit*>::iterator e_it = mParams->WrenchInput.begin();
  for ( ; l_it!=labels.end(); l_it++, e_it++) {
    wl->addWidget(new QLabel(*l_it));
    QCheckBox *box = new QCheckBox();
    QObject::connect(box, SIGNAL (toggled(bool)), this, SLOT (countAxes()));
    mParams->WrenchPlaneInput.push_back(box);
    wl->addWidget(box);
  }

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  int preloadSize;
  if (mHand->inherits("HumanHand")) {
    jointLayout->addWidget(new QLabel(QString("Tendon Preloads:")));
    preloadSize = ((HumanHand*)mHand)->getNumTendons();
  } else {
    jointLayout->addWidget(new QLabel(QString("Preload torques:")));
    mHand->getGrasp()->collectContacts();
    std::list<Joint*> joints = mHand->getGrasp()->getJointsOnContactChains(); 
    preloadSize = joints.size();
  }

  QLayout *jl = new QGridLayout(jointLayout,4,2,0);
  mParams->JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  preload = getDefaultPreload();

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit *line = new QLineEdit();
    jl->addWidget(line);
    mParams->JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
  }
}

void
GSADlg::refinementSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *settingsLayout = new QVBoxLayout(hl);
  settingsLayout->setAlignment(Qt::AlignTop);
  settingsLayout->addWidget(new QLabel(QString("Refinement solver settings:")));
  
  settingsLayout->addWidget(new QLabel("Angular tolerance:"));
  mParams->angTolInput = new QLineEdit();
  mParams->angTolInput->setText(QString::number(1.0));
  settingsLayout->addWidget(mParams->angTolInput);

  settingsLayout->addWidget(new QLabel("Contact normal uncertainty:"));
  mParams->normUncertInput = new QLineEdit();
  mParams->normUncertInput->setText(QString::number(2.5));
  settingsLayout->addWidget(mParams->normUncertInput);
}

void
GSADlg::iterativeSettingsArea(QHBoxLayout *hl)
{
  QVBoxLayout *settingsLayout = new QVBoxLayout(hl);
  settingsLayout->setAlignment(Qt::AlignTop);
  settingsLayout->addWidget(new QLabel(QString("Iterative solver settings:")));

  QGridLayout *sl = new QGridLayout(settingsLayout,4,2,0);

  sl->addWidget(new QLabel(QString("Single step:")));
  mParams->stepInput = new QCheckBox();
  sl->addWidget(mParams->stepInput);

  sl->addWidget(new QLabel(QString("Cone Movement:")));
  mParams->coneInput = new QCheckBox();
  sl->addWidget(mParams->coneInput);
}

void
GSADlg::solveButtonClicked()
{
  DBGA("Success!");
}

void
GSADlg::countAxes()
{
  int counter = 0;
  std::list<QCheckBox*>::iterator it = mParams->WrenchPlaneInput.begin();
  for ( ; it!= mParams->WrenchPlaneInput.end(); it++)
    if ((*it)->isChecked()) counter++;
  if (counter < 2) {
    SolveButton->setEnabled(false);
    it = mParams->WrenchPlaneInput.begin();
    for ( ; it!=mParams->WrenchPlaneInput.end(); it++)
      (*it)->setEnabled(true);
  }
  if (counter == 2) {
    SolveButton->setEnabled(true);
    it = mParams->WrenchPlaneInput.begin();
    for ( ; it!=mParams->WrenchPlaneInput.end(); it++) {
      if (!(*it)->isChecked()) (*it)->setEnabled(false);
    }
  }
}

std::vector<double> 
GSADlg::getDefaultPreload()
{
  std::list<Joint*> joints = mHand->getGrasp()->getJointsOnContactChains(); 
  std::vector<double> joint_preload(joints.size(), 0.0);

  if (mHand->isA("Barrett")) {
    //specific to Barrett hand
    int jNum=0;
    for (std::list<Joint*>::iterator it = joints.begin(); it!=joints.end(); it++, jNum++) {
      if (mHand->getDOF((*it)->getDOFNum())->getDefaultVelocity() > 0) {
  if ( (*it)->getNum() == 1 ) {
    //proximal on f1
    joint_preload[jNum] = 1;
  } else if ( (*it)->getNum() == 4 ) {
    //proximal on f2
    joint_preload[jNum] = 1;       
  } else if ((*it)->getChainNum() == 2) {
    //thumb
    KinematicChain *chain = mHand->getChain((*it)->getChainNum());
    Link *link = chain->getLink(chain->getNumLinks() - 1);
    if (link->getNumContacts() != 0) {
      //contacts on distal link
      if ( (*it)->getNum() == 6 ) joint_preload[jNum] = 1.0;
      else if ( (*it)->getNum() == 7 ) joint_preload[jNum] = 0.0;
      else assert(0);
    } else {
      //no contacts on distal link
      if ( (*it)->getNum() == 6 ) joint_preload[jNum] = 1.0;
      else if ( (*it)->getNum() == 7 ) joint_preload[jNum] = 0;
      else assert(0);
    }
  } else {
    //all others
    joint_preload[jNum] = 0;
  }
      } 
    }
  } else {
    //all other hands
    int jNum=0;
    for (std::list<Joint*>::iterator it = joints.begin(); it!=joints.end(); it++, jNum++) {
      if (mHand->getDOF((*it)->getDOFNum())->getDefaultVelocity() > 0) {
    joint_preload[jNum] = 1.0;
      }
    }
  }
  
  return joint_preload;
}
