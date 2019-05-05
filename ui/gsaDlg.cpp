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
#include "mainWindow.h"

#include "graspit/robots/humanHand.h"

#include "graspit/debug.h"

#define PROF_ENABLED
#include "graspit/profiling.h"

GSADlg::GSADlg(MainWindow *mw, Hand *h, QWidget *parent) : QDialog(parent), mMainWindow(mw), mHand(h)
{
  setupUi(this);
  setupDlgArea();
}

void
GSADlg::setupDlgArea()
{
  QHBoxLayout *hl = new QHBoxLayout(gsaGroupBox);

  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->addWidget(new QLabel(QString("Applied wrench:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  wl->addWidget(new QLabel(QString("Mult:")));
  mParams.wrenchInput = new QLineEdit();
  mParams.wrenchInput->setText(QString::number(1.0));
  wl->addWidget(mParams.wrenchInput);

  wl->addWidget(new QLabel(QString("Fx:")));
  mParams.FxInput = new QLineEdit();
  mParams.FxInput->setText(QString::number(0.0));
  wl->addWidget(mParams.FxInput);

  wl->addWidget(new QLabel(QString("Fy:")));
  mParams.FyInput = new QLineEdit();
  mParams.FyInput->setText(QString::number(0.0));
  wl->addWidget(mParams.FyInput);

  wl->addWidget(new QLabel(QString("Fz:")));
  mParams.FzInput = new QLineEdit();
  mParams.FzInput->setText(QString::number(0.0));
  wl->addWidget(mParams.FzInput);

  wl->addWidget(new QLabel(QString("Tx:")));
  mParams.MxInput = new QLineEdit();
  mParams.MxInput->setText(QString::number(0.0));
  wl->addWidget(mParams.MxInput);

  wl->addWidget(new QLabel(QString("Ty:")));
  mParams.MyInput = new QLineEdit();
  mParams.MyInput->setText(QString::number(0.0));
  wl->addWidget(mParams.MyInput);

  wl->addWidget(new QLabel(QString("Tz:")));
  mParams.MzInput = new QLineEdit();
  mParams.MzInput->setText(QString::number(0.0));
  wl->addWidget(mParams.MzInput);

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

  jl->addWidget(new QLabel(QString("Mult:")));
  mParams.preloadInput = new QLineEdit();
  mParams.preloadInput->setText(QString::number(1.0));
  jl->addWidget(mParams.preloadInput);

  mParams.JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  preload = getDefaultPreload();

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit *line = new QLineEdit();
    jl->addWidget(line);
    mParams.JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
  }

  QVBoxLayout *optionsLayout = new QVBoxLayout(hl);
  optionsLayout->setAlignment(Qt::AlignTop);
  optionsLayout->addWidget(new QLabel(QString("Solver Options:")));

  QLayout *ol = new QGridLayout(optionsLayout,4,2,1);

  bool single_step = false;
  bool iterative = false;
  bool cone_movement = false;
  bool rigid = false;
  bool map = false;

  ol->addWidget(new QLabel(QString("Single step:")));
  mParams.stepInput = new QCheckBox();
  mParams.stepInput->setChecked(single_step);
  ol->addWidget(mParams.stepInput);
  
  ol->addWidget(new QLabel(QString("Iterative:")));
  mParams.iterInput = new QCheckBox();
  mParams.iterInput->setChecked(iterative);
  ol->addWidget(mParams.iterInput);  

  ol->addWidget(new QLabel(QString("Cone Movement:")));
  mParams.coneInput = new QCheckBox();
  mParams.coneInput->setChecked(cone_movement);
  mParams.coneInput->setEnabled(iterative);
  ol->addWidget(mParams.coneInput);

  ol->addWidget(new QLabel(QString("Rigid:")));
  mParams.rigidInput = new QCheckBox();
  mParams.rigidInput->setChecked(rigid);
  mParams.rigidInput->setDisabled(mHand->inherits("HumanHand"));
  mParams.rigidInput->setDisabled(single_step);
  ol->addWidget(mParams.rigidInput);

  ol->addWidget(new QLabel(QString("2D Map:")));
  mParams.mapInput = new QCheckBox();
  mParams.mapInput->setChecked(map);
  ol->addWidget(mParams.mapInput);

  QObject::connect(mParams.iterInput, SIGNAL (toggled(bool)),
    mParams.coneInput, SLOT (setEnabled(bool)));
  QObject::connect(mParams.stepInput, SIGNAL (toggled(bool)),
    mParams.rigidInput, SLOT (setDisabled(bool)));
  QObject::connect(mParams.rigidInput, SIGNAL (toggled(bool)),
    mParams.stepInput, SLOT (setDisabled(bool)));

  mParams.solveButton = new QPushButton("&Solve");
  optionsLayout->addWidget(mParams.solveButton);

  QObject::connect(mParams.solveButton, SIGNAL(clicked()), this, SLOT(solveButtonClicked()));
}

GSADlg::~GSADlg()
{
  mMainWindow->clearContactsList();
}

void
GSADlg::solveButtonClicked()
{
  DBGA("Success!");
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
