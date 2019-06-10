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
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QStandardItemModel>

#define PROF_ENABLED
#include "graspit/profiling.h"

// Structure holding pointers to UI items
struct UIParamT {
  std::list<QLineEdit*> WrenchInput;
  std::list<QCheckBox*> PlaneInput;

  std::list<QLineEdit*> JointInput;
  std::list<QCheckBox*> PreloadInput;

  QLineEdit *coneTolInput;
  QLineEdit *normUncInput;

  QCheckBox *stepInput;
  QCheckBox *coneInput;

  QLineEdit *fileInput;

  LEDIndicator *stableIndicator;
  QLabel *maxWrenchOutput;
};

//! Possible solver types and queries expressed as strings
const char *GSADlg::REFINEMENT = "Refinement";
const char *GSADlg::ITERATIVE = "Iterative";

const char *GSADlg::FORCES = "Contact forces";
const char *GSADlg::MAX_WRENCH = "Maximum wrench";
const char *GSADlg::OPT_PRELOAD = "Optimal preload";
const char *GSADlg::RES_MAP = "Resistible wrenches";

GSADlg::GSADlg(MainWindow *mw, Hand *h, QWidget *parent) : QDialog(parent), mMainWindow(mw), mHand(h)
{
  mParams = new UIParamT();
  setupUi(this);
  init();
}

GSADlg::~GSADlg()
{
  mMainWindow->clearContactsList();
  delete mParams;
}

void
GSADlg::init()
{
  solverTypeComboBox->insertItem(QString(GSADlg::REFINEMENT));
  solverTypeComboBox->insertItem(QString(GSADlg::ITERATIVE));

  solveForComboBox->insertItem(QString(GSADlg::FORCES));
  solveForComboBox->insertItem(QString(GSADlg::MAX_WRENCH));
  solveForComboBox->insertItem(QString(GSADlg::OPT_PRELOAD));
  solveForComboBox->insertItem(QString(GSADlg::RES_MAP));

  mSettingsArea = new QWidget(gsaGroupBox);
  buildSettingsArea();
}

// Slot for selecting desired type of solver. Disable 'solve for'
// options that are not compatible with selected solver from 
// the solveForComboBox
void
GSADlg::solverTypeSelected(const QString &typeStr)
{
  QStandardItemModel *model;
  model = dynamic_cast< QStandardItemModel * >(solveForComboBox->model());

  // Refinement solver can solve all queries currently implemented
  if (!strcmp(typeStr.latin1(), GSADlg::REFINEMENT)) {
    for (int i=0; i<model->rowCount(); i++) {
      QStandardItem *item = model->item(i);
      item->setEnabled(true);      
    }
  }

  // Iterative solver cannot solve for optimal preloads
  else if (!strcmp(typeStr.latin1(), GSADlg::ITERATIVE)) {
    QStandardItem *item = model->findItems(GSADlg::OPT_PRELOAD).front();
    item->setEnabled(false);
  }

  // If currently selected query is disabled default to force problem
  if (!model->item(solveForComboBox->currentIndex())->isEnabled()) {
    solveForComboBox->setCurrentIndex(0);
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
  // Delete old and create new settings area
  delete mParams;
  delete mSettingsArea;
  mParams = new UIParamT();
  mSettingsArea = new QWidget(gsaGroupBox);

  // Enable Solve button in case it was disabled by
  // 'Resistible wrenches' computation
  SolveButton->setEnabled(true);

  QVBoxLayout *vl = new QVBoxLayout(mSettingsArea);
  QHBoxLayout *hl = new QHBoxLayout(vl);

  // Build part of settings area for query settings
  QString solveFor = solveForComboBox->currentText();
  if (!strcmp(solveFor.latin1(), GSADlg::FORCES)) {
    forcesSettingsArea(hl, vl);
  } else if (!strcmp(solveFor.latin1(), GSADlg::MAX_WRENCH)) {
    maxWrenchSettingsArea(hl, vl);
  } else if (!strcmp(solveFor.latin1(), GSADlg::OPT_PRELOAD)) {
    optPreloadSettingsArea(hl, vl);
  } else if (!strcmp(solveFor.latin1(), GSADlg::RES_MAP)) {
    // Disable Solve button until two axes have been selected
    SolveButton->setEnabled(false);
    resMapSettingsArea(hl, vl);
  }

  // Build solver options part of settings area
  QString solverType = solverTypeComboBox->currentText();
  if (!strcmp(solverType.latin1(), GSADlg::REFINEMENT)) {
    refinementSettingsArea(hl);
  } else if (!strcmp(solverType.latin1(), GSADlg::ITERATIVE)) {
    iterativeSettingsArea(hl);
  }

  mSettingsArea->show();
}

void
GSADlg::forcesSettingsArea(QHBoxLayout *hl, QVBoxLayout *vl)
{
  QVBoxLayout *wrenchLayout = new QVBoxLayout(hl);
  wrenchLayout->setAlignment(Qt::AlignTop);
  wrenchLayout->addWidget(new QLabel(QString("Applied wrench:")));

  QLayout *wl = new QGridLayout(wrenchLayout,4,2,0);

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
  jointLayout->setAlignment(Qt::AlignTop);
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

  QHBoxLayout *statusLayout = new QHBoxLayout(vl);
  statusLayout->addWidget(new QLabel(QString("Stable:")));
  statusLayout->addStretch(1);
  mParams->stableIndicator = new LEDIndicator();
  statusLayout->addWidget(mParams->stableIndicator);
  statusLayout->addStretch(25);
}

void
GSADlg::maxWrenchSettingsArea(QHBoxLayout *hl, QVBoxLayout *vl)
{
  QVBoxLayout *wrenchLayout = new QVBoxLayout(hl);
  wrenchLayout->setAlignment(Qt::AlignTop);
  wrenchLayout->addWidget(new QLabel(QString("Wrench direction:")));

  QLayout *wl = new QGridLayout(wrenchLayout,4,2,0);

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
  jointLayout->setAlignment(Qt::AlignTop);
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

  QHBoxLayout *outputLayout = new QHBoxLayout(vl);
  outputLayout->addWidget(new QLabel(QString("Maximum magnitude:")));
  mParams->maxWrenchOutput = new QLabel();
  mParams->maxWrenchOutput->setFrameShape(QFrame::Panel);
  mParams->maxWrenchOutput->setFrameShadow(QFrame::Sunken);
  mParams->maxWrenchOutput->setFixedWidth(100);
  outputLayout->addWidget(mParams->maxWrenchOutput);
  outputLayout->addStretch();
}

void
GSADlg::optPreloadSettingsArea(QHBoxLayout *hl, QVBoxLayout *vl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->setAlignment(Qt::AlignTop);
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
  jointLayout->setAlignment(Qt::AlignTop);
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
    QObject::connect(box, SIGNAL (toggled(bool)), line, SLOT (clear(void)));
  }
}

void
GSADlg::resMapSettingsArea(QHBoxLayout *hl, QVBoxLayout *vl)
{
  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->setAlignment(Qt::AlignTop);
  forceLayout->addWidget(new QLabel(QString("Plane axes:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,0);

  QString lab[] = {"Fx:", "Fy:", "Fz:", "Mx:", "My:", "Mz:"};
  std::list<QString> labels(lab, lab+6);
  std::list<QString>::iterator l_it = labels.begin();
  std::list<QLineEdit*>::iterator e_it = mParams->WrenchInput.begin();
  for ( ; l_it!=labels.end(); l_it++, e_it++) {
    wl->addWidget(new QLabel(*l_it));
    QCheckBox *box = new QCheckBox();
    QObject::connect(box, SIGNAL (toggled(bool)), this, SLOT (countAxes()));
    mParams->PlaneInput.push_back(box);
    wl->addWidget(box);
  }

  forceLayout->addWidget(new QLabel(QString("Output file:")));
  std::stringstream default_filename;
  default_filename << std::getenv("GRASPIT") << "/build/resistible_wrenches.csv";
  mParams->fileInput = new QLineEdit();
  mParams->fileInput->setText(default_filename.str().c_str());
  forceLayout->addWidget(mParams->fileInput);

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  jointLayout->setAlignment(Qt::AlignTop);
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
  mParams->coneTolInput = new QLineEdit();
  mParams->coneTolInput->setText(QString::number(1.0));
  settingsLayout->addWidget(mParams->coneTolInput);

  settingsLayout->addWidget(new QLabel("Contact normal uncertainty:"));
  mParams->normUncInput = new QLineEdit();
  mParams->normUncInput->setText(QString::number(2.5));
  settingsLayout->addWidget(mParams->normUncInput);
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
  mHand->getGrasp()->update();
  GraspSolver graspSolver(mHand->getGrasp());

  QString solverType = solverTypeComboBox->currentText();
  QString solveFor = solveForComboBox->currentText();

  // Clear output areas until computation is completed
  if (!strcmp(solveFor.latin1(), GSADlg::FORCES)) {
    mParams->stableIndicator->setColor(Qt::gray);
    mParams->stableIndicator->repaint();
  } else if (!strcmp(solveFor.latin1(), GSADlg::MAX_WRENCH)) {
    mParams->maxWrenchOutput->setText(QString("Running..."));
    mParams->maxWrenchOutput->repaint();
  }

  // All but the resistible wrenches query specify a wrench.
  // The resistible wrenches query requires two directions
  Matrix wrench(6,1);
  Matrix direction1(Matrix::ZEROES<Matrix>(6,1));
  Matrix direction2(Matrix::ZEROES<Matrix>(6,1));
  if (!strcmp(solveFor.latin1(), GSADlg::RES_MAP)) {
    std::list<QCheckBox*>::iterator it=mParams->PlaneInput.begin();
    int i,j;
    for (i=j=0; it!=mParams->PlaneInput.end(); i++, it++) {
      if ((*it)->isChecked() && j==0) {direction1.elem(i,0) = 1.0; j++;}
      else if ((*it)->isChecked() && j==1) {direction2.elem(i,0) = 1.0; j++;}
    }
  } else {
    std::list<QLineEdit*>::iterator it=mParams->WrenchInput.begin();
    for (int i=0; it!=mParams->WrenchInput.end(); i++, it++) {
      wrench.elem(i,0) = (*it)->text().toDouble();
    }
  }

  // All possible queries to solve specify preloads
  Matrix preload(mParams->JointInput.size(), 1);
  std::list<QLineEdit*>::iterator it = mParams->JointInput.begin();
  for (int i=0 ; it!= mParams->JointInput.end(); i++, it++)
    preload.elem(i,0) = (*it)->text().toDouble();

  // In order to signal to the solver which preloads to optimize
  // we set the appropriate values in the preload vector to -1
  if (!strcmp(solveFor.latin1(), GSADlg::OPT_PRELOAD)) {
    std::list<QCheckBox*>::iterator it=mParams->PreloadInput.begin();
    for (int i=0; it!=mParams->PreloadInput.end(); i++, it++) {
      if ((*it)->isChecked()) preload.elem(i,0) = -1.0;
    }
  }

  // Resistible wrenches query specifies a file to save output
  QString file;
  std::ofstream stream;
  if (!strcmp(solveFor.latin1(), GSADlg::RES_MAP)) {
    file = mParams->fileInput->text();
    stream.open(file.latin1());
  }

  if (!strcmp(solverType.latin1(), GSADlg::REFINEMENT)) {
    double coneTol = mParams->coneTolInput->text().toDouble();
    double normUnc = mParams->normUncInput->text().toDouble();
    coneTol *= M_PI/180.0;
    normUnc *= M_PI/180.0;

    if (!strcmp(solveFor.latin1(), GSADlg::FORCES)) {
      int result = graspSolver.checkWrenchRefinement(preload, wrench, coneTol, normUnc);
      result ? mParams->stableIndicator->setColor(Qt::red) : mParams->stableIndicator->setColor(Qt::green);
    }
    else if (!strcmp(solveFor.latin1(), GSADlg::MAX_WRENCH)) {
      double max = graspSolver.findMaximumWrenchRefinement(preload, wrench, coneTol, normUnc);
      mParams->maxWrenchOutput->setText(QString::number(max));
    }
    else if (!strcmp(solveFor.latin1(), GSADlg::OPT_PRELOAD)) {
      Matrix opt = graspSolver.optimalPreloads(preload, wrench, coneTol, normUnc);
      std::list<QLineEdit*>::iterator it=mParams->JointInput.begin();
      if (opt.rows()) {
        for (int i=0; i<preload.rows(); i++, it++) {
          (*it)->setText(QString::number(opt.elem(i,0)));
        }
      }
    }
    else if (!strcmp(solveFor.latin1(), GSADlg::RES_MAP)) {
      graspSolver.create2DMapRefinement(preload, direction1, direction2, stream, coneTol, normUnc);
      stream.close();
    }

  } else if (!strcmp(solverType.latin1(), GSADlg::ITERATIVE)) {
    bool step = mParams->stepInput->isChecked();
    bool cone = mParams->coneInput->isChecked();

    if (!strcmp(solveFor.latin1(), GSADlg::FORCES)) {
      int result = graspSolver.checkWrenchIterative(preload, wrench, step, cone);
      result ? mParams->stableIndicator->setColor(Qt::red) : mParams->stableIndicator->setColor(Qt::green);
    }
    else if (!strcmp(solveFor.latin1(), GSADlg::MAX_WRENCH)) {
      double max = graspSolver.findMaximumWrenchIterative(preload, wrench, step, cone);
      mParams->maxWrenchOutput->setText(QString::number(max));
    }
    else if (!strcmp(solveFor.latin1(), GSADlg::RES_MAP)) {
      graspSolver.create2DMapIterative(preload, direction1, direction2, stream, step, cone);
      stream.close();
    }
  }
}

/*void
GSADlg::special(Matrix &preload, double coneTol, double normUnc)
{
  mHand->getGrasp()->update();
  GraspSolver graspSolver(mHand->getGrasp());
  return;

  std::ofstream stream;
  stream.open("/home/max/graspit/build/output.csv");
  stream << "Preload, Fy, Mx\n";
  double max;
  double torque = 0.0;
  Matrix Fy(Matrix::ZEROES<Matrix>(6,1));
  Fy.elem(1,0) = -1.0;
  Matrix Mx(Matrix::ZEROES<Matrix>(6,1));
  Mx.elem(3,0) = 1.0;
  while (torque <= 1.0) {
    stream << torque << ", ";
    preload.elem(6,0) = torque;
    max = graspSolver.findMaximumWrenchRefinement(preload, Fy, coneTol, normUnc);
    stream << max << ", ";
    max = graspSolver.findMaximumWrenchRefinement(preload, Mx, coneTol, normUnc);
    stream << max << std::endl;
    torque += 0.01;
  }
  stream.close();
}*/

void
GSADlg::countAxes()
{
  int counter = 0;
  std::list<QCheckBox*>::iterator it = mParams->PlaneInput.begin();
  for ( ; it!= mParams->PlaneInput.end(); it++)
    if ((*it)->isChecked()) counter++;
  if (counter < 2) {
    SolveButton->setEnabled(false);
    it = mParams->PlaneInput.begin();
    for ( ; it!=mParams->PlaneInput.end(); it++)
      (*it)->setEnabled(true);
  } else if (counter == 2) {
    SolveButton->setEnabled(true);
    it = mParams->PlaneInput.begin();
    for ( ; it!=mParams->PlaneInput.end(); it++) {
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

void
LEDIndicator::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setBrush(QBrush(mColor, Qt::SolidPattern));
  p.drawEllipse(0,0,15,15);
}
