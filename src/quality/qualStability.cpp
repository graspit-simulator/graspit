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
// $Id: quality.cpp,v 1.12 2009/03/31 15:36:58 cmatei Exp $
//
//######################################################################

#include "graspit/quality/qualStability.h"

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>

#include "graspit/grasp.h"
#include "graspit/graspSolver.h"
#include "graspit/debug.h"

const char *QualStability::type = "Stability";

//! Structure holding pointers to UI items specific to this quality measure
/*!
  Quality parameter structure that simply holds a pointer to the gws type
  combo box in the parameter area.
*/
struct QualStabilityParamT {
  QLineEdit *wrenchInput;
  QLineEdit *FxInput;
  QLineEdit *FyInput;
  QLineEdit *FzInput;
  QLineEdit *MxInput;
  QLineEdit *MyInput;
  QLineEdit *MzInput;

  QLineEdit *preloadInput;
  std::vector<QLineEdit*> JointInput;

  QCheckBox *stepInput;
  QCheckBox *iterInput;
  QCheckBox *coneInput;
  QCheckBox *rigidInput;
  QCheckBox *mapInput;
};

QualStability::QualStability(Grasp *g, QString n) : QualityMeasure(g, n)
{
  mWrenchMultiplier = 1.0;
  mWrench.resize(6, 0.0);

  mPreloadMultiplier = 1.0;
  if (g->getHand()->inherits("HumanHand")) 
    mPreload.resize(((HumanHand*)g->getHand())->getNumTendons(), 0.0);
  else mPreload.resize(g->getJointsOnContactChains().size(), 0.0);

  mSingle_step = false;
  mIterative = false;
  mCone_movement = false;
  mRigid = false;
  mMap = false;
}

QualStability::QualStability(qmDlgDataT *data) : QualityMeasure(data)
{
  QualStabilityParamT *params = (QualStabilityParamT *)data->paramPtr;

  mWrenchMultiplier = params->wrenchInput->text().toDouble();
  mWrench.resize(6, 0.0);
  mWrench[0] = params->FxInput->text().toDouble();
  mWrench[1] = params->FyInput->text().toDouble();
  mWrench[2] = params->FzInput->text().toDouble();
  mWrench[3] = params->MxInput->text().toDouble();
  mWrench[4] = params->MyInput->text().toDouble();
  mWrench[5] = params->MzInput->text().toDouble();

  mPreloadMultiplier = params->preloadInput->text().toDouble();
  mPreload.resize(params->JointInput.size());
  for (size_t i=0; i<mPreload.size(); i++) {
    mPreload[i] = params->JointInput[i]->text().toDouble();
  }

  mSingle_step = params->stepInput->isChecked();
  mIterative = params->iterInput->isChecked();
  mCone_movement = params->coneInput->isChecked();
  mRigid = params->rigidInput->isChecked();
  mMap = params->mapInput->isChecked();
}

QualStability::~QualStability()
{
#ifdef GRASPITDBG
   std::cout << "deleting QualStability" << std::endl;
#endif
}

void QualStability::setWrench(double wrenchMultiplier, const std::vector<double> &wrench) {
  assert(wrench.size() == 6);
  mWrenchMultiplier = wrenchMultiplier;
  mWrench = wrench;
}

/*!
  Returns 1 if the grasp can withstand the wrench, 0 if it cannot and -1 if
  there is an error.
*/
double
QualStability::evaluate()
{
  GraspSolver grasp_solver(grasp);

  bool tendon = grasp->getHand()->inherits("HumanHand");
  if (tendon) {
    if (((HumanHand*)grasp->getHand())->getNumTendons() != mPreload.size()) {
      DBGA("Tendon preload size does not match current grasp");
      return -1;
    }
  } else {
    if(grasp->getJointsOnContactChains().size() != mPreload.size()) {
      DBGA("Joint preload size does not match current grasp");
      return -1;
    }
  }

  Matrix preload(mPreload.size(), 1);
  for (size_t i=0; i<mPreload.size(); i++) {
    preload.elem(i,0) = mPreload[i] * mPreloadMultiplier;
  }

  if (mMap) return grasp_solver.create2DMap(preload, mSingle_step, tendon, mIterative, mCone_movement, mRigid);

  Matrix dw(&mWrench[0], 6, 1, true);
  double wrenchMultiplier = mWrenchMultiplier;
  if (mWrenchMultiplier) dw.multiply(wrenchMultiplier);

  if (wrenchMultiplier != 0) {
    //try specific wrench
    if (mIterative) return grasp_solver.checkWrenchIterative(preload, dw, mSingle_step, mCone_movement, tendon, mRigid);
    else return grasp_solver.checkWrenchNonIterative(preload, dw, mSingle_step, tendon, mRigid);
  } else {
    //try binary search in normalized wrench direction
    dw.multiply(1.0/dw.fnorm());
    if (mIterative) return grasp_solver.findMaximumWrenchIterative(preload, dw, mSingle_step, mCone_movement, tendon, mRigid);
    else return grasp_solver.findMaximumWrenchNonIterative(preload, dw, mSingle_step, tendon, mRigid);
  }
}

double
QualStability::evaluate3D()
{
  return 0;
}

/*!
  Builds an Stability qm parameter area within the quality measure dialog
  box.  This consists of 6 boxes for inputting the applied wrench. Each of
  the six components of the wrench is specified individually.
*/
void
QualStability::buildParamArea(qmDlgDataT *qmData)
{
  static QualStabilityParamT params;
  QualStability *currQM = (QualStability*)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building qualStability" << std::endl;
#endif

  QHBoxLayout *hl = new QHBoxLayout(qmData->settingsArea);

  QVBoxLayout *forceLayout = new QVBoxLayout(hl);
  forceLayout->setSpacing(0);
  forceLayout->addWidget(new QLabel(QString("Applied wrench:")));

  QLayout *wl = new QGridLayout(forceLayout,4,2,1);
  wl->setSpacing(0);

  std::vector<double> wrench(6, 0.0);
  double wrenchMultiplier = 1.0;
  if (currQM) {
    wrench = currQM->mWrench;
    wrenchMultiplier = currQM->mWrenchMultiplier;
  } else if (qmData->grasp->isGravitySet()) {
    wrench = QualStability::getGravityWrench(qmData->grasp);
    wrenchMultiplier = qmData->grasp->getObject()->getMass() * 1.0e-3 * 9.80665 * 1.0e6;
  } 

  wl->addWidget(new QLabel(QString("Mult:")));
  params.wrenchInput = new QLineEdit();
  if (currQM) params.wrenchInput->setText(QString::number(currQM->mWrenchMultiplier));
  else params.wrenchInput->setText(QString::number(wrenchMultiplier));
  wl->addWidget(params.wrenchInput);

  wl->addWidget(new QLabel(QString("Fx:")));
  params.FxInput = new QLineEdit();
  params.FxInput->setText(QString::number(wrench[0]));
  wl->addWidget(params.FxInput);

  wl->addWidget(new QLabel(QString("Fy:")));
  params.FyInput = new QLineEdit();
  params.FyInput->setText(QString::number(wrench[1]));
  wl->addWidget(params.FyInput);

  wl->addWidget(new QLabel(QString("Fz:")));
  params.FzInput = new QLineEdit();
  params.FzInput->setText(QString::number(wrench[2]));
  wl->addWidget(params.FzInput);

  wl->addWidget(new QLabel(QString("Tx:")));
  params.MxInput = new QLineEdit();
  params.MxInput->setText(QString::number(wrench[3]));
  wl->addWidget(params.MxInput);

  wl->addWidget(new QLabel(QString("Ty:")));
  params.MyInput = new QLineEdit();
  params.MyInput->setText(QString::number(wrench[4]));
  wl->addWidget(params.MyInput);

  wl->addWidget(new QLabel(QString("Tz:")));
  params.MzInput = new QLineEdit();
  params.MzInput->setText(QString::number(wrench[5]));
  wl->addWidget(params.MzInput);

  QVBoxLayout *jointLayout = new QVBoxLayout(hl);
  jointLayout->setSpacing(0);
  int preloadSize;
  if (qmData->grasp->getHand()->inherits("HumanHand")) {
    jointLayout->addWidget(new QLabel(QString("Tendon Preloads:")));
    preloadSize = ((HumanHand*)qmData->grasp->getHand())->getNumTendons();
  } else {
    jointLayout->addWidget(new QLabel(QString("Preload torques:")));
    qmData->grasp->collectContacts();
    std::list<Joint*> joints = qmData->grasp->getJointsOnContactChains(); 
    preloadSize = joints.size();
  }

  QLayout *jl = new QGridLayout(jointLayout,4,2,1);
  jl->setSpacing(0);

  jl->addWidget(new QLabel(QString("Mult:")));
  params.preloadInput = new QLineEdit();
  if (currQM) params.preloadInput->setText(QString::number(currQM->mPreloadMultiplier));
  else params.preloadInput->setText(QString::number(1.0));
  jl->addWidget(params.preloadInput);

  params.JointInput.clear();

  std::vector<double> preload(preloadSize, 0.0);
  if (currQM && preload.size() == currQM->mPreload.size()) {
    preload = currQM->mPreload;
  } else {
    preload = QualStability::getDefaultPreload(qmData->grasp);
  }

  for (int i=0; i<preloadSize; i++) {
    jl->addWidget(new QLabel(QString("P")+QString::number(i)+QString(":")));
    QLineEdit* line = new QLineEdit();
    jl->addWidget(line);
    params.JointInput.push_back(line);
    line->setText(QString::number(preload[i]));
  }

  QVBoxLayout *optionsLayout = new QVBoxLayout(hl);
  optionsLayout->setSpacing(0);
  optionsLayout->setAlignment(Qt::AlignTop);
  optionsLayout->addWidget(new QLabel(QString("Solver Options:")));

  QLayout *ol = new QGridLayout(optionsLayout,4,2,1);
  ol->setSpacing(0);

  bool single_step = false;
  bool iterative = false;
  bool cone_movement = false;
  bool rigid = false;
  bool map = false;
  if (currQM) {
    single_step = currQM->mSingle_step;
    iterative = currQM->mIterative;
    cone_movement = currQM->mCone_movement;
    rigid = currQM->mRigid;
    map = currQM->mMap;
  }

  ol->addWidget(new QLabel(QString("Single step:")));
  params.stepInput = new QCheckBox();
  params.stepInput->setChecked(single_step);
  ol->addWidget(params.stepInput);
  
  ol->addWidget(new QLabel(QString("Iterative:")));
  params.iterInput = new QCheckBox();
  params.iterInput->setChecked(iterative);
  ol->addWidget(params.iterInput);  

  ol->addWidget(new QLabel(QString("Cone Movement:")));
  params.coneInput = new QCheckBox();
  params.coneInput->setChecked(cone_movement);
  params.coneInput->setEnabled(iterative);
  ol->addWidget(params.coneInput);

  ol->addWidget(new QLabel(QString("Rigid:")));
  params.rigidInput = new QCheckBox();
  params.rigidInput->setChecked(rigid);
  params.rigidInput->setDisabled(qmData->grasp->getHand()->inherits("HumanHand"));
  params.rigidInput->setDisabled(single_step);
  ol->addWidget(params.rigidInput);

  ol->addWidget(new QLabel(QString("2D Map:")));
  params.mapInput = new QCheckBox();
  params.mapInput->setChecked(map);
  ol->addWidget(params.mapInput);

  qmData->paramPtr = &params;

  QObject::connect(params.iterInput, SIGNAL (toggled(bool)),
    params.coneInput, SLOT (setEnabled(bool)));
  QObject::connect(params.stepInput, SIGNAL (toggled(bool)),
    params.rigidInput, SLOT (setDisabled(bool)));
  QObject::connect(params.rigidInput, SIGNAL (toggled(bool)),
    params.stepInput, SLOT (setDisabled(bool)));
}

std::vector<double> QualStability::getGravityWrench(Grasp *grasp)
{
  std::vector<double> wrench(6, 0.0);
  vec3 hand_z(0,0,1);// * grasp->getHand()->getTran();
  wrench[0] = hand_z.x();
  wrench[1] = hand_z.y();
  wrench[2] = hand_z.z();
  return wrench;
}

std::vector<double> QualStability::getDefaultPreload(Grasp *grasp)
{
  Hand *hand = grasp->getHand();  
  std::list<Joint*> joints = grasp->getJointsOnContactChains(); 
  std::vector<double> joint_preload(joints.size(), 0.0);

  if (hand->isA("Barrett")) {
    //specific to Barrett hand
    int jNum=0;
    for (std::list<Joint*>::iterator it = joints.begin(); it!=joints.end(); it++, jNum++) {
      if (hand->getDOF((*it)->getDOFNum())->getDefaultVelocity() > 0) {
  if ( (*it)->getNum() == 1 ) {
    //proximal on f1
    joint_preload[jNum] = 1;
  } else if ( (*it)->getNum() == 4 ) {
    //proximal on f2
    joint_preload[jNum] = 1;       
  } else if ((*it)->getChainNum() == 2) {
    //thumb
    KinematicChain *chain = hand->getChain((*it)->getChainNum());
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
      if (hand->getDOF((*it)->getDOFNum())->getDefaultVelocity() > 0) {
    joint_preload[jNum] = 1.0;
      }
    }
  }
  
  return joint_preload;
}
