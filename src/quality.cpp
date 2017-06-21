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

/*! \file
  \brief Implements base QualityMeasure class and specifc qm classes.
*/

#include "quality.h"

#include <QGridLayout>
#include <QBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <q3groupbox.h>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

#include "grasp.h"
#include "gws.h"
#include "matrix.h"
#include "robot.h"

#include <limits>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//Static class variables

//! A list of the possible qm types expressed as strings
const char *QualityMeasure::TYPE_LIST[] = {"Epsilon", "Volume", "PCR", "PGR", NULL};

const char *QualEpsilon::type = "Epsilon";
const char *QualVolume::type = "Volume";
const char *QualPCR::type = "PCR";
const char *QualPGR::type = "PGR";

/*!
  Sets the name of this QM to the text contained within the name widget.
  Sets the grasp pointer for this qm to the grasp in qmData.
*/
QualityMeasure::QualityMeasure(qmDlgDataT *qmData)
{
  name = qmData->qmName->text();
  grasp = qmData->grasp;
}

QualityMeasure::QualityMeasure(Grasp *g, QString n)
{
  grasp = g;
  name = n;
}

/*!
  Stub Destructor
*/
QualityMeasure::~QualityMeasure()
{
#ifdef GRASPITDBG
  std::cout << "deleting QualityMeasure" << std::endl;
#endif
}


/*!
  Calls the appropriate buildParameterArea method from the specific subclass
  of the quality measure contained within qmData.  The type of the qm is
  determined by examining the string qmData->qmType.
*/
void
QualityMeasure::buildParamArea(qmDlgDataT *qmData)
{
  if (!strcmp(qmData->qmType, QualEpsilon::getClassType())) {
    QualEpsilon::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType, QualVolume::getClassType())) {
    QualVolume::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPCR::getClassType())) {
    QualPCR::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPGR::getClassType())) {
    QualPGR::buildParamArea(qmData);
  }
}

/*!
  Creates an instance of the specific subclass that is named in the string
  pointed to by qmData->qmType.
*/
QualityMeasure *
QualityMeasure::createInstance(qmDlgDataT *qmData)
{
  if (!strcmp(qmData->qmType, QualEpsilon::getClassType())) {
    return new QualEpsilon(qmData);
  }

  else if (!strcmp(qmData->qmType, QualVolume::getClassType())) {
    return new QualVolume(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPCR::getClassType())) {
    return new QualPCR(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPGR::getClassType())) {
    return new QualPGR(qmData);
  }

  return NULL;
}


///////////////////////////////////////////////////////////////////////////////
//
//                                QualEpsilon
//
///////////////////////////////////////////////////////////////////////////////

//! Structure holding pointers to UI items specific to this quality measure
/*!
  Quality parameter structure that simply holds a pointer to the gws type
  combo box in the parameter area.
*/
struct QualEpsilonParamT {
  QComboBox *gwsTypeComboBox, *twsTypeComboBox;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualEpsilon::QualEpsilon(qmDlgDataT *data) : QualityMeasure(data)
{
  QualEpsilonParamT *params = (QualEpsilonParamT *)data->paramPtr;
  gws = grasp->addGWS(params->gwsTypeComboBox->currentText().latin1());
}

QualEpsilon::QualEpsilon(Grasp *g, QString n, const char *gwsType) : QualityMeasure(g, n)
{
  gws = grasp->addGWS(gwsType);
}
/*!
  Removes the GWS used for this qm from the grasp.  The grasp will delete it
  if necessary.
*/
QualEpsilon::~QualEpsilon()
{
#ifdef GRASPITDBG
  std::cout << "deleting QualEpsilon" << std::endl;
#endif
  grasp->removeGWS(gws);
}

/*!
  Returns the minimum distance from the origin to any of the hyperplanes
  defining the boundary of the GWS.
*/
double
QualEpsilon::evaluate()
{
  double minOffset = std::numeric_limits<double>::max();

  if (!gws->hyperPlanes) {
#ifdef GRASPITDBG
    std::cout << "hyperplanes is NULL" << std::endl;
#endif
    return -1.0;
  }

#ifdef GRASPITDBG
  std::cout << "numHyperPlanes" << gws->numHyperPlanes << std::endl;
#endif

  for (int i = 0; i < gws->numHyperPlanes; i++) {
    if (i == 0 || -(gws->hyperPlanes[i][6]) < minOffset) {
      minOffset = -(gws->hyperPlanes[i][6]);
      if (minOffset < 0) { return -1.0; }     // not a force-closure grasp
    }
  }

  val = minOffset;
  return val;
}

double
QualEpsilon::evaluate3D()
{
  if (!gws->hyperPlanes) {
#ifdef GRASPITDBG
    std::cout << "hyperplanes is NULL" << std::endl;
#endif
    return -1.0e3;
  }

#ifdef GRASPITDBG
  std::cout << "numHyperPlanes" << gws->numHyperPlanes << std::endl;
#endif

  double minOffset = -1.0e3;
  for (int i = 0; i < gws->numHyperPlanes; i++) {
    if (gws->hyperPlanes[i][6] > minOffset) {
      minOffset = gws->hyperPlanes[i][6];
    }
  }
  val = - minOffset;
  return val;
}

/*!
  Builds an epsilon qm parameter area within the quality measure dialog
  box.  This includes building a QT combo box for the GWS selection and
  one for the TWS selection.  However, currently there is only one possible
  TWS.  In the future, there may be ways to define others.
*/
void
QualEpsilon::buildParamArea(qmDlgDataT *qmData)
{
  int i;
  static QualEpsilonParamT params;

  QualEpsilon *currQM = (QualEpsilon *)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building qualepsilon" << std::endl;
#endif

  QLayout *l = new QGridLayout(qmData->settingsArea, 2, 2, 1);
  l->setAutoAdd(true);

  // create the GWS type menu
  new QLabel(QString("Limit unit GWS using:"), qmData->settingsArea);
  params.gwsTypeComboBox = new QComboBox(qmData->settingsArea, "gwsComboBox");

  new QLabel(QString("Task Wrench Space (TWS):"), qmData->settingsArea);
  params.twsTypeComboBox = new QComboBox(qmData->settingsArea, "twsComboBox");

  // count the number of possible gws types
  for (i = 0; GWS::TYPE_LIST[i]; i++) {
    params.gwsTypeComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
    if (currQM && !strcmp(currQM->gws->getType(), GWS::TYPE_LIST[i])) {
      params.gwsTypeComboBox->setCurrentItem(i);
    }
  }

  params.twsTypeComboBox->insertItem(QString("Unit Ball"));

  qmData->paramPtr = &params;
}

///////////////////////////////////////////////////////////////////////////////
//
//                                QualVolume
//
///////////////////////////////////////////////////////////////////////////////

//! Structure holding pointers to UI items specific to this quality measure
/*!
  Quality parameter structure that simply holds a pointer to the gws type
  combo box in the parameter area.
*/
struct QualVolumeParamT {
  QWidget *gwsTypeComboBox;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualVolume::QualVolume(qmDlgDataT *data) : QualityMeasure(data)
{
  QComboBox *gwsType = (QComboBox *)data->paramPtr;
  gws = grasp->addGWS(gwsType->currentText().latin1());
}

QualVolume::QualVolume(Grasp *g, QString n, const char *gwsType) : QualityMeasure(g, n)
{
  gws = grasp->addGWS(gwsType);
}


/*!
  Removes the GWS used for this qm from the grasp.  The grasp will delete it
  if necessary.
*/
QualVolume::~QualVolume()
{
#ifdef GRASPITDBG
  std::cout << "deleting QualVolume" << std::endl;
#endif
  grasp->removeGWS(gws);
}

/*!
  Returns the volume of the GWS.  The GWS is responsible for computing its
  volume when it is updated.
*/
double
QualVolume::evaluate()
{
  val = gws->hullVolume;
  return val;
}

double
QualVolume::evaluate3D()
{
  val = gws->hullVolume;
  return val;
}

/*!
  Builds a volume qm parameter area within the quality measure dialog
  box.  This is simply building a QT combo box for the GWS selection.
*/
void
QualVolume::buildParamArea(qmDlgDataT *qmData)
{
  int i;
  QualVolume *currQM = (QualVolume *)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building qualvolume" << std::endl;
#endif

  QBoxLayout *l = new QHBoxLayout(qmData->settingsArea);
  l->setAutoAdd(true);

  // create the GWS type menu
  new QLabel(QString("Limit unit GWS using:"), qmData->settingsArea);
  QComboBox *gwsComboBox = new QComboBox(qmData->settingsArea, "gwsComboBox");

  // count the number of possible gws types
  for (i = 0; GWS::TYPE_LIST[i]; i++) {
    gwsComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
    if (currQM && !strcmp(currQM->gws->getType(), GWS::TYPE_LIST[i])) {
      gwsComboBox->setCurrentItem(i);
    }
  }

  qmData->paramPtr = gwsComboBox;
}

///////////////////////////////////////////////////////////////////////////////
//
//                                QualPCR
//
///////////////////////////////////////////////////////////////////////////////

//! Structure holding pointers to UI items specific to this quality measure
struct QualPCRParamT {
  QLineEdit *wrenchInput;
  QLineEdit *FxInput;
  QLineEdit *FyInput;
  QLineEdit *FzInput;
  QLineEdit *MxInput;
  QLineEdit *MyInput;
  QLineEdit *MzInput;

  QLineEdit *maxForceInput;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualPCR::QualPCR(qmDlgDataT *data) : QualityMeasure(data)
{
  QualPCRParamT *params = (QualPCRParamT *)data->paramPtr;

  mWrenchMultiplier = params->wrenchInput->text().toDouble();

  mWrench.resize(6, 0.0);
  mWrench[0] = params->FxInput->text().toDouble();
  mWrench[1] = params->FyInput->text().toDouble();
  mWrench[2] = params->FzInput->text().toDouble();
  mWrench[3] = params->MxInput->text().toDouble();
  mWrench[4] = params->MyInput->text().toDouble();
  mWrench[5] = params->MzInput->text().toDouble();

  mMaxForce = params->maxForceInput->text().toDouble();
}

QualPCR::~QualPCR()
{
#ifdef GRASPITDBG
   std::cout << "deleting QualPCR" << std::endl;
#endif
}

double
QualPCR::evaluate()
{
  Matrix dw(&mWrench[0], 6, 1, true);
  double wrenchMultiplier = mWrenchMultiplier;
  if (mWrenchMultiplier) dw.multiply(wrenchMultiplier);

  return grasp->evaluatePCR(dw, mMaxForce);
}

double
QualPCR::evaluate3D()
{
  return 0;
}

/*!
  Builds a PCR qm parameter area within the quality measure dialog
  box such that the user can input the applied wrench and the 
  desired maximum contact force
*/
void
QualPCR::buildParamArea(qmDlgDataT *qmData)
{
  static QualPCRParamT params;
  QualPCR *currQM = (QualPCR*)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building QualPCR" << std::endl;
#endif

  std::vector<double> wrench(6, 0.0);
  double wrenchMultiplier = 1.0;
  double maxForce = 10.0;
  if (currQM) {
    wrench = currQM->mWrench;
    wrenchMultiplier = currQM->mWrenchMultiplier;
    maxForce = currQM->mMaxForce;
  } else if (qmData->grasp->isGravitySet()) {
    wrench = getGravityWrench(qmData->grasp);
    wrenchMultiplier = qmData->grasp->getObject()->getMass() * 1.0e-3 * 9.80665 * 1.0e6;
  } 

  QGridLayout *wl = new QGridLayout(qmData->settingsArea,8,2);

  wl->addWidget(new QLabel(QString("Multiplier:")));
  params.wrenchInput = new QLineEdit();
  params.wrenchInput->setText(QString::number(wrenchMultiplier));
  wl->addWidget(params.wrenchInput);

  wl->addWidget(new QLabel(QString("Force X:")));
  params.FxInput = new QLineEdit();
  params.FxInput->setText(QString::number(wrench[0]));
  wl->addWidget(params.FxInput);

  wl->addWidget(new QLabel(QString("Force Y:")));
  params.FyInput = new QLineEdit();
  params.FyInput->setText(QString::number(wrench[1]));
  wl->addWidget(params.FyInput);

  wl->addWidget(new QLabel(QString("Force Z:")));
  params.FzInput = new QLineEdit();
  params.FzInput->setText(QString::number(wrench[2]));
  wl->addWidget(params.FzInput);

  wl->addWidget(new QLabel(QString("Torque X:")));
  params.MxInput = new QLineEdit();
  params.MxInput->setText(QString::number(wrench[3]));
  wl->addWidget(params.MxInput);

  wl->addWidget(new QLabel(QString("Torque Y:")));
  params.MyInput = new QLineEdit();
  params.MyInput->setText(QString::number(wrench[4]));
  wl->addWidget(params.MyInput);

  wl->addWidget(new QLabel(QString("Torque Z:")));
  params.MzInput = new QLineEdit();
  params.MzInput->setText(QString::number(wrench[5]));
  wl->addWidget(params.MzInput);

  wl->addWidget(new QLabel(QString("Maximum Force:")));
  params.maxForceInput = new QLineEdit();
  params.maxForceInput->setText(QString::number(maxForce));
  wl->addWidget(params.maxForceInput);

  qmData->paramPtr = &params;
}

std::vector<double> QualPCR::getGravityWrench(Grasp *grasp)
{
  std::vector<double> wrench(6, 0.0);
  vec3 hand_z(0,0,1);// * grasp->getHand()->getTran();
  wrench[0] = hand_z.x();
  wrench[1] = hand_z.y();
  wrench[2] = hand_z.z();
  return wrench;
}


///////////////////////////////////////////////////////////////////////////////
//
//                                QualPGR
//
///////////////////////////////////////////////////////////////////////////////

//! Structure holding pointers to UI items specific to this quality measure
struct QualPGRParamT {
  QLineEdit *wrenchInput;
  QLineEdit *FxInput;
  QLineEdit *FyInput;
  QLineEdit *FzInput;
  QLineEdit *MxInput;
  QLineEdit *MyInput;
  QLineEdit *MzInput;

  QLineEdit *maxForceInput;

  QLineEdit *maxContactsInput;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualPGR::QualPGR(qmDlgDataT *data) : QualityMeasure(data)
{
  QualPGRParamT *params = (QualPGRParamT *)data->paramPtr;

  mWrenchMultiplier = params->wrenchInput->text().toDouble();

  mWrench.resize(6, 0.0);
  mWrench[0] = params->FxInput->text().toDouble();
  mWrench[1] = params->FyInput->text().toDouble();
  mWrench[2] = params->FzInput->text().toDouble();
  mWrench[3] = params->MxInput->text().toDouble();
  mWrench[4] = params->MyInput->text().toDouble();
  mWrench[5] = params->MzInput->text().toDouble();

  mMaxForce = params->maxForceInput->text().toDouble();

  mMaxContacts = params->maxContactsInput->text().toInt();
}

QualPGR::~QualPGR()
{
#ifdef GRASPITDBG
   std::cout << "deleting QualPGR" << std::endl;
#endif
}

double
QualPGR::evaluate()
{
  Matrix dw(&mWrench[0], 6, 1, true);
  double wrenchMultiplier = mWrenchMultiplier;
  if (mWrenchMultiplier) dw.multiply(wrenchMultiplier);

  return grasp->evaluatePGR(dw, mMaxForce, mMaxContacts);
}

double
QualPGR::evaluate3D()
{
  return 0;
}

/*!
  Builds a PCR qm parameter area within the quality measure dialog
  box such that the user can input the applied wrench and the 
  desired maximum contact force
*/
void
QualPGR::buildParamArea(qmDlgDataT *qmData)
{
  static QualPGRParamT params;
  QualPGR *currQM = (QualPGR*)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building QualPGR" << std::endl;
#endif

  std::vector<double> wrench(6, 0.0);
  double wrenchMultiplier = 1.0;
  double maxForce = 1.0;
  int maxContacts = 8;
  if (currQM) {
    wrench = currQM->mWrench;
    wrenchMultiplier = currQM->mWrenchMultiplier;
    maxForce = currQM->mMaxForce;
    maxContacts = currQM->mMaxContacts;
  } else if (qmData->grasp->isGravitySet()) {
    wrench = QualPCR::getGravityWrench(qmData->grasp);
    wrenchMultiplier = qmData->grasp->getObject()->getMass() * 1.0e-3 * 9.80665 * 1.0e6;
  } 

  QGridLayout *wl = new QGridLayout(qmData->settingsArea,8,2);

  wl->addWidget(new QLabel(QString("Multiplier:")));
  params.wrenchInput = new QLineEdit();
  params.wrenchInput->setText(QString::number(wrenchMultiplier));
  wl->addWidget(params.wrenchInput);

  wl->addWidget(new QLabel(QString("Force X:")));
  params.FxInput = new QLineEdit();
  params.FxInput->setText(QString::number(wrench[0]));
  wl->addWidget(params.FxInput);

  wl->addWidget(new QLabel(QString("Force Y:")));
  params.FyInput = new QLineEdit();
  params.FyInput->setText(QString::number(wrench[1]));
  wl->addWidget(params.FyInput);

  wl->addWidget(new QLabel(QString("Force Z:")));
  params.FzInput = new QLineEdit();
  params.FzInput->setText(QString::number(wrench[2]));
  wl->addWidget(params.FzInput);

  wl->addWidget(new QLabel(QString("Torque X:")));
  params.MxInput = new QLineEdit();
  params.MxInput->setText(QString::number(wrench[3]));
  wl->addWidget(params.MxInput);

  wl->addWidget(new QLabel(QString("Torque Y:")));
  params.MyInput = new QLineEdit();
  params.MyInput->setText(QString::number(wrench[4]));
  wl->addWidget(params.MyInput);

  wl->addWidget(new QLabel(QString("Torque Z:")));
  params.MzInput = new QLineEdit();
  params.MzInput->setText(QString::number(wrench[5]));
  wl->addWidget(params.MzInput);

  wl->addWidget(new QLabel(QString("Max. Force:")));
  params.maxForceInput = new QLineEdit();
  params.maxForceInput->setText(QString::number(maxForce));
  wl->addWidget(params.maxForceInput);

  wl->addWidget(new QLabel(QString("Max. no. of contacts:")));
  params.maxContactsInput = new QLineEdit();
  params.maxContactsInput->setText(QString::number(maxContacts));
  wl->addWidget(params.maxContactsInput);

  qmData->paramPtr = &params;
}


///////////////////////////////////////////////////////////////////////////////
//
//                      More quality measures here
//
///////////////////////////////////////////////////////////////////////////////
