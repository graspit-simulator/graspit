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
//######################################################################

#include "graspit/quality/qualPCR.h"

#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>

#include "graspit/grasp.h"
#include "graspit/math/matrix.h"
#include "graspit/robot.h"
#include "graspit/contact/contact.h"
#include "graspit/debug.h"

const char *QualPCR::type = "PCR";

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

  return evaluatePCR(grasp, dw, mMaxForce);
}

double
QualPCR::evaluate3D()
{
  return 0;
}

/*! 
  Evaluate PCR quality metric as described in 'Contact and Grasp 
  Robustness Measures: Analysis and Experiment' (1997) by 
  Prattichizzo et al. Care must be taken to pass the show=false
  flag when calling from multiple threads as displayContactWrenches()
  and drawObjectWrench() are not thread safe
*/
double
QualPCR::evaluatePCR(Grasp *g, const Matrix &wrench, double maxForce, 
                     std::vector<int> states /*=empty*/, bool show /*=true*/)
{
  //use the pre-set list of contacts. This includes contacts on the palm, but
  //not contacts with other objects or obstacles
  int numContacts = g->getNumContacts();
  std::list<Contact*> contacts;
  for (int i=0; i<numContacts; i++) contacts.push_back(g->getContact(i));
  //if there are no contacts we are done
  if (contacts.empty()) {
    DBGA("No contacts");
    return -1;
  }
  
  //check if G is full rank
  Matrix H(g->contactModelMatrix(contacts.size(), states));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix G(matrixMultiply(g->graspMapMatrix(R), H.transposed()));
  if (G.rank() < 6) {
    DBGA("G not full rank");
    return -1;
  }

  //get only the joints on chains that make contact;
  std::list<Joint*> joints = g->getJointsOnContactChains();

  Matrix contactWrenches(6*contacts.size(), 1);
  double dmin = g->findOptimalContactForces(wrench, maxForce, contactWrenches, joints, contacts, states);
  if (dmin!=dmin) return -1;
  
  Matrix GRK(g->KweightedGinverse(joints, contacts, states));
  Matrix S(std::min(GRK.rows(), GRK.cols()), 1);
  Matrix U(GRK.rows(), GRK.rows());
  Matrix VT(GRK.cols(), GRK.cols());
  GRK.SVD(S,U,VT);

  Matrix GKGTInv(g->graspStiffness(joints, contacts));

  /*if (show) {
    displayContactWrenches(&contacts, contactWrenches);
    drawObjectWrench(wrench.negative());
  }
  drawObjectMovement(matrixMultiply(GKGTInv, wrench));*/

  return dmin/S.elem(0,0);
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
