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
// Author(s): Matei T. Ciocarlie
//
// $Id: gfoDlg.cpp,v 1.6 2009/09/19 00:35:12 cmatei Exp $
//
//######################################################################

#include "gfoDlg.h"

#include <QInputDialog>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "grasp.h"
#include "body.h"
#include "matrix.h"
#include "mainWindow.h"

// for hand-specific optimizations, might be temporary
#include "mcGrip.h"
#include "humanHand.h"

#include "debug.h"

#define PROF_ENABLED
#include "profiling.h"

GFODlg::GFODlg(MainWindow *mw, Hand *h, QWidget *parent) : QDialog(parent), mMainWindow(mw), mHand(h) 
{
  setupUi(this);
  statusLabel->setText("Status: optimization off");
  optimizationTypeBox->insertItem("Contact force existence");
  optimizationTypeBox->insertItem("Contact force optimization");
  optimizationTypeBox->insertItem("Grasp force existence");
  optimizationTypeBox->insertItem("Grasp force optimization");
  optimizationTypeBox->insertItem("Compliant joint equilibrium");
  optimizationTypeBox->insertItem("DOF force equilibrium");
  if (mHand->isA("McGrip")) {
    optimizationTypeBox->insertItem("McGrip tendon route");
    optimizationTypeBox->insertItem("McGrip joint equilibrium");
  }
  QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButtonClicked()));
  QObject::connect(mHand, SIGNAL(configurationChanged()), this, SLOT(handConfigurationChanged()));
  QObject::connect(optimizationOnBox, SIGNAL(clicked()), this, SLOT(optimizationOnBoxClicked()));
}

GFODlg::~GFODlg()
{
  mMainWindow->clearContactsList();
}

void 
GFODlg::optimizationOnBoxClicked()
{
  if (!optimizationOnBox->isChecked()) {
    statusLabel->setText("Status: optimization off");
  } else {
    runOptimization();
  }
}

void
GFODlg::handConfigurationChanged()
{
  if (!optimizationOnBox->isChecked()) {
    return;
  }
  runOptimization();
}

void
GFODlg::runOptimization()
{
  mHand->getGrasp()->update();
  if (mHand->getGrasp()->getObject()) {
    mHand->getGrasp()->getObject()->resetExtWrenchAcc();	
  }
  
  if (optimizationTypeBox->currentText()=="Grasp force existence") {
    graspForceOptimization(Grasp::GRASP_FORCE_EXISTENCE);
  } else if (optimizationTypeBox->currentText()=="Grasp force optimization") {
    graspForceOptimization(Grasp::GRASP_FORCE_OPTIMIZATION);
  } else if (optimizationTypeBox->currentText()=="Contact force existence") {
    graspForceOptimization(Grasp::CONTACT_FORCE_EXISTENCE);
  } else if (optimizationTypeBox->currentText()=="Contact force optimization") {
    graspForceOptimization(Grasp::CONTACT_FORCE_OPTIMIZATION);
  } else if (optimizationTypeBox->currentText()=="Compliant joint equilibrium") {
    compliantEquilibriumOptimization(false);
  } else if (optimizationTypeBox->currentText()=="DOF force equilibrium") {
    compliantEquilibriumOptimization(true);
  } else if (optimizationTypeBox->currentText()=="McGrip tendon route") {
    tendonRouteOptimization();
  } else if (optimizationTypeBox->currentText()=="McGrip joint equilibrium") {
    mcgripEquilibrium();
  } else {
    DBGA("Unkown option selected in optimization box");
  }
}

void 
GFODlg::displayResults(int result)
{
  if (result < 0) {
    statusLabel->setText("Status: optimization error");
    mMainWindow->clearContactsList();
  } else if (result > 0) {
    mMainWindow->clearContactsList();
    statusLabel->setText("Status: problem unfeasible");
  } else {
    statusLabel->setText("Status: optimization successful");
    graspItGUI->getIVmgr()->drawDynamicForces();
    //keep in mind that World::updateGrasps() will overwrite this and use the wrenches
    //to draw the worst case disturbance instead
    graspItGUI->getIVmgr()->drawUnbalancedForces();
    mMainWindow->updateContactsList();
  }
}

void
GFODlg::mcgripEquilibrium()
{
  if (!mHand->isA("McGrip")) {
    DBGA("Hand is not a McGrip!");
    return;
  }
  int result = static_cast<McGrip*>(mHand)->jointTorqueEquilibrium();
  displayResults(result);
}

void GFODlg::tendonRouteOptimization()
{
  if (!mHand->isA("McGrip")) {
    DBGA("Hand is not a McGrip!");
    return;
  }
  /*
  //tendon route optimization
  Matrix l(6,1);
  int result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonRouteOptimization(&l);
  DBGA("l matrix:\n" << l);
  */
  
  //tendon and construction optimization with new formulation
  Matrix p(8,1);
  double obj;
  int result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonAndHandOptimization(&p, obj);
  DBGA("p matrix:\n" << p);
  displayResults(result);
  
  /*
    Matrix *a, *B;
    static_cast<McGrip*>(mHand)->getRoutingMatrices(&B, &a);
    delete a;
    delete B;
  */
}

void
GFODlg::compliantEquilibriumOptimization(bool useDynamicDofForce)
{
  Matrix tau(mHand->staticJointTorques(useDynamicDofForce));
  int result = mHand->getGrasp()->computeQuasistaticForces(tau);
  displayResults(result);
}

void
GFODlg::graspForceOptimization(int computation)
{
  Matrix tau(Matrix::ZEROES<Matrix>(mHand->getNumJoints(),1));
  int result = mHand->getGrasp()->computeQuasistaticForcesAndTorques(&tau, computation);
  if (!result) {
    DBGA("Optimal joint torques:\n" << tau);
  }
  displayResults(result);
}
