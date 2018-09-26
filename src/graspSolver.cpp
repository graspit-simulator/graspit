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
// Author(s): Maximilian K.G. Haas-Heger and Matei T. Ciocarlie
//
// $Id: gws.h,v 1.9 2016/07/25 15:21:24 mkghaas Exp $
//
//######################################################################

// Implements the Grasp Solver class

#include "graspit/graspSolver.h"
#include "graspit/debug.h"
#include "graspit/graspitCore.h"
#include "graspit/ivmgr.h"
#include <sstream>

const double GraspSolver::kBetaMaxBase = 50;
const double GraspSolver::kTauMaxBase = 2500;
const double GraspSolver::kAlphaMaxBase = 125;
const double GraspSolver::kQMaxBase = 50;
const double GraspSolver::kKMaxBase = 250;
const double GraspSolver::kMaxMotion = 5.0;

#define BIN_ULIMIT 1

// Constructor
GraspSolver::GraspSolver(Grasp *grasp)
{
  g = grasp;
  contacts = g->getContactList();
  joints = g->getJointsOnContactChains();

  numContacts = contacts.size();
  numJoints = joints.size();
}

//  ------------------------  Optimization Objectives  ------------------------------  //

void
GraspSolver::springDeformationObjective(GraspStruct &P)
{
  P.Q = new Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = new Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  // Selection matrix for normal forces
  Matrix S(normalForceSelectionMatrix(contacts));
  Matrix STS(matrixMultiply(S.transposed(), S));
  P.Q->copySubMatrixBlockIndices(P.var["beta"], P.var["beta"], STS);
}

void
GraspSolver::resultantWrenchObjective(GraspStruct &P)
{
  P.Q = new Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = new Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.Q->copySubMatrixBlockIndices(P.var["r"], P.var["r"], Matrix::EYE(6, 6));
}

void
GraspSolver::virtualLimitsObjective(GraspStruct &P)
{
  P.Q = new Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = new Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.cj->copySubMatrixBlockIndices(0, P.var["v"], Matrix::EYE(1,1));
}

void
GraspSolver::resultantAndVLObjective(GraspStruct &P)
{
  P.Q = new Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = new Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.Q->copySubMatrixBlockIndices(P.var["r"], P.var["r"], Matrix::EYE(6, 6));
  P.cj->copySubMatrixBlockIndices(0, P.var["v"], Matrix::EYE(1,1));
  P.cj->multiply(1.0e-6);
}

//  --------------------------  Equality Constraints  -------------------------------  //

void
GraspSolver::objectWrenchConstraint(GraspStruct &P, const Matrix &wrench)
{
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix G(g->graspMapMatrixFrictionEdges(R,D));

  Matrix Eq( Matrix::ZEROES<Matrix>(G.rows(), P.block_cols) );
  Eq.copySubMatrixBlockIndices(0, P.var["beta"], G);
  P.Eq_list.push_back(Eq);

  P.b_list.push_back(wrench);
}

void
GraspSolver::resultantWrenchConstraint(GraspStruct &P, const Matrix &wrench)
{
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix G(g->graspMapMatrixFrictionEdges(R,D));

  Matrix Eq( Matrix::ZEROES<Matrix>(G.rows(), P.block_cols) );
  Eq.copySubMatrixBlockIndices(0, P.var["beta"], G);
  Eq.copySubMatrixBlockIndices(0, P.var["r"],    Matrix::NEGEYE(6,6) );
  P.Eq_list.push_back(Eq);
  
  P.b_list.push_back(wrench);
}

void
GraspSolver::objectMovementConstraint(GraspStruct &P, const Matrix &startingX, const Matrix &directions)
{
  Matrix Eq( Matrix::ZEROES<Matrix>(6, P.block_cols) );

  Eq.copySubMatrixBlockIndices(0, P.var["x"], Matrix::EYE(6,6));
  Eq.copySubMatrixBlockIndices(0, P.var["s"], directions);
  P.Eq_list.push_back(Eq);

  P.b_list.push_back(startingX);
}

void
GraspSolver::movementAmplitudesConstraint(GraspStruct &P)
{
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix D(Contact::frictionForceBlockMatrix(contacts));

  // Matrix that selects only forces that are transmissible at a contact given its
  // friction model. This is needed to use the friction edge matrix D with contact
  // movement amplitudes alpha because Matrix D cannot represent relative contact 
  // rotations for example
  Matrix H( Matrix::ZEROES<Matrix>(RT.rows(), RT.rows()) );

  std::list<Contact*>::iterator it=contacts.begin();
  for (int i=0; it!=contacts.end(); it++, i++) {
    if ((*it)->getFrictionType() == PCWF)
      H.copySubMatrix(6*i, 6*i, Matrix::EYE(3,3));
    else {
      DBGA("Contact has friction type not yet supported");
      exit(0);
    }
  }

  Matrix HRT(matrixMultiply(H, RT));
  Matrix HJ(matrixMultiply(H, J));

  Matrix Eq( Matrix::ZEROES<Matrix>(D.rows(), P.block_cols) );
  Eq.copySubMatrixBlockIndices(0, P.var["alpha"], D);
  Eq.copySubMatrixBlockIndices(0, P.var["x"],     HRT);
  Eq.copySubMatrixBlockIndices(0, P.var["q"],     HJ.negative());
  P.Eq_list.push_back(Eq);

  P.b_list.push_back( Matrix::ZEROES<Matrix>(D.rows(), 1) );
}

void
GraspSolver::rigidJointsConstraint(GraspStruct &P)
{
  Matrix Eq(Matrix::ZEROES<Matrix>(numJoints, P.block_cols)); 
  Eq.copySubMatrixBlockIndices(0, P.var["q"], Matrix::EYE(numJoints, numJoints));
  P.Eq_list.push_back(Eq);
  P.b_list.push_back(Matrix::ZEROES<Matrix>(numJoints, 1));
}

void
GraspSolver::objectMotionConstraint(GraspStruct &P, const Matrix &wrench)
{
  Matrix Eq( Matrix::ZEROES<Matrix>(6, P.block_cols) );

  Eq.copySubMatrixBlockIndices(0, P.var["x"], Matrix::EYE(6,6));
  P.Eq_list.push_back(Eq);

  P.b_list.push_back(wrench);
}

//  -------------------------  Inequality Constraints  ------------------------------  //

void
GraspSolver::virtualSpringConstraint(GraspStruct &P, const Matrix &beta_p) 
{
  // stiffness
  double k = 1;

  std::vector<int> block_rows(4, numContacts);
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) );

  Matrix ib( Matrix::ZEROES<Matrix>(block_rows, 1) );

  Matrix J(g->contactJacobian(joints, contacts));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix S(normalForceSelectionMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix NJ(matrixMultiply(N, J));

  Matrix S_k(S);
  S_k.multiply(1/k);

  // vector of normal forces at contacts
  Matrix c(matrixMultiply(S, beta_p));

  // Contact force complementarity
  Matrix k1( Matrix::EYE(numContacts, numContacts) );
  Matrix k2(k1);
  k1.multiply(-BetaMax);
  for (int i=0; i<k1.rows(); i++) k1.elem(i,i) -= c.elem(i,0);
  k2.multiply(KMax);
  Matrix k3(numContacts,1);
  k3.setAllElements(KMax);

  InEq.copySubMatrixBlockIndices(0, P.var["beta"], S.negative());

  InEq.copySubMatrixBlockIndices(1, P.var["beta"], S_k.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["x"],    K);
  InEq.copySubMatrixBlockIndices(1, P.var["q"],    NJ.negative());

  InEq.copySubMatrixBlockIndices(2, P.var["beta"], S);
  InEq.copySubMatrixBlockIndices(2, P.var["y1"],   k1);

  InEq.copySubMatrixBlockIndices(3, P.var["beta"], S_k);
  InEq.copySubMatrixBlockIndices(3, P.var["x"],    K.negative());
  InEq.copySubMatrixBlockIndices(3, P.var["q"],    NJ);
  InEq.copySubMatrixBlockIndices(3, P.var["y1"],   k2);
  P.InEq_list.push_back(InEq);

  ib.copySubMatrixBlockIndices(0, 0, c);
  ib.copySubMatrixBlockIndices(2, 0, c.negative());
  ib.copySubMatrixBlockIndices(3, 0, k3);
  P.ib_list.push_back(ib);
}

void
GraspSolver::nonBackdrivableJointConstraint(GraspStruct &P, const Matrix &preload, 
  const Matrix &beta_p)
{
  std::vector<int> block_rows(4, numJoints);
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) ); 
  Matrix ib( Matrix::ZEROES<Matrix>(block_rows, 1) );

  Matrix J(g->contactJacobian(joints, contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix JTD(matrixMultiply(J.transposed(), D));

  // Joint torque preloads
  Matrix tau_p(matrixMultiply(JTD, beta_p));
  Matrix extraTau(matrixAdd(tau_p, preload.negative()));

  // Joint angle/torque complementarity
  Matrix k4( Matrix::EYE(numJoints, numJoints) );
  k4.multiply(-TauMax);
  for (int i=0; i<k4.rows(); i++) k4.elem(i,i) -= extraTau.elem(i,0);
  Matrix k5( Matrix::EYE(numJoints, numJoints) );
  k5.multiply(QMax);

  InEq.copySubMatrixBlockIndices(0, P.var["beta"], JTD.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["beta"], JTD);
  InEq.copySubMatrixBlockIndices(1, P.var["y2"],   k4);
  InEq.copySubMatrixBlockIndices(2, P.var["q"],    Matrix::NEGEYE(numJoints, numJoints));
  InEq.copySubMatrixBlockIndices(3, P.var["q"],    Matrix::EYE(numJoints, numJoints));
  InEq.copySubMatrixBlockIndices(3, P.var["y2"],   k5);
  P.InEq_list.push_back(InEq);

  ib.copySubMatrixBlockIndices(0, 0, extraTau);
  ib.copySubMatrixBlockIndices(1, 0, extraTau.negative());
  Matrix k6(numJoints, 1);
  // For joints with zero preload, set the corresponding 
  // value in k6 to zero in order to prevent the joint from moving.
  // This also forces y=0 for this joint.
  for (int i=0; i<numJoints; i++) {
    if (preload.elem(i,0) == 0) {
      k6.elem(i,0) = 0;
      ib.elem(i,0) = TauMax;
      ib.elem(i+numJoints,0) = TauMax;
    }
    else {
      k6.elem(i,0) = QMax;
    }
  }
  ib.copySubMatrixBlockIndices(3, 0, k6);
  P.ib_list.push_back(ib);
}

void 
GraspSolver::tendonConstraint(GraspStruct &P, const Matrix &preload, const Matrix &beta_p)
{
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix JTD(matrixMultiply(J.transposed(), D));

  int numTendons = ((HumanHand*)(g->getHand()))->getNumTendons();
  std::vector<int> block_rows(4, numTendons);
  Matrix Eq( Matrix::ZEROES<Matrix>(numJoints, P.block_cols) );
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) );
  Matrix ib( Matrix::ZEROES<Matrix>(block_rows, 1) );

  // Populate the matrix holding the moment arms the tendons have at the joints.
  Matrix torqueRatios( Matrix::ZEROES<Matrix>(numJoints, numTendons) );
  for (int i=0; i<numTendons; i++) {
    Tendon *t = ((HumanHand*)(g->getHand()))->getTendon(i);
    for (int j=0; j < t->getNumJointPulleys(); j++) {
      bool added = false;
      JointPulley *pulley = t->getJointPulley(j);
      int chain = pulley->getChainNum();
      int joint = pulley->getJointNum();
      std::list<Joint*>::iterator it = joints.begin();
      for (int k=0; it!=joints.end(); it++, k++) {
        if ((*it)->getChainNum() == chain && (*it)->getNum() == joint) {
          torqueRatios.elem(k,i) = pulley->getRadius();
          added = true;
        }
      }
      if (!added) {
        DBGA("Could not find corresponding joint to place pulley at chain " << chain << " and joint " << joint);
        exit(0);
      }
    }
  }

  Matrix qRatios(torqueRatios.transposed());

  // Tendon force/ length complementarity
  Matrix k1( Matrix::EYE(numTendons, numTendons) );
  k1.multiply(-TauMax);
  Matrix k2( Matrix::EYE(numTendons, numTendons) );
  k2.multiply(QMax);
  Matrix k3(numTendons, 1);
  k3.setAllElements(QMax);

  InEq.copySubMatrixBlockIndices(0, P.var["f"], Matrix::NEGEYE(numTendons, numTendons));
  InEq.copySubMatrixBlockIndices(1, P.var["f"], Matrix::EYE(numTendons, numTendons));
  InEq.copySubMatrixBlockIndices(1, P.var["y2"], k1);
  ib.copySubMatrixBlockIndices(0, 0, preload.negative());
  ib.copySubMatrixBlockIndices(1, 0, preload);

  InEq.copySubMatrixBlockIndices(2, P.var["q"], qRatios.negative());
  InEq.copySubMatrixBlockIndices(3, P.var["q"], qRatios);
  InEq.copySubMatrixBlockIndices(3, P.var["y2"], k2);
  ib.copySubMatrixBlockIndices(3, 0, k3);
  P.InEq_list.push_back(InEq);
  P.ib_list.push_back(ib);

  // Contact/ Tendon force equality
  Matrix k4(matrixMultiply(JTD, beta_p));
  std::list<Joint*>::iterator it = joints.begin();
  for (int i=0; it!=joints.end(); it++, i++) {
    k4.elem(i,0) += (*it)->getSpringForce()/1.0e7;
  }
  Eq.copySubMatrixBlockIndices(0, P.var["beta"], JTD);
  Eq.copySubMatrixBlockIndices(0, P.var["f"], torqueRatios.negative());
  P.Eq_list.push_back(Eq);
  P.b_list.push_back(k4.negative());
}

void
GraspSolver::frictionConeConstraint(GraspStruct &P, const Matrix &beta_p) 
{
  // results of friction inequality arising from original contact forces
  Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
  Matrix d(matrixMultiply(F, beta_p));

  Matrix InEq( Matrix::ZEROES<Matrix>(numContacts, P.block_cols) );
  InEq.copySubMatrixBlockIndices(0, P.var["beta"], F);
  P.InEq_list.push_back(InEq);

  P.ib_list.push_back(d.negative());
}

void
GraspSolver::frictionConeEdgeConstraint(GraspStruct &P, const Matrix &beta_p)
{
    // results of friction inequality arising from original contact forces
  Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
  Matrix d(matrixMultiply(F, beta_p));

  // Joint angle/torque complementarity
  Matrix k7( Matrix::EYE(numContacts, numContacts) );
  k7.multiply(-2*BetaMax);
  for (int i=0; i<k7.rows(); i++) k7.elem(i,i) += d.elem(i,0);

  Matrix InEq( Matrix::ZEROES<Matrix>(numContacts, P.block_cols) );
  InEq.copySubMatrixBlockIndices(0, P.var["beta"], F.negative());
  InEq.copySubMatrixBlockIndices(0, P.var["y3"],   k7);
  P.InEq_list.push_back(InEq);

  P.ib_list.push_back(d);
}

void
GraspSolver::contactMovementConstraint(GraspStruct &P)
{
  std::list<Matrix> sigma_list;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    Matrix sigma(1, (*it)->numFrictionEdges+1);
    sigma.setAllElements(1.0);
    sigma.elem(0,0) = 0;
    sigma_list.push_back(sigma);
  }

  Matrix sigmaDiag( Matrix::BLOCKDIAG<Matrix>(sigma_list) );

  Matrix k8( Matrix::EYE(numContacts, numContacts) );
  k8.multiply(2*AlphaMax);
  Matrix k9(numContacts,1);
  k9.setAllElements(2*AlphaMax);

  Matrix InEq( Matrix::ZEROES<Matrix>(numContacts, P.block_cols) );

  InEq.copySubMatrixBlockIndices(0, P.var["alpha"], sigmaDiag);
  InEq.copySubMatrixBlockIndices(0, P.var["y3"],    k8);
  P.InEq_list.push_back(InEq);

  P.ib_list.push_back(k9);
}

void
GraspSolver::amplitudesSOS2Constraint(GraspStruct &P, const Matrix &beta_p)
{
  Matrix alphaSelector( Matrix::ZEROES<Matrix>(8,9) );
  alphaSelector.copySubMatrix(0, 1, Matrix::EYE(8,8));
  Matrix betaSelector( Matrix::ZEROES<Matrix>(8,9) );
  betaSelector.copySubMatrix(0, 1, Matrix::EYE(8,8));
  Matrix zSelector( Matrix::ZEROES<Matrix>(8,9) );
  zSelector.copySubMatrix(0, 0, Matrix::EYE(8,8));
  zSelector.elem(0,8) = 1.0;

  std::list<Matrix*> alpha_list;
  std::list<Matrix*> beta_list;
  std::list<Matrix*> z_list;

  int z_index = 0;
  for (int i=0; i<P.var["z"]; i++) z_index += P.block_cols[i];

  for (int i=0; i<numContacts; i++) {
    alpha_list.push_back(&alphaSelector);
    beta_list.push_back(&betaSelector);
    z_list.push_back(&zSelector);

    P.SOS_index.push_back(z_index + 9*i);
    P.SOS_len.push_back(9);
    P.SOS_type.push_back(2);
  }
  Matrix alphaDiag( Matrix::BLOCKDIAG<Matrix>(&alpha_list) );
  Matrix betaDiag( Matrix::BLOCKDIAG<Matrix>(&beta_list) );
  Matrix zDiag( Matrix::BLOCKDIAG<Matrix>(&z_list) );

  std::vector<int> block_rows(2, 8*numContacts);
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) );

  InEq.copySubMatrixBlockIndices(0, P.var["alpha"], alphaDiag);
  InEq.copySubMatrixBlockIndices(0, P.var["z"],     zDiag.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["beta"],  betaDiag);
  InEq.copySubMatrixBlockIndices(1, P.var["z"],     zDiag.negative());
  P.InEq_list.push_back(InEq);

  Matrix ib( Matrix::ZEROES<Matrix>(block_rows, 1) );
  ib.copySubMatrixBlockIndices(1, 0, matrixMultiply(betaDiag, beta_p.negative()));
  P.ib_list.push_back(ib);
}

void
GraspSolver::virtualLimitLBConstraint(GraspStruct &P, bool iterative /*= false*/)
{
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix JTD(matrixMultiply(J.transposed(), D));

  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix NJ(matrixMultiply(N, J));

  int numBetas = P.block_cols[P.var["beta"]];
  int numQs = P.block_cols[P.var["q"]];
  int numAlphas = 1;
  if (!iterative) numAlphas = P.block_cols[P.var["alpha"]];
  std::vector<int> block_rows;
  block_rows.push_back(numBetas);
  block_rows.push_back(numJoints);
  block_rows.push_back(numQs);
  block_rows.push_back(numContacts);
  if (!iterative) block_rows.push_back(numAlphas);
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) );

  Matrix S_beta(numBetas, 1);
  S_beta.setAllElements(-BetaMax);
  Matrix S_tau(numJoints, 1);
  S_tau.setAllElements(-TauMax);
  Matrix S_q(numQs, 1);
  S_q.setAllElements(-QMax);
  Matrix S_alpha(numAlphas, 1);
  S_alpha.setAllElements(-AlphaMax);
  Matrix S_k(numContacts, 1);
  S_k.setAllElements(-KMax);

  InEq.copySubMatrixBlockIndices(0, P.var["beta"], Matrix::EYE(numBetas, numBetas));
  InEq.copySubMatrixBlockIndices(0, P.var["v"], S_beta);
  InEq.copySubMatrixBlockIndices(1, P.var["beta"], JTD);
  InEq.copySubMatrixBlockIndices(1, P.var["v"], S_tau);
  InEq.copySubMatrixBlockIndices(2, P.var["q"], Matrix::EYE(numQs, numQs));
  InEq.copySubMatrixBlockIndices(2, P.var["v"], S_q);
  InEq.copySubMatrixBlockIndices(3, P.var["x"], K.negative());
  InEq.copySubMatrixBlockIndices(3, P.var["q"], NJ);
  InEq.copySubMatrixBlockIndices(3, P.var["v"], S_k);
  if (!iterative) {
    InEq.copySubMatrixBlockIndices(4, P.var["alpha"], Matrix::EYE(numAlphas, numAlphas));
    InEq.copySubMatrixBlockIndices(4, P.var["v"], S_alpha);
  }
  P.InEq_list.push_back(InEq);

  P.ib_list.push_back(Matrix::ZEROES<Matrix>(block_rows, 1));
}

//  ---------------------  Quadratic Inequality Constraints  -------------------------  //

void
GraspSolver::objectMotionLimit(GraspStruct &P) 
{
  Matrix Q( Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols) );
  Q.copySubMatrixBlockIndices(P.var["x"], P.var["x"], Matrix::EYE(6,6));
  P.QInEq.push_back(Q);  

  Matrix q( Matrix::ZEROES<Matrix>(1, P.block_cols) );
  P.iq.push_back(q);

  Matrix I(1,1);
  I.setAllElements(kMaxMotion);
  P.qib.push_back(I);
}

//  ------------------------  Variable Bounds and Types  -----------------------------  //

void 
GraspSolver::variableBoundsAndTypes(GraspStruct &P, const Matrix &beta_p)
{
  std::list<std::string>::iterator it;
  for (it=P.varNames.begin(); it!=P.varNames.end(); it++) {
    std::string key = *it;
    int i = P.var[key];
    int size = P.block_cols[i];

    if (!key.compare("alpha")) {
      Matrix alpha_lb(size, 1);
      alpha_lb.setAllElements(0.0);
      for (int j=0; j<numContacts; j++)
        alpha_lb.elem(9*j,0) = -AlphaMax;
      P.lb_list.push_back(alpha_lb);

      Matrix alpha_ub(size, 1);
      alpha_ub.setAllElements(AlphaMax);
      P.ub_list.push_back(alpha_ub);

      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("beta")) {
      P.lb_list.push_back(beta_p.negative());
      Matrix beta_ub(size, 1);
      beta_ub.setAllElements(BetaMax);
      P.ub_list.push_back(beta_ub);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("x")) { 
      P.lb_list.push_back(Matrix::MIN_VECTOR(size));
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("q")) {
      Matrix qbound(size, 1);
      qbound.setAllElements(QMax);
      P.lb_list.push_back(qbound.negative());
      P.ub_list.push_back(qbound);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("f")) {
      Matrix f_lb(Matrix::ZEROES<Matrix>(size, 1));
      Matrix f_ub(Matrix::MAX_VECTOR(size));
      P.lb_list.push_back(f_lb);
      P.ub_list.push_back(f_ub);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("y1")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      Matrix y1_ub(size, 1);
      y1_ub.setAllElements(1);
      P.ub_list.push_back(y1_ub);
      Matrix t(size, 1);
      t.setAllElements(1);
      P.types.push_back(t);
    }

    else if (!key.compare("y2")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      Matrix y2_ub(size, 1);
      y2_ub.setAllElements(1);
      P.ub_list.push_back(y2_ub);
      Matrix t(size, 1);
      t.setAllElements(1);
      P.types.push_back(t);
    }

    else if (!key.compare("y3")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      Matrix y3_ub(size, 1);
      y3_ub.setAllElements(1);
      P.ub_list.push_back(y3_ub);
      Matrix t(size, 1);
      t.setAllElements(1);
      P.types.push_back(t);
    }

    else if (!key.compare("v")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      Matrix v_ub(size,1);
      v_ub.setAllElements(1);
      P.ub_list.push_back(v_ub);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("z")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("r")) {
      P.lb_list.push_back(Matrix::MIN_VECTOR(size));
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("s")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      Matrix s_ub(size, 1);
      s_ub.setAllElements(1.0);
      P.ub_list.push_back(s_ub);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else {
      DBGA("variableBoundsAndTypes: Requested unknown variable"); 
      assert(0);
    }
  }
}

//  -----------  Routines constructing non-iterative problems  ----------------  //

//! The non-iterative formulation for direct drive hands
/*!
  Variables: contact motion amplitude (alpha)
             contact force amplitude (beta)
             object motion (x)
             joint motion (q)
             contact breaking decision variable (y1)
             joint backdriving decision variable (y2)
             contact sliding decision variable (y3)
             SOS2 variables for contact forces and motions (z)
             relative closeness of variables to virtual limits (v)
*/
void
GraspSolver::nonIterativeFormulation(GraspStruct &P, const Matrix &preload, 
  const Matrix &wrench /*=ZEROES*/, const Matrix &beta /*=Matrix(0,0)*/, 
  bool rigid /*=false*/) 
{
  // unknowns
  P.var["alpha"] = 0; P.block_cols.push_back(9*numContacts); P.varNames.push_back("alpha");
  P.var["beta"]  = 1; P.block_cols.push_back(9*numContacts); P.varNames.push_back("beta");
  P.var["x"]     = 2; P.block_cols.push_back(6);             P.varNames.push_back("x");
  P.var["q"]     = 3; P.block_cols.push_back(numJoints);     P.varNames.push_back("q");
  P.var["y1"]    = 4; P.block_cols.push_back(numContacts);   P.varNames.push_back("y1");
  P.var["y2"]    = 5; P.block_cols.push_back(numJoints);     P.varNames.push_back("y2");
  P.var["y3"]    = 6; P.block_cols.push_back(numContacts);   P.varNames.push_back("y3");
  P.var["z"]     = 7; P.block_cols.push_back(9*numContacts); P.varNames.push_back("z");
  P.var["v"]     = 8; P.block_cols.push_back(1);             P.varNames.push_back("v");

  // Set virtual limits for MIP representation of linear complementarities
  setVirtualLimits(preload, wrench);

  // Preload contact forces (if present)
  Matrix beta_p( Matrix::ZEROES<Matrix>(9*numContacts, 1) );
  if (beta.rows()) beta_p.copyMatrix(beta);

  // Objective 
  //springDeformationObjective(P);
  virtualLimitsObjective(P);

  // equality constraints
  objectWrenchConstraint(P, wrench);
  movementAmplitudesConstraint(P);

  // inequality constraints
  virtualSpringConstraint(P, beta_p);
  if (rigid) rigidJointsConstraint(P);
  else nonBackdrivableJointConstraint(P, preload, beta_p);
  frictionConeConstraint(P, beta_p);
  frictionConeEdgeConstraint(P, beta_p);
  contactMovementConstraint(P);
  amplitudesSOS2Constraint(P, beta_p);
  virtualLimitLBConstraint(P);

  // quadratic inequality constraints
  //objectMotionLimit(P);

  // solution bounds and types
  variableBoundsAndTypes(P, beta_p);
}

void
GraspSolver::nonIterativeTendonFormulation(GraspStruct &P, const Matrix &preload, 
  const Matrix &wrench /*=ZEROES*/, const Matrix &beta /*=Matrix(0,0)*/)
{
  int numTendons = preload.rows();

  // unknowns
  P.var["alpha"] = 0; P.block_cols.push_back(9*numContacts); P.varNames.push_back("alpha");
  P.var["beta"]  = 1; P.block_cols.push_back(9*numContacts); P.varNames.push_back("beta");
  P.var["x"]     = 2; P.block_cols.push_back(6);             P.varNames.push_back("x");
  P.var["q"]     = 3; P.block_cols.push_back(numJoints);     P.varNames.push_back("q");
  P.var["f"]     = 4; P.block_cols.push_back(numTendons);    P.varNames.push_back("f");
  P.var["y1"]    = 5; P.block_cols.push_back(numContacts);   P.varNames.push_back("y1");
  P.var["y2"]    = 6; P.block_cols.push_back(numTendons);    P.varNames.push_back("y2");
  P.var["y3"]    = 7; P.block_cols.push_back(numContacts);   P.varNames.push_back("y3");
  P.var["z"]     = 8; P.block_cols.push_back(9*numContacts); P.varNames.push_back("z");

  // Set virtual limits for MIP representation of linear complementarities
  setVirtualLimits(preload, wrench);

  // Preload contact forces (if present)
  Matrix beta_p( Matrix::ZEROES<Matrix>(9*numContacts,1) );
  if (beta.rows()) beta_p.copyMatrix(beta);

  // Objective 
  springDeformationObjective(P);

  // equality constraints
  objectWrenchConstraint(P, wrench);
  movementAmplitudesConstraint(P);

  // inequality constraints
  virtualSpringConstraint(P, beta_p);
  tendonConstraint(P, preload, beta_p);
  frictionConeConstraint(P, beta_p);
  frictionConeEdgeConstraint(P, beta_p);
  contactMovementConstraint(P);
  amplitudesSOS2Constraint(P, beta_p);

  // quadratic inequality constraints
  objectMotionLimit(P);

  // solution bounds and types
  variableBoundsAndTypes(P, beta_p);
}

//  --------------  Routines constructing iterative problems  -------------------  //

void
GraspSolver::iterativeFormulation(GraspStruct &P, const Matrix &preload, const Matrix &startingX, 
  const Matrix &movement_directions, const Matrix &wrench, const Matrix &beta, bool rigid) 
{
  // unknowns
  P.var["beta"]  = 0; P.block_cols.push_back(9*numContacts); P.varNames.push_back("beta");
  P.var["x"]     = 1; P.block_cols.push_back(6);             P.varNames.push_back("x");
  P.var["q"]     = 2; P.block_cols.push_back(numJoints);     P.varNames.push_back("q");
  P.var["y1"]    = 3; P.block_cols.push_back(numContacts);   P.varNames.push_back("y1");
  P.var["y2"]    = 4; P.block_cols.push_back(numJoints);     P.varNames.push_back("y2");
  P.var["r"]     = 5; P.block_cols.push_back(6);             P.varNames.push_back("r");
  P.var["s"]     = 6; P.block_cols.push_back(movement_directions.cols());
    P.varNames.push_back("s");

  // Set virtual limits for MIP representation of linear complementarities
  setVirtualLimits(preload, wrench);

  // Preload contact forces (if present)
  Matrix beta_p( Matrix::ZEROES<Matrix>(9*numContacts,1) );
  if (beta.rows()) beta_p.copyMatrix(beta);

  // Objective
  resultantWrenchObjective(P);

  // equality constraints
  resultantWrenchConstraint(P, wrench);
  objectMovementConstraint(P, startingX, movement_directions);

  // inequality constraints
  virtualSpringConstraint(P, beta_p);
  if (rigid) rigidJointsConstraint(P);
  else nonBackdrivableJointConstraint(P, preload, beta_p);
  frictionConeConstraint(P, beta_p);

  // solution bounds and types
  variableBoundsAndTypes(P, beta_p);
}

void
GraspSolver::iterativeTendonFormulation(GraspStruct &P, const Matrix &preload, const Matrix &startingX,
  const Matrix &movement_directions, const Matrix &wrench, const Matrix &beta)
{
  int numTendons = preload.rows();

  // unknowns
  P.var["beta"]  = 0; P.block_cols.push_back(9*numContacts); P.varNames.push_back("beta");
  P.var["x"]     = 1; P.block_cols.push_back(6);             P.varNames.push_back("x");
  P.var["q"]     = 2; P.block_cols.push_back(numJoints);     P.varNames.push_back("q");
  P.var["f"]     = 3; P.block_cols.push_back(numTendons);    P.varNames.push_back("f");
  P.var["y1"]    = 4; P.block_cols.push_back(numContacts);   P.varNames.push_back("y1");
  P.var["y2"]    = 5; P.block_cols.push_back(numTendons);    P.varNames.push_back("y2");
  P.var["r"]     = 6; P.block_cols.push_back(6);             P.varNames.push_back("r");
  P.var["s"]     = 7; P.block_cols.push_back(movement_directions.cols());
    P.varNames.push_back("s");

  // Set virtual limits for MIP representation of linear complementarities
  setVirtualLimits(preload, wrench);

  // Preload contact forces (if present)
  Matrix beta_p( Matrix::ZEROES<Matrix>(9*numContacts,1) );
  if (beta.rows()) beta_p.copyMatrix(beta);

  // Objective
  resultantWrenchObjective(P);

  // equality constraints
  resultantWrenchConstraint(P, wrench);
  objectMovementConstraint(P, startingX, movement_directions);

  // inequality constraints
  virtualSpringConstraint(P, beta_p);
  tendonConstraint(P, preload, beta_p);
  frictionConeConstraint(P, beta_p);

  // solution bounds and types
  variableBoundsAndTypes(P, beta_p);
}

int
GraspSolver::iterativeSolver(SolutionStruct &S, bool cone_movement,
  std::tr1::function<void(GraspStruct&,const Matrix&,const Matrix&)> formulation)
{
  int result;
  double previous_best = 0;
  int iterations = 0;
  Matrix startingX( Matrix::ZEROES<Matrix>(6,1) );
  Matrix movement_directions( Matrix::ZEROES<Matrix>(6,1) );
  while (1) {
    iterations++;
    
    GraspStruct P;
    formulation(P, startingX, movement_directions);
    result = solveProblem(P, S);

    if (result) {
      DBGA(" Problem failed");
      result = 1;
      break;
    }

    Matrix x(S.sol->getSubMatrixBlockIndices(S.var["x"], 0));
    Matrix r(S.sol->getSubMatrixBlockIndices(S.var["r"], 0));

    // Check for convergence to nonzero resultant
    if (previous_best) {
      double improvement = (previous_best - r.fnorm()) / previous_best;
      DBGA("    Improvement: " << improvement);
      if (improvement < 1.0e-2) {
        if (improvement < -1.0e-3) {
          DBGA("NEGATIVE IMPROVEMENT OF " << improvement << " AT RESIDUAL OF " << r.fnorm());
          //exit(0);
        }
        DBGA("    Failed to reduce resultant wrench");
        result = 1;
        break;
      }
    }
    previous_best = r.fnorm();

    // Check for equilibrium
    if (r.fnorm() < 1.0e-5) {
      DBGA("  Equilibrium found");
      result = 0;
      break;
    }
    
    // Break when maximum number of iterations reached
    if (iterations >= 100) {
      DBGA("  Max iterations reached at unbalanced norm " << r.fnorm());
      result = 1;
      break;
    }

    // Set up parameters for next iteration
    if (cone_movement) {
      Matrix old_directions(movement_directions);
      movement_directions.resize(movement_directions.rows(), movement_directions.cols()+1);
      movement_directions.copySubMatrix(0, 0, old_directions);
      movement_directions.copySubMatrix(0, old_directions.cols(), r.negative().normalized());
    } else {
      startingX.copyMatrix(x);
      movement_directions.copyMatrix(r.negative().normalized());
    }
  }
  return result;
}

//  -----------------------  Grasp Analysis Routines  ----------------------------  //

int
GraspSolver::checkWrenchNonIterative(Matrix &preload, const Matrix &wrench, 
  bool single_step, bool tendon /*=false*/, bool rigid /*= false*/)
{
  if (removeJointsBeyondContact(joints, contacts, preload))
    if (tendon) {
      DBGA("Removed joint beyond contact. For underactuated hands this is not allowed");
      exit(0);
    }
  numJoints = joints.size();

  int result;
  GraspStruct P;
  SolutionStruct S;
  Matrix beta_p( Matrix::ZEROES<Matrix>(9*numContacts, 1) );
  if (single_step) {
    if (tendon) nonIterativeTendonFormulation(P, preload, wrench);
    else nonIterativeFormulation(P, preload, wrench);
    result = solveProblem(P, S);
  } else {
    GraspStruct P_p;
    SolutionStruct S_p;
    if (tendon) nonIterativeTendonFormulation(P_p, preload);
    else nonIterativeFormulation(P_p, preload);
    int preload_result = solveProblem(P_p, S_p);
    if (preload_result) {
      DBGA("Preload creation failed");
      return preload_result;
    }
    beta_p.copyMatrix(S_p.sol->getSubMatrixBlockIndices(S_p.var["beta"], 0));
    if (tendon) nonIterativeTendonFormulation(P, preload, wrench, beta_p);
    else nonIterativeFormulation(P, preload, wrench, beta_p, rigid);
    result = solveProblem(P, S);  
  }
  if (result) return result;
  Matrix beta_t(matrixAdd(beta_p, S.sol->getSubMatrixBlockIndices(S.var["beta"], 0)));
  drawContactWrenches(beta_t);
  drawObjectMovement(S);
  return result;
}
int 
GraspSolver::checkWrenchNonIterative(Matrix &preload, const Matrix &wrench, 
  const Matrix &beta_p, bool tendon, bool rigid) 
{
  GraspStruct P;
  SolutionStruct S;
  if (tendon) nonIterativeTendonFormulation(P, preload, wrench, beta_p);
  else nonIterativeFormulation(P, preload, wrench, beta_p, rigid);
  return solveProblem(P, S); 
}

int
GraspSolver::checkWrenchIterative(Matrix &preload, const Matrix &wrench,
  bool single_step, bool cone_movement, bool tendon /*=false*/, bool rigid /*= false*/)
{
  if (removeJointsBeyondContact(joints, contacts, preload))
    if (tendon) {
      DBGA("Removed joint beyond contact. For underactuated hands this is not allowed");
      exit(0);
    }
  numJoints = joints.size();

  int result;
  SolutionStruct S;
  std::tr1::function<void(GraspStruct&,const Matrix&,const Matrix&)> formulation;
  if (single_step) {
    if (tendon)
      formulation = std::tr1::bind(
        &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, Matrix(0,0));
    else {
      formulation = std::tr1::bind(
        &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload,  
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, Matrix(0,0), false);
    }
    result = iterativeSolver(S, cone_movement, formulation);
  } else {
    if (tendon)
      formulation = std::tr1::bind(
        &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload,
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, Matrix::ZEROES<Matrix>(6,1), Matrix(0,0));
    else
      formulation = std::tr1::bind(
        &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload,  
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, Matrix::ZEROES<Matrix>(6,1), Matrix(0,0), false);
    SolutionStruct S_p;
    int preload_result = iterativeSolver(S_p, cone_movement, formulation);
    if (preload_result) {
      DBGA("Preload creation failed");
      return preload_result;
    } 
    Matrix beta_p(S_p.sol->getSubMatrixBlockIndices(S_p.var["beta"], 0));
    if (tendon)
      formulation = std::tr1::bind(
        &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p);
    else 
      formulation = std::tr1::bind(
        &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p, rigid);
    result = iterativeSolver(S, cone_movement, formulation);
    Matrix beta(S.sol->getSubMatrixBlockIndices(S.var["beta"], 0));
    matrixAdd(beta, beta_p, beta);
    S.sol->copySubMatrixBlockIndices(S.var["beta"], 0, beta);
  }
  if (result) return result;
  Matrix beta(S.sol->getSubMatrixBlockIndices(S.var["beta"], 0));
  drawContactWrenches(beta);
  drawObjectMovement(S);
  return result;
}
int
GraspSolver::checkWrenchIterative(Matrix &preload, const Matrix &wrench,
  const Matrix &beta_p, bool cone_movement, bool tendon, bool rigid)
{
  SolutionStruct S;
  std::tr1::function<void(GraspStruct&,const Matrix&,const Matrix&)> formulation;
  if (tendon)
    formulation = std::tr1::bind(
      &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload,
      std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p);
  else
    formulation = std::tr1::bind(
      &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload, 
      std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p, rigid);
  return iterativeSolver(S, cone_movement, formulation);
}

double
GraspSolver::findMaximumWrenchNonIterative(Matrix &preload, const Matrix &direction,
  bool single_step, bool tendon /*=false*/, bool rigid /*= false*/)
{
  if (removeJointsBeyondContact(joints, contacts, preload))
    if (tendon) {
      DBGA("Removed joint beyond contact. For underactuated hands this is not allowed");
      exit(0);
    }
  numJoints = joints.size();

  std::tr1::function<int(const Matrix&)> func;
  if (single_step) {
    func = std::tr1::bind((int(GraspSolver::*)(Matrix&,const Matrix&,bool,bool,bool))&GraspSolver::checkWrenchNonIterative, 
      this, preload, std::tr1::placeholders::_1, single_step, tendon, false);
  } else {
    GraspStruct P_p;
    SolutionStruct S_p;
    if (tendon) nonIterativeTendonFormulation(P_p, preload);
    else nonIterativeFormulation(P_p, preload);
    int preload_result = solveProblem(P_p, S_p);
    if (preload_result) {
      DBGA("Preload creation failed");
      return preload_result;
    }
    Matrix beta_p(S_p.sol->getSubMatrixBlockIndices(S_p.var["beta"], 0));
    func = std::tr1::bind((int(GraspSolver::*)(Matrix&,const Matrix&,const Matrix&,bool,bool))&GraspSolver::checkWrenchNonIterative, 
      this, preload, std::tr1::placeholders::_1, beta_p, tendon, rigid); 
  }

  double max = Matrix::binarySearch(func, direction, BIN_ULIMIT);
  Matrix wrench(direction);
  wrench.multiply(max);
  if (checkWrenchNonIterative(preload, wrench, single_step, tendon, rigid)) {
    DBGA("Failed maximum resistible wrench returned by the binary search!");
    assert(0);
  }
  return max;
}

double
GraspSolver::findMaximumWrenchIterative(Matrix &preload, const Matrix &direction,
  bool single_step, bool cone_movement, bool tendon /*=false*/, bool rigid /*= false*/)
{
  if (removeJointsBeyondContact(joints, contacts, preload))
    if (tendon) {
      DBGA("Removed joint beyond contact. For underactuated hands this is not allowed");
      exit(0);
    }
  numJoints = joints.size();
  
  std::tr1::function<int(const Matrix&)> func;
  if (single_step) {
    func = std::tr1::bind((int(GraspSolver::*)(Matrix&,const Matrix&,bool,bool,bool,bool))&GraspSolver::checkWrenchIterative, 
      this, preload, std::tr1::placeholders::_1, single_step, cone_movement, tendon, false); 
  } else {
    SolutionStruct S_p;
    std::tr1::function<void(GraspStruct&,const Matrix&,const Matrix&)> formulation;
    if (tendon)
      formulation = std::tr1::bind(
        &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, Matrix::ZEROES<Matrix>(6,1), Matrix(0,0));
    else
      formulation = std::tr1::bind(
        &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload,  
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, Matrix::ZEROES<Matrix>(6,1), Matrix(0,0), false);
    int preload_result = iterativeSolver(S_p, cone_movement, formulation);
    if (preload_result) {
      DBGA("Preload creation failed");
      return preload_result;
    } 
    Matrix beta_p(S_p.sol->getSubMatrixBlockIndices(S_p.var["beta"], 0));
    func = std::tr1::bind((int(GraspSolver::*)(Matrix&,const Matrix&,const Matrix&,bool,bool,bool))&GraspSolver::checkWrenchIterative, 
      this, preload, std::tr1::placeholders::_1, beta_p, cone_movement, tendon, rigid); 
  }

  double max = Matrix::binarySearch(func, direction, BIN_ULIMIT);
  Matrix wrench(direction);
  wrench.multiply(max);
  if (max) {
    if (checkWrenchIterative(preload, wrench, single_step, cone_movement, tendon, rigid)) {
      DBGA("Failed maximum resistable wrench returned by the binary search!");
      assert(0);
    }
  } else {
    DBGA("Found no resistible wrench");
  }
  return max;
}

int 
GraspSolver::create2DMap(Matrix &preload, bool single_step, bool tendon, bool iterative, 
  bool cone_movement, bool rigid)
{
  std::ofstream output;
  output.open("./log/2Dmap.txt");
  output << "d1, d2\n";

  // Directions defining the plane in which to create the 2D map. If there
  // is a lot of detail in one direction specifically and you want a higher
  // density of points in that direction, you can increase the magnitude of
  // the vector defining that direction
  Matrix direction1(Matrix::ZEROES<Matrix>(6,1));
  Matrix direction2(Matrix::ZEROES<Matrix>(6,1));
  direction1.elem(0,0) = 1;
  direction2.elem(2,0) = 1;

  int directionSteps = 500;
  for (int i=0; i<directionSteps; i++) {

    Matrix xcomponent(direction1);
    Matrix ycomponent(direction2);
    xcomponent.multiply(sin(2*M_PI*i/directionSteps));
    ycomponent.multiply(cos(2*M_PI*i/directionSteps));

    Matrix direction(matrixAdd(xcomponent, ycomponent));
    direction.multiply(1.0 / direction.fnorm());
    double max;
    if (iterative) max = findMaximumWrenchIterative(preload, direction, single_step, cone_movement, tendon, rigid);  
    else max = findMaximumWrenchNonIterative(preload, direction, single_step, tendon, rigid);

    Matrix w_max(2,1);
    w_max.elem(0,0) = sin(2*M_PI*i/directionSteps) * direction1.fnorm();
    w_max.elem(1,0) = cos(2*M_PI*i/directionSteps) * direction2.fnorm();
    w_max.multiply(max/w_max.fnorm());
    output << -w_max.elem(0,0) << ", " << -w_max.elem(1,0) << "\n";

    DBGA("---------------- Iteration " << i << "/" << directionSteps);
  }
  output.close();
  return 1;
}


int
GraspSolver::solveProblem(GraspStruct &P, SolutionStruct &S)
{
  // TODO: add tests to make sure all matrices are of appropriate size
  Matrix Eq   ( Matrix::BLOCKCOLUMN<Matrix>(P.Eq_list) );
  Matrix b    ( Matrix::BLOCKCOLUMN<Matrix>(P.b_list) );
  Matrix InEq ( Matrix::BLOCKCOLUMN<Matrix>(P.InEq_list) );
  Matrix ib   ( Matrix::BLOCKCOLUMN<Matrix>(P.ib_list) );
  Matrix lb   ( Matrix::BLOCKCOLUMN<Matrix>(P.lb_list) );
  Matrix ub   ( Matrix::BLOCKCOLUMN<Matrix>(P.ub_list) );
  Matrix types( Matrix::BLOCKCOLUMN<Matrix>(P.types) );

  double objVal = 0;
  Matrix sol(P.block_cols, 1);
  int result = MIPSolver(*(P.Q), *(P.cj), Eq, b, InEq, ib, P.QInEq, P.iq, P.qib, P.SOS_index, 
                P.SOS_len, P.SOS_type, lb, ub, sol, types, &objVal);
  delete S.sol; S.sol = NULL;
  S.block_cols = P.block_cols;
  S.var = P.var;
  S.sol = new Matrix(sol);

  if (result) return result;

  // Check if solution is close to virtual bounds. If so issue a warning
  if (S.var.find("beta") != S.var.end()) {
    Matrix beta(S.sol->getSubMatrixBlockIndices(S.var["beta"], 0));
    if (beta.max() > 0.25 * BetaMax) {
      DBGA("beta (" << beta.max() 
        << ") is larger than 1/4 of the virtual limit (" << BetaMax 
        << "). Consider increasing BetaMax");      
      exit(0);
    }
    Matrix J(g->contactJacobian(joints, contacts));
    Matrix D(Contact::frictionForceBlockMatrix(contacts));
    Matrix JTD(matrixMultiply(J.transposed(), D));
    Matrix tau(matrixMultiply(JTD, beta));
    if (tau.max() > 0.25 * TauMax) {
      DBGA("tau (" << tau.max() 
        << ") is larger than 1/4 of the virtual limit (" << TauMax 
        << "). Consider increasing TauMax");
      exit(0);
    }
  }
  if (S.var.find("q") != S.var.end()) {
    Matrix q(S.sol->getSubMatrixBlockIndices(S.var["q"], 0));
    if (q.max() > 0.25 * QMax && q.max() != QMax) {
      DBGA("q (" << q.max() 
        << ") is larger than 1/4 of the virtual limit (" << QMax 
        << "). Consider increasing QMax");      
      exit(0);
    }
  }
  if (S.var.find("f") != S.var.end()) {
    Matrix f(S.sol->getSubMatrixBlockIndices(S.var["f"], 0));
    if (f.max() > 0.25 * TauMax) {
      DBGA("f (" << f.max() 
        << ") is larger than 1/4 of the virtual limit (" << TauMax 
        << "). Consider increasing TauMax");      
      exit(0);
    }
  }
  if (S.var.find("alpha") != S.var.end()) {
    Matrix alpha(S.sol->getSubMatrixBlockIndices(S.var["alpha"], 0));
    // In special cases where object movement does not compress any virtual 
    // springs, the solver has the annoying habit of moving the object as far 
    // as it may without breaking the constraint on alpha. In this case the 
    // greatest alpha will be exactly equal to the virtual limit. Hence, an
    // additional test for this case had to be added to the below check. 
    if (alpha.max() > 0.25 * AlphaMax && alpha.max() != AlphaMax) {
      DBGA("alpha (" << alpha.max() 
        << ") is larger than 1/4 of the virtual limit (" << AlphaMax 
        << "). Consider increasing AlphaMax");
      exit(0);
    }
  }
  if ((S.var.find("q") != S.var.end()) && (S.var.find("x") != S.var.end())) {
    Matrix J(g->contactJacobian(joints, contacts));
    Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
    Matrix N(normalDisplacementSelectionMatrix(numContacts));
    Matrix RT(Grasp::graspMapMatrix(R).transposed());
    Matrix K(matrixMultiply(N, RT));
    Matrix NJ(matrixMultiply(N, J));

    Matrix q(S.sol->getSubMatrixBlockIndices(S.var["q"], 0));
    Matrix x(S.sol->getSubMatrixBlockIndices(S.var["x"], 0));
    Matrix k(matrixAdd(matrixMultiply(NJ, q), matrixMultiply(K, x).negative()));
    if (k.max() > 0.25 * KMax  && (KMax - k.max()) > 1) {
      DBGA("k (" << k.max() 
        << ") is larger than 1/4 of the virtual limit (" << KMax 
        << "). Consider increasing KMax");      
      exit(0);
    }
  }
  return result;
}

void
GraspSolver::logSolution(SolutionStruct &S)
{
  std::ofstream output;
  std::stringstream filename;
  filename << "log/solution.txt";
  output.open(filename.str().c_str());
  std::map<std::string, int>::iterator it;
  for (it=S.var.begin(); it!=S.var.end(); it++) {
    output << it->first << std::endl;
    output << S.sol->getSubMatrixBlockIndices(it->second, 0) << std::endl;
  }
  Matrix beta(S.sol->getSubMatrixBlockIndices(S.var["beta"], 0));
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix JTD(matrixMultiply(J.transposed(), D));
  output << "tau\n" << matrixMultiply(JTD, beta);
  output.close();
}

void
GraspSolver::setVirtualLimits(const Matrix &preload, const Matrix &wrench /*=ZEROES(6,1)*/)
{
  double scale = wrench.fnorm();
  scale = std::max(scale, preload.absMax());
  BetaMax = kBetaMaxBase*scale;
  TauMax = kTauMaxBase*scale;
  AlphaMax = kAlphaMaxBase*scale;
  QMax = kQMaxBase*scale;
  KMax = kKMaxBase*scale;
}

bool 
GraspSolver::removeJointsBeyondContact(std::list<Joint*> &joints, 
  const std::list<Contact*> &contacts, Matrix &preload)
{
  bool removed = false;
  Matrix J_complete(g->contactJacobian(joints, contacts));
  std::list<Joint*>::iterator it=joints.begin();
  for (int i=0; it!=joints.end(); i++) {
    Matrix JCol(J_complete.getSubMatrix(0,i,J_complete.rows(),1));
    if (JCol.absMax() == 0) {
      it = joints.erase(it);
      Matrix tmp(preload);
      tmp.copySubBlock(i, 0, preload.rows()-i-1, 1, preload, i+1, 0);
      preload.resize(preload.rows()-1, 1);
      preload.copySubBlock(0, 0, preload.rows(), 1, tmp, 0, 0);
      removed = true;
    }
    else it++;
  }
  return removed;
}

bool
GraspSolver::removePassiveJoints(std::list<Joint*> &joints, 
  Matrix &preload)
{
  bool removed = false;
  Matrix preload_complete(preload);
  std::list<Joint*>::iterator it=joints.begin();
  for (int i=0; it!=joints.end(); i++) {
    if (preload_complete.elem(i,0) == 0) {
      it = joints.erase(it);
      Matrix tmp(preload);
      tmp.copySubBlock(i, 0, preload.rows()-i-1, 1, preload, i+1, 0);
      preload.resize(preload.rows()-1, 1);
      preload.copySubBlock(0, 0, preload.rows(), 1, tmp, 0, 0);
      removed = true;
    }
    else it++;
  }
  return removed;
}

void
GraspSolver::drawContactWrenches(const Matrix &beta)
{
  Matrix D(Contact::frictionForceBlockMatrix(contacts));  
  Matrix cWrenches(matrixMultiply(D, beta));
  double scale = 2.0e7 / cWrenches.getSubMatrix(0,0,6,1).fnorm();
  for (int i=1; i<cWrenches.rows()/6; i++) {
    scale = std::min(scale, 2.0e7 / cWrenches.getSubMatrix(6*i,0,6,1).fnorm());
  }
  cWrenches.multiply(scale);
  g->displayContactWrenches(&contacts, cWrenches);
  graspitCore->getIVmgr()->drawDynamicForces();
}

void
GraspSolver::drawObjectMovement(SolutionStruct &S) 
{
  Matrix x( S.sol->getSubMatrixBlockIndices(S.var["x"], 0) );
  double minWrench[6];
  for (int i=0; i<6; i++) {
    minWrench[i] = -x.elem(i,0);
  }
  g->setMinWrench(minWrench);
  graspitCore->getIVmgr()->drawWorstCaseWrenches();
}

//  --------------------------  Useful Matrices  -------------------------------  //

//  Selection Matrix for normal forces
Matrix
normalForceSelectionMatrix(std::list<Contact*> &contacts)
{
  std::list<Contact*>::iterator it;
  it=contacts.begin();
  Matrix S(contacts.size(), contacts.size() * ((*it)->numFrictionEdges + 1));
  S.setAllElements(0.0);
  for (int i=0, j=0; it!=contacts.end(); i++, it++) {
    S.elem(i, j) = 1.0;
    j += ((*it)->numFrictionEdges + 1);
  }
  return S;
}

//  Selection matrix for normal contact displacements
Matrix
normalDisplacementSelectionMatrix(int numContacts)
{
  Matrix N(numContacts, 6 * numContacts);
  N.setAllElements(0.0);
  for (int i=0; i<numContacts; i++)
    N.elem(i, 6*i+2) = 1.0;
  return N;
}

//  Selection Matrix for tangential contact displacements
Matrix
tangentialDisplacementSelectionMatrix(int numContacts)
{
  Matrix M(2*numContacts, 6 * numContacts);
  M.setAllElements(0.0);
  for (int i=0; i<numContacts; i++) {
    M.elem(2*i, 6*i) = 1.0;
    M.elem(2*i+1, 6*i+1) = 1.0;
  }
  return M;
}
