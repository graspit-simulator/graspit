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
#include <iomanip>

#define PROF_ENABLED
#include "graspit/profiling.h"
PROF_DECLARE(TIMER);

const double GraspSolver::kSpringStiffness = 1.0;

const double GraspSolver::kBetaMaxBase = 200;
const double GraspSolver::kTauMaxBase = 2500;
const double GraspSolver::kAlphaMaxBase = 100000;
const double GraspSolver::kQMaxBase = 50;
const double GraspSolver::kKMaxBase = 2000;

const double GraspSolver::kMaxMotion = 5.0;

const double GraspSolver::kVirtualLimitTolerance = 0.25;

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
  P.Q = Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  // Selection matrix for normal forces
  Matrix S(normalForceSelectionMatrix(contacts));
  Matrix STS(matrixMultiply(S.transposed(), S));
  P.Q.copySubMatrixBlockIndices(P.var["beta"], P.var["beta"], STS);
}

void
GraspSolver::resultantWrenchObjective(GraspStruct &P)
{
  P.Q = Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.Q.copySubMatrixBlockIndices(P.var["r"], P.var["r"], Matrix::EYE(6, 6));
}

void
GraspSolver::virtualLimitsObjective(GraspStruct &P)
{
  P.Q = Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.cj.copySubMatrixBlockIndices(0, P.var["v"], Matrix::EYE(1,1));
}

void
GraspSolver::resultantAndVLObjective(GraspStruct &P)
{
  P.Q = Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  P.cj = Matrix(Matrix::ZEROES<Matrix>(1, P.block_cols));
  P.Q.copySubMatrixBlockIndices(P.var["r"], P.var["r"], Matrix::EYE(6, 6));
  P.cj.copySubMatrixBlockIndices(0, P.var["v"], Matrix::EYE(1,1));
  P.cj.multiply(1.0e-6);
}

void
GraspSolver::preloadTauObjective(GraspStruct &P)
{
  P.Q = Matrix(Matrix::ZEROES<Matrix>(P.block_cols, P.block_cols));
  Matrix cj(1, P.block_cols[P.var["tau"]]);
  cj.setAllElements(1.0);
  P.cj = Matrix::ZEROES<Matrix>(1, P.block_cols);
  P.cj.copySubMatrixBlockIndices(0, P.var["tau"], cj);
  //P.Q.copySubMatrixBlockIndices(P.var["tau"], P.var["tau"], Matrix::EYE(numPreloadVar, numPreloadVar));
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

void
GraspSolver::resultantDirectionConstraint(GraspStruct &P, const Matrix &wrench)
{
  Matrix Eq( Matrix::ZEROES<Matrix>(6, P.block_cols) );
  Eq.copySubMatrixBlockIndices(0, P.var["r"], Matrix::EYE(6,6));
  Eq.copySubMatrixBlockIndices(0, P.var["s"], wrench);

  P.Eq_list.push_back(Eq);
  P.b_list.push_back(Matrix::ZEROES<Matrix>(6,1));
}

//  -------------------------  Inequality Constraints  ------------------------------  //

void
GraspSolver::virtualSpringIndicatorConstraint(GraspStruct &P, double normUnc)
{
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix S(normalForceSelectionMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix NJ(matrixMultiply(N, J));
  Matrix E(tangentialDisplacementSummationMatrix(contacts));

  Matrix S_k(S);
  S_k.multiply(1/kSpringStiffness);

  // we assume the worst case such that normal force is diminished
  K.multiply(cos(normUnc));
  NJ.multiply(cos(normUnc));
  E.multiply(sin(normUnc));

  std::vector<int> block_rows(2, numContacts);
  Matrix InEq(Matrix::ZEROES<Matrix>(block_rows, P.block_cols));
  InEq.copySubMatrixBlockIndices(0, P.var["beta"], S.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["beta"], S_k.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["x"],    K);
  InEq.copySubMatrixBlockIndices(1, P.var["q"],    NJ.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["alpha"],E.negative());
  P.InEq_list.push_back(InEq);
  P.ib_list.push_back(Matrix::ZEROES<Matrix>(InEq.rows(), 1));

  int y1_index = 0;
  for (int i=0; i<P.var["y1"]; i++) y1_index += P.block_cols[i];

  int counter = 0;
  int index = 0;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++, counter++) {
    int numFE = (*it)->numFrictionEdges;

    // y1==0 -> Contact breaks
    Matrix Si(Matrix::ZEROES<Matrix>(1, P.block_cols[P.var["beta"]]));
    Si.elem(0, index) = 1.0;
    Matrix Eq(Matrix::ZEROES<Matrix>(1, P.block_cols));
    Eq.copySubMatrixBlockIndices(0, P.var["beta"], Si);
    P.Indic_lhs.push_back(Eq);
    P.Indic_rhs.push_back(Matrix::ZEROES<Matrix>(1,1));
    P.Indic_var.push_back(y1_index+counter);
    P.Indic_val.push_back(0);
    P.Indic_sense.push_back("eq");

    // y1==1 -> virtual spring
    Eq.multiply(1/kSpringStiffness);
    Eq.copySubMatrixBlockIndices(0, P.var["x"],     K.getSubMatrix(counter, 0, 1, K.cols()).negative());
    Eq.copySubMatrixBlockIndices(0, P.var["q"],     NJ.getSubMatrix(counter, 0, 1, NJ.cols()));
    Eq.copySubMatrixBlockIndices(0, P.var["alpha"], E.getSubMatrix(counter, 0, 1, E.cols()));
    P.Indic_lhs.push_back(Eq);
    P.Indic_rhs.push_back(Matrix::ZEROES<Matrix>(1,1));
    P.Indic_var.push_back(y1_index+counter);
    P.Indic_val.push_back(1);
    P.Indic_sense.push_back("eq");

    index += numFE+1;
  }
}

void
GraspSolver::virtualSpringConstraint(GraspStruct &P, const Matrix &beta_p) 
{
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
  S_k.multiply(1/kSpringStiffness);

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
GraspSolver::nonBackdrivableJointIndicatorConstraint(GraspStruct &P, const Matrix &preload)
{
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  Matrix JTD(matrixMultiply(J.transposed(), D));

  int y2_index = 0;
  for (int i=0; i<P.var["y2"]; i++) y2_index += P.block_cols[i];
  
  int tau_index = 0;
  int numJoints = preload.rows();
  for (int i=0; i<numJoints; i++) {
    Matrix sq(Matrix::ZEROES<Matrix>(1, numJoints));
    sq.elem(0, i) = 1.0;
    if (preload.elem(i,0) == 0) {
      Matrix Eq(Matrix::ZEROES<Matrix>(1, P.block_cols));
      Eq.copySubMatrixBlockIndices(0, P.var["q"], sq);
      P.Eq_list.push_back(Eq);
      P.b_list.push_back(Matrix::ZEROES<Matrix>(1,1));
    } else {
      std::vector<int> block_rows(2, 1);
      Matrix InEq(Matrix::ZEROES<Matrix>(block_rows, P.block_cols));
      InEq.copySubMatrixBlockIndices(0, P.var["q"], sq.negative());
      InEq.copySubMatrixBlockIndices(1, P.var["beta"], JTD.getSubMatrix(i, 0, 1, JTD.cols()).negative());
      Matrix ib(2,1);
      ib.elem(0,0) = 0;
      if (preload.elem(i,0) > 0) {
        ib.elem(1,0) = -preload.elem(i,0);
      } else {
        Matrix stau(Matrix::ZEROES<Matrix>(1, P.block_cols[P.var["tau"]]));
        stau.elem(0, tau_index) = 1.0;
        InEq.copySubMatrixBlockIndices(1, P.var["tau"], stau);
        ib.elem(1,0) = 0.0;
      }
      P.InEq_list.push_back(InEq);
      P.ib_list.push_back(ib);

      // y2==0 -> finger follows
      Matrix Eq(Matrix::ZEROES<Matrix>(1, P.block_cols));
      Eq.copySubMatrixBlockIndices(0, P.var["beta"], JTD.getSubMatrix(i, 0, 1, JTD.cols()));
      Matrix b(1,1);
      if (preload.elem(i,0) > 0) {
        b.elem(0,0) = preload.elem(i,0);
      } else {
        Matrix stau(Matrix::ZEROES<Matrix>(1, P.block_cols[P.var["tau"]]));
        stau.elem(0, tau_index) = -1.0;
        Eq.copySubMatrixBlockIndices(0, P.var["tau"], stau);
        b.elem(0,0) = 0.0;
        tau_index++;
      }
      P.Indic_lhs.push_back(Eq);
      P.Indic_rhs.push_back(b);
      P.Indic_var.push_back(y2_index+i);
      P.Indic_val.push_back(0);
      P.Indic_sense.push_back("eq");

      // y2==1 -> q==0, finger does not backdrive
      Eq.setAllElements(0.0);
      Eq.copySubMatrixBlockIndices(0, P.var["q"], sq);
      P.Indic_lhs.push_back(Eq);
      P.Indic_rhs.push_back(Matrix::ZEROES<Matrix>(1,1));
      P.Indic_var.push_back(y2_index+i);
      P.Indic_val.push_back(1);
      P.Indic_sense.push_back("eq");
    }
  }
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
GraspSolver::frictionConeConstraint(GraspStruct &P, const Matrix &beta_p/*=Matrix(0,0)*/) 
{
  // results of friction inequality arising from original contact forces
  Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
  Matrix d(Matrix::ZEROES<Matrix>(F.rows(), 1));
  if (beta_p.rows()) d = matrixMultiply(F, beta_p).negative();

  Matrix InEq( Matrix::ZEROES<Matrix>(numContacts, P.block_cols) );
  InEq.copySubMatrixBlockIndices(0, P.var["beta"], F);
  P.InEq_list.push_back(InEq);

  P.ib_list.push_back(d);
}

void
GraspSolver::frictionConeEdgeIndicatorConstraint(GraspStruct &P)
{
  Matrix G(frictionConeEdgeMatrix(contacts));
  Matrix Eq(Matrix::ZEROES<Matrix>(numContacts, P.block_cols));
  Eq.copySubMatrixBlockIndices(0, P.var["beta"], G);

  int y3_index = 0;
  for (int i=0; i<P.var["y3"]; i++) y3_index += P.block_cols[i];

  for (int i=0; i<numContacts; i++) {
    P.Indic_lhs.push_back(Eq.getSubMatrix(i, 0, 1, Eq.cols()));
    P.Indic_rhs.push_back(Matrix::ZEROES<Matrix>(1,1));
    P.Indic_var.push_back(y3_index+i);
    P.Indic_val.push_back(0);
    P.Indic_sense.push_back("geq");
  }
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
GraspSolver::contactMovementIndicatorConstraint(GraspStruct &P)
{
  int y3_index = 0;
  for (int i=0; i<P.var["y3"]; i++) y3_index += P.block_cols[i];

  int counter = 0;
  int index = 0;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++, counter++) {
    int numFE = (*it)->numFrictionEdges;

    Matrix sigma(Matrix::ZEROES<Matrix>(numFE, P.block_cols[P.var["alpha"]]));
    sigma.copySubMatrix(0, index+1, Matrix::EYE(numFE, numFE));

    Matrix Eq( Matrix::ZEROES<Matrix>(numFE, P.block_cols));
    Eq.copySubMatrixBlockIndices(0, P.var["alpha"], sigma);
    P.Indic_lhs.push_back(Eq);
    P.Indic_rhs.push_back(Matrix::ZEROES<Matrix>(numFE, 1));
    P.Indic_var.push_back(y3_index+counter);
    P.Indic_val.push_back(1);
    P.Indic_sense.push_back("eq");

    index += numFE+1;
  }
}

void
GraspSolver::contactMovementConstraint(GraspStruct &P)
{
  Matrix sigmaDiag(tangentialDisplacementSummationMatrix(contacts));

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
GraspSolver::amplitudesSOS2Constraint(GraspStruct &P, const Matrix &beta_p/*=Matrix(0,0)*/)
{
  int z_index = 0;
  for (int i=0; i<P.var["z"]; i++) z_index += P.block_cols[i];

  std::list<Matrix *> alpha_blocks;
  std::list<Matrix *> beta_blocks;
  std::list<Matrix *> z_blocks;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    int numFrictionEdges = (*it)->numFrictionEdges;

    Matrix *block = new Matrix(Matrix::ZEROES<Matrix>(numFrictionEdges, numFrictionEdges+1));
    block->copySubMatrix(0, 1, Matrix::EYE(numFrictionEdges, numFrictionEdges));
    
    Matrix *z_block = new Matrix(Matrix::ZEROES<Matrix>(numFrictionEdges, numFrictionEdges+1));
    z_block->copySubMatrix(0, 0, Matrix::EYE(numFrictionEdges, numFrictionEdges));
    z_block->elem(0, numFrictionEdges) = 1.0;

    alpha_blocks.push_back(block);
    beta_blocks.push_back(block);
    z_blocks.push_back(z_block);

    P.SOS_index.push_back(z_index);
    z_index += numFrictionEdges+1;
    P.SOS_len.push_back(numFrictionEdges+1);
    P.SOS_type.push_back(2);
  }

  Matrix alphaDiag( Matrix::BLOCKDIAG<Matrix>(&alpha_blocks) );
  Matrix betaDiag( Matrix::BLOCKDIAG<Matrix>(&beta_blocks) );
  Matrix zDiag( Matrix::BLOCKDIAG<Matrix>(&z_blocks) );

  while (!alpha_blocks.empty()) {
    delete alpha_blocks.back();
    alpha_blocks.pop_back();
  }
  while (!z_blocks.empty()) {
    delete z_blocks.back();
    z_blocks.pop_back();
  }

  std::vector<int> block_rows(2, alphaDiag.rows());
  Matrix InEq( Matrix::ZEROES<Matrix>(block_rows, P.block_cols) );

  InEq.copySubMatrixBlockIndices(0, P.var["alpha"], alphaDiag);
  InEq.copySubMatrixBlockIndices(0, P.var["z"],     zDiag.negative());
  InEq.copySubMatrixBlockIndices(1, P.var["beta"],  betaDiag);
  InEq.copySubMatrixBlockIndices(1, P.var["z"],     zDiag.negative());
  P.InEq_list.push_back(InEq);

  Matrix ib( Matrix::ZEROES<Matrix>(block_rows, 1) );
  if (beta_p.rows())
    ib.copySubMatrixBlockIndices(1, 0, matrixMultiply(betaDiag, beta_p.negative()));
  P.ib_list.push_back(ib);
}

void
GraspSolver::preloadTorqueLBConstraint(GraspStruct &P)
{
  int numPreloadVar = P.block_cols[P.var["tau"]];
  Matrix ones(numPreloadVar, 1);
  ones.setAllElements(-1.0);
  Matrix InEq = Matrix::ZEROES<Matrix>(numPreloadVar, P.block_cols);
  InEq.copySubMatrixBlockIndices(0, P.var["tau"], Matrix::EYE(numPreloadVar, numPreloadVar));
  InEq.copySubMatrixBlockIndices(0, P.var["v"], ones);
  P.InEq_list.push_back(InEq);
  P.ib_list.push_back(Matrix::ZEROES<Matrix>(numPreloadVar, 1));
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
  std::vector<int> block_rows;
  block_rows.push_back(numBetas);
  block_rows.push_back(numJoints);
  block_rows.push_back(numQs);
  block_rows.push_back(numContacts);
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
GraspSolver::variableBoundsAndTypes(GraspStruct &P, const Matrix &beta_p/*=Matrix(0,0)*/)
{
  std::list<std::string>::iterator it;
  for (it=P.varNames.begin(); it!=P.varNames.end(); it++) {
    std::string key = *it;
    int i = P.var[key];
    int size = P.block_cols[i];

    if (!key.compare("alpha")) {
      /*Matrix alpha_lb(size, 1);
      alpha_lb.setAllElements(0.0);*/
      Matrix alpha_lb(Matrix::MIN_VECTOR(size));
      int pos = 0;
      std::list<Contact*>::iterator it;
      for (it=contacts.begin(); it!=contacts.end(); it++) {
        alpha_lb.copySubMatrix(pos+1, 0, Matrix::ZEROES<Matrix>((*it)->numFrictionEdges, 1));
        //alpha_lb.elem(pos,0) = -AlphaMax;
        pos += (*it)->numFrictionEdges+1;
      }
      P.lb_list.push_back(alpha_lb);

      /*Matrix alpha_ub(size, 1);
      alpha_ub.setAllElements(AlphaMax);*/
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));

      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("beta")) {
      if (beta_p.rows()) P.lb_list.push_back(beta_p.negative());
      else P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      /*Matrix beta_ub(size, 1);
      beta_ub.setAllElements(BetaMax);*/
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("x")) { 
      P.lb_list.push_back(Matrix::MIN_VECTOR(size));
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("q")) {
      /*Matrix qbound(size, 1);
      qbound.setAllElements(QMax);*/
      Matrix qbound(Matrix::MAX_VECTOR(size));
      P.lb_list.push_back(qbound.negative());
      P.ub_list.push_back(qbound);
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else if (!key.compare("tau")) {
      P.lb_list.push_back(Matrix::ZEROES<Matrix>(size, 1));
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
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
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
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
      /*Matrix s_ub(size, 1);
      s_ub.setAllElements(1.0);
      P.ub_list.push_back(s_ub);*/
      P.ub_list.push_back(Matrix::MAX_VECTOR(size));
      P.types.push_back(Matrix::ZEROES<Matrix>(size, 1));
    }

    else {
      DBGA("variableBoundsAndTypes: Requested unknown variable"); 
      assert(0);
    }
  }
}

//  ----  Routines constructing refinement solver problems  ----  //

//! The variables necessary for a base problem formulation for the
//! refinement solver
/*!
  alpha: contact motion amplitude
  beta: contact force amplitude
  x: object motion
  q: joint motion
  y1: contact breaking decision variable
  y2: joint backdriving decision variable
  y3: contact sliding decision variable
  z: SOS2 variables for contact forces and motions
*/
void
GraspSolver::refinementBaseVariables(GraspStruct &P)
{
  int totalFrictionEdges = 0;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    totalFrictionEdges += (*it)->numFrictionEdges+1;
  }

  // unknowns
  P.var["alpha"] = 0; P.block_cols.push_back(totalFrictionEdges); P.varNames.push_back("alpha");
  P.var["beta"]  = 1; P.block_cols.push_back(totalFrictionEdges); P.varNames.push_back("beta");
  P.var["x"]     = 2; P.block_cols.push_back(6);                  P.varNames.push_back("x");
  P.var["q"]     = 3; P.block_cols.push_back(numJoints);          P.varNames.push_back("q");
  P.var["y1"]    = 4; P.block_cols.push_back(numContacts);        P.varNames.push_back("y1");
  P.var["y2"]    = 5; P.block_cols.push_back(numJoints);          P.varNames.push_back("y2");
  P.var["y3"]    = 6; P.block_cols.push_back(numContacts);        P.varNames.push_back("y3");
  P.var["z"]     = 7; P.block_cols.push_back(totalFrictionEdges); P.varNames.push_back("z");  
}

//! The constraints making up a base problem formulation for the
//! refinement solver
void
GraspSolver::refinementBaseConstraints(GraspStruct &P, const Matrix &preload, 
                                       double normUnc, bool rigid) 
{
  // equality constraints
  movementAmplitudesConstraint(P);

  // inequality constraints
  virtualSpringIndicatorConstraint(P, normUnc);
  if (rigid) rigidJointsConstraint(P);
  else nonBackdrivableJointIndicatorConstraint(P, preload);
  frictionConeConstraint(P);
  frictionConeEdgeIndicatorConstraint(P);
  contactMovementIndicatorConstraint(P);
  amplitudesSOS2Constraint(P);

  // solution bounds and types
  variableBoundsAndTypes(P);
}

//! The equilibrium problem formulation for the refinement solver
void
GraspSolver::refinementFormulationEquilibrium(GraspStruct &P, const Matrix &preload,
  double normUnc, const Matrix &wrench /*=ZEROES*/, bool rigid /*=false*/)
{
  refinementBaseVariables(P);
  refinementBaseConstraints(P, preload, normUnc, rigid);

  // equality constraints
  objectWrenchConstraint(P, wrench);
}

//! The maximum wrench problem formulation for the refinement solver
/*!
  Variables
  r: residual
  s: magnitude of residual
*/
void
GraspSolver::refinementFormulationFindMax(GraspStruct &P, const Matrix &preload,
  double normUnc, const Matrix &direction, bool rigid /*=false*/)
{
  refinementBaseVariables(P);
  P.var["r"] = 8; P.block_cols.push_back(6); P.varNames.push_back("r");
  P.var["s"] = 9; P.block_cols.push_back(1); P.varNames.push_back("s");
  refinementBaseConstraints(P, preload, normUnc, rigid);

  // objective: minimize resultant
  resultantWrenchObjective(P);

  // equality constraints
  resultantWrenchConstraint(P, direction);
  resultantDirectionConstraint(P, direction);
}

//! The optimal preload problem formulation for the refinement solver
/*!
  Variables
  tau: preload torques
  v: maximum preload torque
*/

void
GraspSolver::refinementFormulationPreload(GraspStruct &P, const Matrix &preload,
  double normUnc, const Matrix &wrench /*=ZEROES*/, bool rigid /*=false*/)
{
  refinementBaseVariables(P);
  int numPreloadVar = 0;
  for (int i=0; i<preload.rows(); i++) {
    if (preload.elem(i,0) < 0) numPreloadVar++;
  }
  P.var["tau"] = 8; P.block_cols.push_back(numPreloadVar); P.varNames.push_back("tau");
  P.var["v"] = 9;   P.block_cols.push_back(1);             P.varNames.push_back("v");
  refinementBaseConstraints(P, preload, normUnc, rigid);

  // objective: minimize maximum joint torque
  virtualLimitsObjective(P);

  // equality constraints
  objectWrenchConstraint(P, wrench);

  // inequality constraints
  preloadTorqueLBConstraint(P);
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
  int totalFrictionEdges = 0;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    totalFrictionEdges += (*it)->numFrictionEdges+1;
  }

  // unknowns
  P.var["beta"]  = 0; P.block_cols.push_back(totalFrictionEdges); P.varNames.push_back("beta");
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
  Matrix beta_p( Matrix::ZEROES<Matrix>(totalFrictionEdges,1) );
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
GraspSolver::refinementSolver(SolutionStruct &S, std::tr1::function<void(GraspStruct&)> formulation, double coneTol)
{
  while (true) {

    // Solve problem and exit if infeasible
    GraspStruct P;
    formulation(P);
    int result = solveProblem(P, S);
    if (result) return result;

    // get active friction and motion edges as well as friction edge amplitudes
    Matrix z(S.sol.getSubMatrixBlockIndices(S.var["z"], 0));
    Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));

    // Step through all contacts
    int contact_index = 0;
    bool complete = true;
    std::list<Contact*>::iterator it;
    for (it=contacts.begin(); it!=contacts.end(); it++) {

      // Number of friction edges for this contact
      int numFrictionEdges = (*it)->numFrictionEdges;

      // If this contact is inactive, we are done with it
      if (beta.elem(contact_index, 0) < Matrix::EPS) {
        contact_index += numFrictionEdges+1;
        continue;
      }

      // Unpack friction edges for simplicity
      int container_size = sizeof((*it)->frictionEdges) / sizeof((*it)->frictionEdges[0]);
      double frictionEdges[container_size];
      memcpy(frictionEdges, (*it)->frictionEdges, sizeof(frictionEdges));

      // Find the two active friction edges
      int edge1 = -1;
      int edge2;
      for (int i=0; i<numFrictionEdges+1; i++) {
        if (z.elem(contact_index + i, 0) > Matrix::EPS) {
          edge1 = (i       < numFrictionEdges) ?       i : 0;
          edge2 = (edge1+1 < numFrictionEdges) ? edge1+1 : 0;
          break;
        }
      }
      contact_index += numFrictionEdges+1;

      // If no friction edges are active, we are done with this contact
      if (edge1 == -1) continue;

      // Find angle between and length of active friction edges
      double angle, len1, len2;
      int check_counter = 0;
      while (true) {
        double num  = frictionEdges[6*edge1]   * frictionEdges[6*edge2];
               num += frictionEdges[6*edge1+1] * frictionEdges[6*edge2+1];
        len1 = sqrt(pow(frictionEdges[6*edge1], 2) + pow(frictionEdges[6*edge1+1], 2));
        len2 = sqrt(pow(frictionEdges[6*edge2], 2) + pow(frictionEdges[6*edge2+1], 2));
        angle = acos(num/(len1*len2));
        if (angle > Matrix::EPS) break;

        // If the angle is zero, work on adjacent pair of friction edges
        // with least refinement instead
        if (len1 > len2) {
          edge1 = (edge1-1 < 0) ? numFrictionEdges-1 : edge1-1;
          edge2 = (edge2-1 < 0) ? numFrictionEdges-1 : edge2-1;
        } else if (len1 < len2) {
          edge1 = (edge1+1 < numFrictionEdges) ? edge1+1 : 0;
          edge2 = (edge2+1 < numFrictionEdges) ? edge2+1 : 0;
        } else {
          DBGA("Edges with zero angle between them and same length");
          exit(0);
        }

        // Just a check to make sure we do not have incorrect friction
        // edges for any reason
        if (check_counter++) {
          DBGA("Three coincident friction edges");
          exit(0);
        }
      }

      //  Active friction edges should have the same length
      if (abs(len1 - len2) > Matrix::EPS) {
        DBGA("Active friction edges have different length");
        exit(0);
      }

      // If the angle is already below the required tolerance we are done
      // with this contact. Otherwise work is required and we know there
      // will be at least another iteration so we can set the no-exit flag
      if (angle <= coneTol) continue;
      complete = false;

      // Check if adding new friction edges would exceed container size
      if (numFrictionEdges + 3 > container_size/6) {
        DBGA("FrictionEdge container size exceeded. Cannot add any more friction edges.");
        exit(0);
      }

      // Put friction edges into a vector of matrices to make modifying them easier
      std::vector<Matrix> fe_vec;
      fe_vec.reserve(numFrictionEdges);
      for (int i=0; i<numFrictionEdges; i++) {
        Matrix fe(6,1);
        for (int j=0; j<6; j++) {
          fe.elem(j,0) = frictionEdges[6*i+j];
        }
        fe_vec.push_back(fe);
      }

      // Vector of new friction edges
      std::vector<Matrix> new_fe_vec;
      new_fe_vec.reserve(3);
      Matrix new_fe(Matrix::ZEROES<Matrix>(6,1));

      double mult = 1.0;
      double new_angle = angle / 2.0;
      do {
        new_angle /= 2;
        mult *= cos(new_angle);
      } while (new_angle > coneTol / 2.0);
      new_fe.elem(0,0) = frictionEdges[6*edge1] / (len1 * mult);
      new_fe.elem(1,0) = frictionEdges[6*edge1+1] / (len1 * mult);
      new_fe_vec.push_back(new_fe);

      new_fe.setAllElements(0.0);
      new_fe.elem(0,0) = frictionEdges[6*edge1] + frictionEdges[6*edge2];
      new_fe.elem(1,0) = frictionEdges[6*edge1+1] + frictionEdges[6*edge2+1];
      double new_len = sqrt(pow(new_fe.elem(0,0), 2) + pow(new_fe.elem(1,0), 2));
      new_fe.multiply(1/ (new_len * mult));
      new_fe_vec.push_back(new_fe);

      new_fe.setAllElements(0.0);
      new_fe.elem(0,0) = frictionEdges[6*edge2] / (len1 * mult);
      new_fe.elem(1,0) = frictionEdges[6*edge2+1] / (len1 * mult);
      new_fe_vec.push_back(new_fe);

      // Populate list of final friction edges
      std::list<Matrix> final_fe;

      for (int i=0; i<edge1; i++) final_fe.push_back(fe_vec[i]);

      int edge0 = (edge1-1 < 0) ? numFrictionEdges-1 : edge1-1;
      if (angleBetweenEdges(fe_vec[edge0], fe_vec[edge1]) < Matrix::EPS) {
        double difference = matrixAdd(fe_vec[edge0], new_fe_vec[0].negative()).fnorm();
        if (difference > Matrix::EPS) {
          final_fe.push_back(new_fe_vec[0]);
        }
      } else {
        final_fe.push_back(fe_vec[edge1]);
        final_fe.push_back(new_fe_vec[0]);
      }

      final_fe.push_back(new_fe_vec[1]);

      int edge3 = (edge2+1 < numFrictionEdges) ? edge2+1 : 0;
      if (angleBetweenEdges(fe_vec[edge2], fe_vec[edge3]) < Matrix::EPS) {
        double difference = matrixAdd(fe_vec[edge3], new_fe_vec[2].negative()).fnorm();
        if (difference > Matrix::EPS) {
          final_fe.push_back(new_fe_vec[2]);
        }
        if (edge3 && edge2) {
          for (int i=edge3; i<numFrictionEdges; i++) final_fe.push_back(fe_vec[i]);
        } else if (!edge2) {
          final_fe.pop_front();
        }
      } else {
        final_fe.push_back(new_fe_vec[2]);
        if (edge2) {
          for (int i=edge2; i<numFrictionEdges; i++) final_fe.push_back(fe_vec[i]);
        }
      }

      // Check if number of friction edges is consistent
      if (final_fe.size() > numFrictionEdges+3) {
        DBGA("Number of final friction edges exceeds maximum possible");
        exit(0);
      }
      // Check if friction edges are correct
      if (!checkFrictionEdges(final_fe)) exit(0);

      // Update friction edges
      std::list<Matrix>::iterator fe_it = final_fe.begin();
      for (int i=0; fe_it!=final_fe.end(); i++, fe_it++) {
        for (int j=0; j<6; j++) {
          (*it)->frictionEdges[6*i+j] = fe_it->elem(j,0);
          (*it)->getMate()->frictionEdges[6*i+j] = fe_it->elem(j,0);
        }
      }
      (*it)->numFrictionEdges = final_fe.size();
      (*it)->getMate()->numFrictionEdges = final_fe.size();
    }
    if (complete) return result;
  }
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
  while (true) {
    iterations++;
    
    GraspStruct P;
    formulation(P, startingX, movement_directions);
    result = solveProblem(P, S);

    if (result) {
      DBGA(" Problem failed");
      result = 1;
      break;
    }

    Matrix x(S.sol.getSubMatrixBlockIndices(S.var["x"], 0));
    Matrix r(S.sol.getSubMatrixBlockIndices(S.var["r"], 0));

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
GraspSolver::checkWrenchRefinement(Matrix &preload, const Matrix &wrench, double coneTol/*=1.0*/, double normUnc/*=2.5*/)
{
  removeJointsBeyondContact(joints, contacts, preload);
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    (*it)->setUpFrictionEdges();
    (*it)->getMate()->setUpFrictionEdges();
  }
  modifyFrictionEdges(coneTol);

  SolutionStruct S;
  std::tr1::function<void(GraspStruct&)> formulation
    = std::tr1::bind(&GraspSolver::refinementFormulationEquilibrium, this, 
      std::tr1::placeholders::_1, preload, normUnc, wrench, false);
  int result = refinementSolver(S, formulation, coneTol);
  g->getObject()->redrawFrictionCones();
  if (result) return result;

  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
  drawContactWrenches(beta);
  drawObjectMovement(S);
  return result;
}

double
GraspSolver::findMaximumWrenchRefinement(Matrix &preload, const Matrix &direction, double coneTol/*=1.0*/, double normUnc/*=2.5*/)
{
  removeJointsBeyondContact(joints, contacts, preload);
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    (*it)->setUpFrictionEdges();
    (*it)->getMate()->setUpFrictionEdges();
  }
  modifyFrictionEdges(coneTol);

  SolutionStruct S;
  std::tr1::function<void(GraspStruct&)> formulation
    = std::tr1::bind(&GraspSolver::refinementFormulationFindMax, this, 
      std::tr1::placeholders::_1, preload, normUnc, direction, false);
  int result = refinementSolver(S, formulation, coneTol);
  g->getObject()->redrawFrictionCones();
  if (result) return 0.0;

  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
  drawContactWrenches(beta);
  drawObjectMovement(S);
  Matrix r(S.sol.getSubMatrixBlockIndices(S.var["r"], 0));
  return direction.fnorm() - r.fnorm();
}

Matrix
GraspSolver::optimalPreloads(Matrix &preload, const Matrix &wrench, double coneTol/*=1.0*/, double normUnc/*=2.5*/)
{
  removeJointsBeyondContact(joints, contacts, preload);
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    (*it)->setUpFrictionEdges();
    (*it)->getMate()->setUpFrictionEdges();
  }
  modifyFrictionEdges(coneTol);

  SolutionStruct S;
  std::tr1::function<void(GraspStruct&)> formulation
    = std::tr1::bind(&GraspSolver::refinementFormulationPreload, this, 
      std::tr1::placeholders::_1, preload, normUnc, wrench, false);
  int result = refinementSolver(S, formulation, coneTol);
  g->getObject()->redrawFrictionCones();
  if (result) return Matrix(0,0);

  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
  drawContactWrenches(beta);
  drawObjectMovement(S);

  Matrix tau(S.sol.getSubMatrixBlockIndices(S.var["tau"], 0));
  Matrix optPreload(preload);
  int i,j;
  for (i=j=0; i<preload.rows(); i++)
    if (optPreload.elem(i,0) < 0)
      optPreload.elem(i,0) = tau.elem(j++,0);
  return optPreload;
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
    Matrix beta_p(S_p.sol.getSubMatrixBlockIndices(S_p.var["beta"], 0));
    if (tendon)
      formulation = std::tr1::bind(
        &GraspSolver::iterativeTendonFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p);
    else 
      formulation = std::tr1::bind(
        &GraspSolver::iterativeFormulation, this, std::tr1::placeholders::_1, preload, 
        std::tr1::placeholders::_2, std::tr1::placeholders::_3, wrench, beta_p, rigid);
    result = iterativeSolver(S, cone_movement, formulation);
    Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
    matrixAdd(beta, beta_p, beta);
    S.sol.copySubMatrixBlockIndices(S.var["beta"], 0, beta);
  }
  if (result) return result;
  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
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
    Matrix beta_p(S_p.sol.getSubMatrixBlockIndices(S_p.var["beta"], 0));
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

void
GraspSolver::create2DMapRefinement(Matrix &preload, const Matrix &direction1, const Matrix &direction2,
  std::ofstream &file, double coneTol/*=1.0*/, double normUnc/*=2.5*/)
{
  file << "d1, d2" << std::endl;

  int directionSteps = 360;
  for (int i=0; i<directionSteps; i++) {

    Matrix xcomponent(direction1);
    Matrix ycomponent(direction2);
    xcomponent.multiply(sin(2*M_PI*i/directionSteps));
    ycomponent.multiply(cos(2*M_PI*i/directionSteps));

    Matrix direction(matrixAdd(xcomponent, ycomponent));
    direction.multiply(1.0 / direction.fnorm());
    double max = findMaximumWrenchRefinement(preload, direction, coneTol, normUnc);

    Matrix w_max(2,1);
    w_max.elem(0,0) = sin(2*M_PI*i/directionSteps);
    w_max.elem(1,0) = cos(2*M_PI*i/directionSteps);
    w_max.multiply(max);
    file << -w_max.elem(0,0) << ", " << -w_max.elem(1,0) << "\n";

    DBGA("Iteration " << i+1 << "/" << directionSteps << ": " << max);
  }
}

void
GraspSolver::create2DMapIterative(Matrix &preload, const Matrix &direction1, const Matrix &direction2,
  std::ofstream &file, bool single_step/*=false*/, bool cone_movement/*=false*/, bool tendon/*=false*/,
  bool rigid/*=false*/)
{
  file << "d1, d2" << std::endl;

  int directionSteps = 360;
  for (int i=0; i<directionSteps; i++) {

    Matrix xcomponent(direction1);
    Matrix ycomponent(direction2);
    xcomponent.multiply(sin(2*M_PI*i/directionSteps));
    ycomponent.multiply(cos(2*M_PI*i/directionSteps));

    Matrix direction(matrixAdd(xcomponent, ycomponent));
    direction.multiply(1.0 / direction.fnorm());
    double max = findMaximumWrenchIterative(preload, direction, single_step, cone_movement, tendon, rigid);  

    Matrix w_max(2,1);
    w_max.elem(0,0) = sin(2*M_PI*i/directionSteps);
    w_max.elem(1,0) = cos(2*M_PI*i/directionSteps);
    w_max.multiply(max);
    file << -w_max.elem(0,0) << ", " << -w_max.elem(1,0) << "\n";

    DBGA("---------------- Iteration " << i+1 << "/" << directionSteps << " complete");
  }
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
  int result = MIPSolver(P.Q, P.cj, 
                         Eq, b, InEq, ib, P.QInEq, P.iq, P.qib,
                         P.Indic_lhs, P.Indic_rhs, P.Indic_var, P.Indic_val, P.Indic_sense,
                         P.SOS_index, P.SOS_len, P.SOS_type, 
                         lb, ub, sol, types, &objVal);
  S.block_cols = P.block_cols;
  S.var = P.var;
  S.sol = sol;

  return result;

  if (result) return result;

  /*Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));

  DBGA("R:");
  DBGA(R);
  DBGA("RT:");
  DBGA(RT);
  DBGA("K:");
  DBGA(K);
  DBGA("x:");
  DBGA(S.sol.getSubMatrixBlockIndices(S.var["x"], 0));
  DBGA("k:");
  DBGA(matrixMultiply(K, S.sol.getSubMatrixBlockIndices(S.var["x"], 0)));*/

  // Check if solution is close to virtual bounds. If so issue a warning
  if (S.var.find("beta") != S.var.end()) {
    Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
    if (beta.max() > kVirtualLimitTolerance * BetaMax) {
      DBGA("beta (" << beta.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << BetaMax << "). Consider increasing BetaMax");      
      exit(0);
    }
    Matrix J(g->contactJacobian(joints, contacts));
    Matrix D(Contact::frictionForceBlockMatrix(contacts));
    Matrix JTD(matrixMultiply(J.transposed(), D));
    Matrix tau(matrixMultiply(JTD, beta));
    if (tau.max() > kVirtualLimitTolerance * TauMax) {
      DBGA("tau (" << tau.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << TauMax << "). Consider increasing TauMax");
      exit(0);
    }
  }
  if (S.var.find("q") != S.var.end()) {
    Matrix q(S.sol.getSubMatrixBlockIndices(S.var["q"], 0));
    if (q.max() > kVirtualLimitTolerance * QMax) {
      DBGA("q (" << q.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << QMax << "). Consider increasing QMax");      
      exit(0);
    }
  }
  if (S.var.find("f") != S.var.end()) {
    Matrix f(S.sol.getSubMatrixBlockIndices(S.var["f"], 0));
    if (f.max() > kVirtualLimitTolerance * TauMax) {
      DBGA("f (" << f.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << TauMax << "). Consider increasing TauMax");      
      exit(0);
    }
  }
  if (S.var.find("alpha") != S.var.end()) {
    Matrix alpha(S.sol.getSubMatrixBlockIndices(S.var["alpha"], 0));
    // In special cases where object movement does not compress any virtual 
    // springs, the solver has the annoying habit of moving the object as far 
    // as it may without breaking the constraint on alpha. In this case the 
    // greatest alpha will be exactly equal to the virtual limit. Hence, an
    // additional test for this case had to be added to the below check. 
    if (alpha.max() > kVirtualLimitTolerance * AlphaMax/* && alpha.max() != AlphaMax*/) {
      DBGA("alpha (" << alpha.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << AlphaMax << "). Consider increasing AlphaMax");
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

    Matrix q(S.sol.getSubMatrixBlockIndices(S.var["q"], 0));
    Matrix x(S.sol.getSubMatrixBlockIndices(S.var["x"], 0));
    Matrix Kx(matrixMultiply(K, x));
    Matrix NJq(matrixMultiply(NJ, q));
    Matrix k(matrixAdd(matrixMultiply(NJ, q), matrixMultiply(K, x).negative()));
    if (k.max() > kVirtualLimitTolerance * KMax) {
      DBGA("k (" << k.max() 
        << ") is larger than " << kVirtualLimitTolerance << " of the virtual limit ("
        << KMax << "). Consider increasing KMax");      
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
    output << S.sol.getSubMatrixBlockIndices(it->second, 0) << std::endl;
  }
  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
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

void
GraspSolver::modifyFrictionEdges(double coneTol)
{
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    for (int i=0; i<(*it)->numFrictionEdges; i++) {
      int j = (i<(*it)->numFrictionEdges-1) ? i+1 : 0;
      double num  = (*it)->frictionEdges[6*i]   * (*it)->frictionEdges[6*j];
             num += (*it)->frictionEdges[6*i+1] * (*it)->frictionEdges[6*j+1];
      double len1 = sqrt(pow((*it)->frictionEdges[6*i], 2) + pow((*it)->frictionEdges[6*i+1], 2));
      double len2 = sqrt(pow((*it)->frictionEdges[6*j], 2) + pow((*it)->frictionEdges[6*j+1], 2));
      double angle = acos(num/(len1*len2));
      double mult = 1.0;
      if (angle > Matrix::EPS) {
        mult = len1;
        do {
          angle /= 2;
          mult *= cos(angle);
        } while(angle > coneTol / 2.0);
      } else continue;
      (*it)->frictionEdges[6*i]   /= mult;
      (*it)->frictionEdges[6*i+1] /= mult;
      (*it)->getMate()->frictionEdges[6*i]   /= mult;
      (*it)->getMate()->frictionEdges[6*i+1] /= mult;
    }
  }
  g->getObject()->redrawFrictionCones();
}

void
GraspSolver::systemEnergyError(SolutionStruct &S, const Matrix &preload, const Matrix &wrench, double *e, bool findMax /*=false*/)
{
  Matrix x( S.sol.getSubMatrixBlockIndices(S.var["x"], 0) );
  Matrix q( S.sol.getSubMatrixBlockIndices(S.var["q"], 0) );
  Matrix beta( S.sol.getSubMatrixBlockIndices(S.var["beta"], 0) );

  // Work done by applied wrench
  double E_in;
  if (findMax) {
    Matrix r( S.sol.getSubMatrixBlockIndices(S.var["r"], 0) );
    E_in = -0.5 * matrixMultiply(matrixAdd(wrench, r).transposed(), x).elem(0,0);
  } else {
    E_in = -0.5 * matrixMultiply(wrench.transposed(), x).elem(0,0);
  }

  // Work done by joints
  double E_tor = 0.5 * matrixMultiply(preload.transposed(), q).elem(0,0);
  E_in += E_tor;

  // Work done compressing virtual springs
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix S_matrix(normalForceSelectionMatrix(contacts));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix NJ(matrixMultiply(N, J));
  Matrix cn = matrixMultiply(S_matrix, beta);
  Matrix kn = matrixAdd(matrixMultiply(K, x), matrixMultiply(NJ, q).negative());
  double E_out = -0.5 * matrixMultiply(cn.transposed(), kn).elem(0,0);

  // Work done by sliding contacts
  Matrix M(tangentialDisplacementSelectionMatrix(numContacts));
  Matrix MRT(matrixMultiply(M, RT));
  Matrix MJ(matrixMultiply(M, J));
  Matrix kf(matrixAdd(matrixMultiply(MRT, x), matrixMultiply(MJ, q).negative()));
  std::list<Contact*>::iterator it = contacts.begin();
  double E_fric = 0;
  for (int i=0; it!=contacts.end(); it++, i++) {
    Matrix kif(kf.getSubMatrix(2*i, 0, 2, 1));
    double fric = 0.5 * (*it)->getCof() * cn.elem(i,0) * kif.fnorm();
    E_fric -= fric;
  }
  E_out += E_fric;
  e[0] = E_in + E_out;
  e[1] = (E_in + E_out) / E_in;
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
  Matrix x( S.sol.getSubMatrixBlockIndices(S.var["x"], 0) );
  double minWrench[6];
  for (int i=0; i<6; i++) {
    minWrench[i] = -x.elem(i,0);
  }
  g->setMinWrench(minWrench);
  graspitCore->getIVmgr()->drawWorstCaseWrenches();
}

Matrix
GraspSolver::computeSignificantX(SolutionStruct &S)
{
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix S_matrix(normalForceSelectionMatrix(contacts));
  Matrix c(matrixMultiply(S_matrix, S.sol.getSubMatrixBlockIndices(S.var["beta"], 0)));
  Matrix x(S.sol.getSubMatrixBlockIndices(S.var["x"], 0));
  Matrix x_sig(Matrix::ZEROES<Matrix>(x.rows(), x.cols()));
  for (int i=0; i<c.rows(); i++) {
    if (c.elem(i,0) / c.max() > 0.05) {
      Matrix sub_K(K.getSubMatrix(i, 0, 1, 6));
      Matrix components(6,1);
      for (int j=0; j<6; j++) {
        components.elem(j,0) = sub_K.elem(0,j) * x.elem(j,0);
      }
      for (int j=0; j<6; j++) {
        if (!x_sig.elem(j,0) && components.elem(j,0) / components.max() > 0.05) {
          x_sig.elem(j,0) = x.elem(j,0);
        }
      }
    }
  }
  return x_sig;
}

Matrix
GraspSolver::postProcessX(SolutionStruct &S)
{
  Matrix q( S.sol.getSubMatrixBlockIndices(S.var["q"], 0) );
  Matrix alpha( S.sol.getSubMatrixBlockIndices(S.var["alpha"], 0) );
  Matrix beta( S.sol.getSubMatrixBlockIndices(S.var["beta"], 0) );

  Matrix S_matrix(normalForceSelectionMatrix(contacts));
  S_matrix.multiply(1/kSpringStiffness);
  Matrix J(g->contactJacobian(joints, contacts));
  Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
  Matrix N(normalDisplacementSelectionMatrix(numContacts));
  Matrix RT(Grasp::graspMapMatrix(R).transposed());
  Matrix K(matrixMultiply(N, RT));
  Matrix NJ(matrixMultiply(N, J));
  Matrix M(tangentialDisplacementSelectionMatrix(numContacts));
  Matrix MRT(matrixMultiply(M, RT));
  Matrix k(matrixAdd(matrixMultiply(S_matrix, beta), matrixMultiply(NJ, q)));

  std::list<Matrix> spring_lhs_list;
  std::list<Matrix> spring_rhs_list;
  std::list<Matrix> nonpen_lhs_list;
  std::list<Matrix> nonpen_rhs_list;
  std::list<Matrix> slip_list;
  std::list<Matrix> E_list;

  int contact_count = 0;
  int contact_index = 0;
  std::list<Contact*>::iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++, contact_count++) {
    int numFrictionEdges = (*it)->numFrictionEdges;

    if (beta.elem(contact_index, 0) > Matrix::EPS) {
      spring_lhs_list.push_back(K.getSubMatrix(contact_count, 0, 1, 6));
      spring_rhs_list.push_back(k.getSubMatrix(contact_count, 0, 1, 1));

      slip_list.push_back(MRT.getSubMatrix(2*contact_count, 0, 2, 6));
      Matrix E(Matrix::ZEROES<Matrix>(2,2));
      int edge_count = 0;
      for (int j=0; j<numFrictionEdges; j++) {
        if (alpha.elem(contact_index+j+1, 0) > Matrix::EPS) {
          E.elem(0, edge_count) = (*it)->frictionEdges[6*j];
          E.elem(1, edge_count) = (*it)->frictionEdges[6*j+1];
          edge_count++;
        }
      }
      E_list.push_back(E);
    } else {
      nonpen_lhs_list.push_back(K.getSubMatrix(contact_count, 0, 1, 6));
      nonpen_rhs_list.push_back(k.getSubMatrix(contact_count, 0, 1, 1));
    }

    contact_index += numFrictionEdges+1;
  }

  Matrix E(Matrix::BLOCKDIAG<Matrix>(E_list));
  Matrix slip_lhs_x(Matrix::BLOCKCOLUMN<Matrix>(slip_list));

  Matrix spring_lhs(Matrix::BLOCKCOLUMN<Matrix>(spring_lhs_list));
  Matrix spring_rhs(Matrix::BLOCKCOLUMN<Matrix>(spring_rhs_list));
  Matrix nonpen_lhs(Matrix::BLOCKCOLUMN<Matrix>(nonpen_lhs_list));
  Matrix nonpen_rhs(Matrix::BLOCKCOLUMN<Matrix>(nonpen_rhs_list));
  Matrix slip_lhs(Matrix::BLOCKROW<Matrix>(slip_lhs_x, E.negative()));

  Matrix Eq(Matrix::ZEROES<Matrix>(spring_lhs.rows()+slip_lhs.rows(), slip_lhs.cols()));
  Eq.copySubMatrix(0, 0, spring_lhs);
  Eq.copySubMatrix(spring_lhs.rows(), 0, slip_lhs);

  Matrix b(Matrix::ZEROES<Matrix>(Eq.rows(), 1));
  b.copySubMatrix(0, 0, spring_rhs);

  Matrix InEq(Matrix::ZEROES<Matrix>(nonpen_lhs.rows(), Eq.cols()));
  InEq.copySubMatrix(0, 0, nonpen_lhs);
  Matrix ib(nonpen_rhs);

  Matrix sol(Eq.cols(), 1);
  Matrix Q(Matrix::ZEROES<Matrix>(sol.rows(), sol.rows()));
  Q.copySubMatrix(0, 0, Matrix::EYE(6,6));

  Matrix lb(Matrix::MIN_VECTOR(Eq.cols()));
  lb.copySubMatrix(6, 0, Matrix::ZEROES<Matrix>(E.cols(), 1));
  Matrix ub(Matrix::MAX_VECTOR(Eq.cols()));

  double objVal;
  int result = QPSolver(Q, Matrix(0,0),
                        Eq, b, InEq, ib,
                        lb, ub, sol, &objVal);

  Matrix x( S.sol.getSubMatrixBlockIndices(S.var["x"], 0) );
  if (result) {
    DBGA("Failed to post-process object motion");
    return x;
  } else {
    return sol.getSubMatrix(0, 0, 6, 1);
  }
}

//  --------------------------  Useful Matrices  -------------------------------  //

//  Selection Matrix for normal forces
Matrix
normalForceSelectionMatrix(const std::list<Contact*> &contacts)
{
  std::list<Matrix> S_list;
  std::list<Contact*>::const_iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    Matrix S(Matrix::ZEROES<Matrix>(1, (*it)->numFrictionEdges+1));
    S.elem(0,0) = 1.0;
    S_list.push_back(S);
  }

  return Matrix::BLOCKDIAG<Matrix>(S_list);
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

//  Summation matrix for tangential contact displacements
Matrix
tangentialDisplacementSummationMatrix(const std::list<Contact*> &contacts)
{
  std::list<Matrix> sigma_list;
  std::list<Contact*>::const_iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    Matrix sigma(1, (*it)->numFrictionEdges+1);
    sigma.elem(0,0) = 0;
    for (int i=0; i<(*it)->numFrictionEdges; i++) {
      int j = (i<(*it)->numFrictionEdges-1) ? i+1 : 0;
      double num  = (*it)->frictionEdges[6*i]   * (*it)->frictionEdges[6*j];
             num += (*it)->frictionEdges[6*i+1] * (*it)->frictionEdges[6*j+1];
      double len1 = sqrt(pow((*it)->frictionEdges[6*i], 2) + pow((*it)->frictionEdges[6*i+1], 2));
      double len2 = sqrt(pow((*it)->frictionEdges[6*j], 2) + pow((*it)->frictionEdges[6*j+1], 2));
      double angle = acos(num/(len1*len2));
      if (angle > Matrix::EPS) {
        sigma.elem(0, i+1) = len1 * cos(angle / 2.0);
        sigma.elem(0, j+1) = len2 * cos(angle / 2.0);
      }
    }
    sigma_list.push_back(sigma);
  }

  return Matrix::BLOCKDIAG<Matrix>(sigma_list);
}

Matrix
frictionConeEdgeMatrix(const std::list<Contact*> &contacts) {
  std::list<Matrix*> blocks;
  std::list<Contact*>::const_iterator it;
  for (it=contacts.begin(); it!=contacts.end(); it++) {
    Matrix *Gi = new Matrix(Matrix::ZEROES<Matrix>(1, (*it)->numFrictionEdges + 1));
    Gi->elem(0,0) = -1.0 * (*it)->getCof();
    for (int i=0; i<(*it)->numFrictionEdges; i++) {
      double len = sqrt(pow((*it)->frictionEdges[6*i], 2) + pow((*it)->frictionEdges[6*i+1], 2));
      Gi->elem(0,i+1) = len;
    }
    blocks.push_back(Gi);
  }
  Matrix G(Matrix::BLOCKDIAG<Matrix>(&blocks));
  while (!blocks.empty()) {
    delete blocks.back();
    blocks.pop_back();
  }
  return G;
}

double
angleBetweenEdges(const Matrix edge1, const Matrix edge2) {
  double num  = edge1.elem(0,0) * edge2.elem(0,0) + edge1.elem(1,0) * edge2.elem(1,0);
  double len1 = sqrt(pow(edge1.elem(0,0), 2) + pow(edge1.elem(1,0), 2));
  double len2 = sqrt(pow(edge2.elem(0,0), 2) + pow(edge2.elem(1,0), 2));
  double val = num/(len1*len2);
  val = (val >  1.0) ?  1.0 : val;
  val = (val < -1.0) ? -1.0 : val;
  return acos(val);
}

bool
checkFrictionEdges(const std::list<Matrix> &frictionEdges) {
  std::list<Matrix>::const_iterator edge1 = frictionEdges.end();
  std::list<Matrix>::const_iterator edge2 = frictionEdges.begin();
  edge1--;

  int counter = 0;
  for (int i=0; i<=frictionEdges.size(); i++) {
    double angle = angleBetweenEdges(*edge1, *edge2);
    if (angle < Matrix::EPS) {
      if (++counter>1) {
        DBGA("Three consecutive collinear edges");
        return false;
      }
      if (matrixAdd(*edge1, (*edge2).negative()).fnorm() < Matrix::EPS) {
        DBGA("Adjacent equal edges");
        return false;
      }
    } else {
      counter = 0;
      double len1 = (*edge1).fnorm();
      double len2 = (*edge2).fnorm();
      if (len1 - len2 > Matrix::EPS) {
        DBGA("Unequal friction edges defining sector");
        return false;
      }
      /*if (len1 - 1.0/cos(angle / 2.0) > Matrix::EPS) {
        DBGA("Friction edge has incorrect length");
        return false;
      }*/
    }
    edge1 = edge2++;
    if (edge2 == frictionEdges.end()) edge2 = frictionEdges.begin();
  }
  return true;
}

void writeResultsToFile(SolutionStruct &S, const std::list<Contact*> &contacts) {
  std::ofstream file;
  file.open("./log/solution.log", std::ios_base::app);
  file << "######## -------- New Problem -------- ########" << std::endl;
  std::map<std::string, int>::const_iterator it;
  for (it=S.var.begin(); it!=S.var.end(); it++) {
    file << it->first << std::endl;
    file << S.sol.getSubMatrixBlockIndices(it->second, 0) << std::endl;
  }

  Matrix beta(S.sol.getSubMatrixBlockIndices(S.var["beta"], 0));
  Matrix D(Contact::frictionForceBlockMatrix(contacts));
  file << "contact forces:" << std::endl;
  file << matrixMultiply(D, beta);

  file << std::endl;
  file.close();
}
