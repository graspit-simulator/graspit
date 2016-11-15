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
// $Id:$
//
//######################################################################

/*! \file
	A wrapper for using the QPOASES solver from within GraspIt!
*/

#include "qpoases.h"

#include <QProblem.hpp>

#include <limits>
#include <math.h>

#include "matrix.h"
//#define GRASPITDBG
#include "debug.h"

/*! Populates an OASES style matrix (dense, row-major) from a GraspIt matrix
  (potentially sparse, column-major).
*/
void oasesFromMatrix(std::vector<double> &O, const Matrix &M, double scale = 1.0)
{
  int i,j;
  double value;
  M.sequentialReset();
  O.resize(0);
  O.resize( M.rows()*M.cols(), 0.0 );
  while (M.nextSequentialElement(i, j, value)) {
    O.at( i*M.cols() + j) = value / scale;
  }
}

void printMatrix(const std::vector<double> &M, std::string name)
{
  std::cerr << name << "[";
  for(size_t i=0; i<M.size(); i++) {
    if (i!=0) std::cerr << " ";
    std::cerr << M[i];
  }
  std::cerr << "]\n";
}

int QPOASESSolverWrapperQP(const Matrix &Q, 
                           const Matrix &Eq, const Matrix &b,
                           const Matrix &InEq, const Matrix &ib,
                           const Matrix &lowerBounds, const Matrix &upperBounds,
                           Matrix &sol, double *objVal)
{
  // number of variables
  int nV = sol.rows();
  // number of constraints 
  int nC = Eq.rows() + InEq.rows();
  // set up QProblem object
  qpOASES::QProblem problem( nV, nC );
  problem.setPrintLevel(qpOASES::PL_LOW);
  
  //solvers are sensitive to numerical problems. Scale the problem down
  //we will use this value to scale down the right hand side of equality
  //and inequality constraints and lower and upper bounds
  //after solving, we must scale back up the solution and the value of the
  //objective
  double scale = b.absMax();
  if (scale < 1.0e2) {
    scale = 1.0;
  } else {
    DBGP("qpOASES solver: scaling problem down by " << scale);
  }
  
  // prepare the quadratic matrix
  std::vector<double> H;
  oasesFromMatrix(H, Q);

  //prepare the constraints matrix, first in Matrix form
  SparseMatrix GA(Matrix::BLOCKCOLUMN<SparseMatrix>(Eq, InEq));
  std::vector<double> A;
  oasesFromMatrix(A, GA);
  //upper bounds
  SparseMatrix GubA(Matrix::BLOCKCOLUMN<SparseMatrix>(b, ib));
  std::vector<double> ubA;
  oasesFromMatrix(ubA, GubA, scale);
  //lower bounds; min inf for inequalities
  SparseMatrix GlbA( Matrix::BLOCKCOLUMN<SparseMatrix>( b, Matrix::MIN_VECTOR(ib.rows()) ) );
  std::vector<double> lbA;
  oasesFromMatrix(lbA, GlbA, scale);
  
  //lower and upper bounds
  std::vector<double> lb;
  lowerBounds.getData(&lb);
  std::vector<double> ub;
  upperBounds.getData(&ub);
  for (size_t i=0; i<lb.size(); i++) {
    if (lb[i] != std::numeric_limits<double>::max() && lb[i] != -std::numeric_limits<double>::max() ) {
      lb[i] /= scale;
    }
    if (ub[i] != std::numeric_limits<double>::max() && ub[i] != -std::numeric_limits<double>::max() ) {
      ub[i] /= scale;
    }
  }

  //g vector, all zeroes
  std::vector<double> g;
  g.resize(sol.rows(), 0.0);

  // solve QP
  int nWSR = 1000;
  qpOASES::returnValue retVal = problem.init( &H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR, 0 );

  if (retVal == qpOASES::RET_MAX_NWSR_REACHED) {
    DBGA("qpOASES QP:max iterations reached");
    return -1;
  } else if (retVal == qpOASES::RET_INIT_FAILED) {
    DBGA("qpOASES QP:init failed with generic RET_INIT_FAILED error code");
    return -1;
  } else if (retVal == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
    DBGA("qpOASES QP:init failed; problem infeasible");
    return 1;
  } else if (retVal == qpOASES::RET_INIT_FAILED_UNBOUNDEDNESS) {
    DBGA("qpOASES QP:init failed; problem unbounded");
    return 1;
  } else if (retVal != qpOASES::SUCCESSFUL_RETURN) {
    DBGA("qpOASES QP:init failed with error code " << retVal);
    return -1;
  }
  DBGP("qpOASES QP:problem reports success");

  // retrieve the solution
  std::vector<double> xOpt;
  xOpt.resize(sol.rows());
  retVal = problem.getPrimalSolution(&xOpt[0]);
  if (retVal != qpOASES::SUCCESSFUL_RETURN) {
    DBGA("qpOASES QP:failed to retrieve solution");
    return -1;
  }
  for (size_t i=0; i<xOpt.size(); i++) {
    sol.elem(i,0) = scale * xOpt[i];
  }
  //retrieve the objective
  *objVal = problem.getObjVal();
  if (*objVal == qpOASES::INFTY) {
    DBGA("qpOASES QP:objective is infinity");
    return -1;
  }
  *objVal *= scale * scale;

  return 0;
}

int QPOASESSolverWrapperLP(const Matrix &Q,
                           const Matrix &Eq, const Matrix &b,
                           const Matrix &InEq, const Matrix &ib,
                           const Matrix &lowerBounds, const Matrix &upperBounds,
                           Matrix &sol, double* objVal)
{
  // number of variables
  int nV = sol.rows();
  // number of constraints 
  int nC = Eq.rows() + InEq.rows();
  // set up QProblem object, inform it of 0 Hessian (since we have an LP)
  qpOASES::QProblem problem( nV, nC, qpOASES::HST_ZERO );
  problem.setPrintLevel(qpOASES::PL_LOW);
  
  //solvers are sensitive to numerical problems. Scale the problem down
  //we will use this value to scale down the right hand side of equality
  //and inequality constraints and lower and upper bounds
  //after solving, we must scale back up the solution and the value of the
  //objective
  double scale = b.absMax();
  if (scale < 1.0e2) {
    scale = 1.0;
  } else {
    DBGP("qpOASES solver: scaling problem down by " << scale);
  }
  
  // prepare the objective matrix
  assert(Q.rows() == 1);
  std::vector<double> g;
  oasesFromMatrix(g, Q);

  //prepare the constraints matrix, first in Matrix form
  SparseMatrix GA(Matrix::BLOCKCOLUMN<SparseMatrix>(Eq, InEq));
  std::vector<double> A;
  oasesFromMatrix(A, GA);
  //upper bounds
  SparseMatrix GubA(Matrix::BLOCKCOLUMN<SparseMatrix>(b, ib));
  std::vector<double> ubA;
  oasesFromMatrix(ubA, GubA, scale);
  //lower bounds; min inf for inequalities
  SparseMatrix GlbA( Matrix::BLOCKCOLUMN<SparseMatrix>( b, Matrix::MIN_VECTOR(ib.rows()) ) );
  std::vector<double> lbA;
  oasesFromMatrix(lbA, GlbA, scale);
  
  //lower and upper bounds
  std::vector<double> lb;
  lowerBounds.getData(&lb);
  std::vector<double> ub;
  upperBounds.getData(&ub);
  for (size_t i=0; i<lb.size(); i++) {
    if (lb[i] != std::numeric_limits<double>::max() && lb[i] != -std::numeric_limits<double>::max() ) {
      lb[i] /= scale;
    }
    if (ub[i] != std::numeric_limits<double>::max() && ub[i] != -std::numeric_limits<double>::max() ) {
      ub[i] /= scale;
    }
  }

  // solve LP
  int nWSR = 1000;
  qpOASES::returnValue retVal = problem.init( NULL, &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR, 0 );

  if (retVal == qpOASES::RET_MAX_NWSR_REACHED) {
    DBGA("qpOASES LP: max iterations reached");
    return -1;
  } else if (retVal == qpOASES::RET_INIT_FAILED) {
    DBGA("qpOASES LP: init failed with generic RET_INIT_FAILED error code");
    return -1;
  } else if (retVal == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
    DBGA("qpOASES LP: init failed; problem infeasible");
    return 1;
  } else if (retVal == qpOASES::RET_INIT_FAILED_UNBOUNDEDNESS) {
    DBGA("qpOASES LP: init failed; problem unbounded");
    return 1;
  } else if (retVal != qpOASES::SUCCESSFUL_RETURN) {
    DBGA("qpOASES LP: init failed with error code " << retVal);
    return -1;
  }
  DBGP("qpOASES LP: problem reports success");

  // retrieve the solution
  std::vector<double> xOpt;
  xOpt.resize(sol.rows());
  retVal = problem.getPrimalSolution(&xOpt[0]);
  if (retVal != qpOASES::SUCCESSFUL_RETURN) {
    DBGA("qpOASES LP: failed to retrieve solution");
    return -1;
  }
  for (size_t i=0; i<xOpt.size(); i++) {
    sol.elem(i,0) = scale * xOpt[i];
  }
  //retrieve the objective
  *objVal = problem.getObjVal();
  if (*objVal == qpOASES::INFTY) {
    DBGA("qpOASES LP: objective is infinity");
    return -1;
  }
  *objVal *= scale;

  return 0;
}
