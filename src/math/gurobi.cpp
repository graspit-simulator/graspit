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
// $Id: gurobi.h,v 1.6 2016/02/17 21:49:05 m.haas Exp $
//
//######################################################################

/*! \file
  A wrapper for using the Gurobi solver from within GraspIt! Currently
  only implemented for a specific type of binary optimization problem.
*/

#include "graspit/math/gurobi.h"

#include <math.h>
#include <limits>
#include <vector>
#include <fstream>

#include "graspit/math/matrix.h"
//#define GRASPITDBG
#include "graspit/debug.h"

#include "gurobi_c++.h"

using namespace std;

/*! Creates a Gurobi environment singleton if it does not exist already 
  and returns it */
GRBEnv& getGurobiEnv(){
  //the one and only instance of the gurobi environment
  static GRBEnv env = GRBEnv();
  return env;
}

int gurobiSolverWrapper(const Matrix &Q, const Matrix &c, 
  const Matrix &Eq, const Matrix &b,
  const Matrix &InEq, const Matrix &ib,
  const std::list<Matrix> &QInEq, const std::list<Matrix> &iq, const std::list<Matrix> &qib,
  const std::list<Matrix> &indic_lhs, const std::list<Matrix> &indic_rhs, const std::list<int> &indic_var, const std::list<int> &indic_val, const std::list<std::string> &indic_sense,
  const std::list<int> &SOS_index, const std::list<int> &SOS_len, const std::list<int> &SOS_type,
  const Matrix &lowerBounds, const Matrix &upperBounds,
  Matrix &sol, const Matrix &types, double *objVal)
{
  try {
    DBGP("Gurobi Wrapper started");

    // Get the only instance of the gurobi environment
    GRBEnv env = getGurobiEnv();
    GRBModel model = GRBModel(env);

    model.getEnv().set("LogToConsole", "0");
    model.getEnv().set("NumericFocus", "3");
    model.getEnv().set("IntFeasTol", "1e-9");
    model.getEnv().set("FeasibilityTol", "1e-9");
    //model.getEnv().set("PreSOS2BigM", "-1");
    model.getEnv().set("TimeLimit", "600");

    if (b.rows())
      if (b.absMax() > 1e2)
        DBGA("The problem given to Gurobi is badly scaled (" << b.absMax() << "). The results may be inaccurate.");

    //---------------------------------------
    //convert bounds and variable types into format that can be handed to gurobi
    std::vector<double> lb(lowerBounds.rows());
    std::vector<double> ub(upperBounds.rows());
    std::vector<char> vtype(types.rows());

    for (int i=0; i<sol.rows(); i++) {
      // lower bounds
      if (lowerBounds.elem(i,0) == -std::numeric_limits<double>::max())
        lb[i] = -GRB_INFINITY;
      else
        lb[i] = lowerBounds.elem(i,0);
      // upper bounds
      if (upperBounds.elem(i,0) == std::numeric_limits<double>::max())
        ub[i] = GRB_INFINITY;
      else
        ub[i] = upperBounds.elem(i,0);
      // variable types
      if (types.elem(i,0) == 0)
        vtype[i] = GRB_CONTINUOUS;
      else if (types.elem(i,0) == 1)
        vtype[i] = GRB_BINARY;
      else if (types.elem(i,0) == 2)
        vtype[i] = GRB_INTEGER;
      else if (types.elem(i,0) == 3)
        vtype[i] = GRB_SEMICONT;
      else if (types.elem(i,0) == 4)
        vtype[i] = GRB_SEMIINT;
      else {
        DBGA("requested unknown variable type");
        return -1;
      }
    }

    //---------------------------------------
    // Start inputing the problem

    // Add variables to the model
    GRBVar *vars = model.addVars(&lb[0], &ub[0], NULL, &vtype[0], NULL, sol.rows());

    // Add objective function
    // quadratic objective
    if (Q.rows()) {
      GRBQuadExpr obj = 0;
      for (int i=0; i<Q.rows(); i++) {
        for (int j=0; j<Q.cols(); j++)
          obj += Q.elem(i,j) * vars[i] * vars[j];
        if (c.rows())
          obj += c.elem(0,i) * vars[i];
      }
      model.setObjective(obj, GRB_MINIMIZE);
    }
    // linear objective
    else {
      GRBLinExpr obj = 0;
      for (int j=0; j<c.cols(); j++)
        obj += c.elem(0,j) * vars[j];
      model.setObjective(obj, GRB_MINIMIZE);
    }

    // Add equality constraints to the model
    // linear equality constraints
    if (Eq.rows()) {
      for (int i=0; i<Eq.rows(); i++) {
        GRBLinExpr constr = 0;
        for (int j=0; j<Eq.cols(); j++)
          if (Eq.elem(i,j) != 0)
            constr += Eq.elem(i,j) * vars[j];
        model.addConstr(constr, GRB_EQUAL, b.elem(i,0));
      }
    }

    // Add inequality constraints to the model
    // linear inequality constraints
    if (InEq.rows()) {
      for (int i=0; i<InEq.rows(); i++) {
        GRBLinExpr constr = 0;
        for (int j=0; j<InEq.cols(); j++)
          if (InEq.elem(i,j) != 0)
            constr += InEq.elem(i,j) * vars[j];
        model.addConstr(constr, GRB_LESS_EQUAL, ib.elem(i,0));
      }
    }
    // quadratic inequality constraint
    if (!QInEq.empty()) {
      std::list<Matrix>::const_iterator Q_it = QInEq.begin();
      std::list<Matrix>::const_iterator q_it = iq.begin();
      std::list<Matrix>::const_iterator b_it = qib.begin();
      for( ; Q_it!=QInEq.end(); Q_it++, q_it++, b_it++) {
        GRBQuadExpr constr = 0;
        for (int i=0; i<sol.rows(); i++) {
          for (int j=0; j<sol.rows(); j++)
            if ((*Q_it).elem(i,j) != 0)
              constr += (*Q_it).elem(i,j) * vars[i] * vars[j];
          if ((*q_it).rows())
            if ((*q_it).elem(0,i) != 0)
              constr += (*q_it).elem(0,i) * vars[i];
        }
        model.addQConstr(constr, GRB_LESS_EQUAL, (*b_it).elem(0,0));            
      }
    }

    // Add indicator constraints
    std::list<Matrix>::const_iterator lhs = indic_lhs.begin();
    std::list<Matrix>::const_iterator rhs = indic_rhs.begin();
    std::list<int>::const_iterator var = indic_var.begin();
    std::list<int>::const_iterator val = indic_val.begin();
    std::list<std::string>::const_iterator sen = indic_sense.begin();
    for ( ; lhs!=indic_lhs.end(); lhs++, rhs++, var++, val++, sen++) {
      char sense;
      if (*sen == "leq") sense = GRB_LESS_EQUAL;
      else if (*sen == "eq") sense = GRB_EQUAL;
      else if (*sen == "geq") sense = GRB_GREATER_EQUAL;
      else {
        DBGA("Requested unknown sense in indicator constraint");
        return -1;
      }
      for (int i=0; i<(*lhs).rows(); i++) {
        GRBLinExpr constr = 0;
        for (int j=0; j<(*lhs).cols(); j++) {
          constr += (*lhs).elem(i,j) * vars[j];
        }
        model.addGenConstrIndicator(vars[*var], *val, constr, sense, (*rhs).elem(i,0));
      }
    }

    // SOS constraints
    std::list<int>::const_iterator SOS_index_it = SOS_index.begin();
    std::list<int>::const_iterator SOS_len_it = SOS_len.begin();
    std::list<int>::const_iterator SOS_type_it = SOS_type.begin();
    for ( ; SOS_index_it!=SOS_index.end(); SOS_index_it++, SOS_len_it++, SOS_type_it++) {
      double weights[*SOS_len_it];
      // Weights for SOS variables should be unique
      for (int j=0; j<*SOS_len_it; j++) weights[j] = j;
      if (*SOS_type_it == 1)
        model.addSOS(vars+(*SOS_index_it), weights, *SOS_len_it, GRB_SOS_TYPE1);
      else if (*SOS_type_it == 2)
        model.addSOS(vars+(*SOS_index_it), weights, *SOS_len_it, GRB_SOS_TYPE2);
      else {
        DBGA("Requested unknown SOS constraint");
        return -1;
      }
    }

    // Optimize
    model.update();
    model.optimize();

    // Retrieve solution
    // Gurobi found an optimal solution
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      DBGP("Solution is optimal");
      *objVal = model.get(GRB_DoubleAttr_ObjVal);
      for (int i=0; i<sol.rows(); i++)
        sol.elem(i,0) = vars[i].get(GRB_DoubleAttr_X);
      return 0;
    }

    // Gurobi returned with a different status code
    if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
      // Model was proven to be infeasible
      DBGP("Problem infeasible");
      return 1;
    } else if (model.get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD) {
      // Model was proven to be either infeasible or unbounded
      DBGA("Problem is either infeasible or unbounded");
      return 1;
    } else if (model.get(GRB_IntAttr_Status) == GRB_UNBOUNDED) {
      // Model was proven to be unbounded
      DBGA("Problem is unbounded");
      return -1;
    } else if (model.get(GRB_IntAttr_Status) == GRB_NUMERIC) {
      // Optimization was terminated due to unrecoverable nomerical difficulties
      DBGA("Problem was abandoned due to numerical issues");
      return -1;
    } else if (model.get(GRB_IntAttr_Status) == GRB_SUBOPTIMAL) {
      // Unable to satisfy optimality tolerances. A suboptimal solution is available
      DBGA("Found suboptimal solution");
      return 1;
    } else if (model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
      // Optimization terminated because the time expended exceeded the limit
      DBGA("Time limit exceeded");
      return 1;
    }
  } catch (GRBException e) {
    DBGA("Error Code = " << e.getErrorCode());
    DBGA(e.getMessage());
  } catch (...) {
    DBGA("Exception during gurobi optimization");
  }
  return -1;
}

int gurobiSolverWrapper(const Matrix &Q, const Matrix &c, 
      const Matrix &Eq, const Matrix &b,
      const Matrix &InEq, const Matrix &ib,
      const std::list<Matrix> &QInEq, const std::list<Matrix> &iq, const std::list<Matrix> &qib,
      const Matrix &lowerBounds, const Matrix &upperBounds,
      Matrix &sol, double *objVal)
{
  std::list<Matrix> indic_lhs;
  std::list<Matrix> indic_rhs;
  std::list<int> indic_var;
  std::list<int> indic_val;
  std::list<std::string> indic_sense;
  std::list<int> SOS_index;
  std::list<int> SOS_len;
  std::list<int> SOS_type;
  Matrix types(Matrix::ZEROES<Matrix>(sol.rows(), 1));
  return gurobiSolverWrapper(Q, c, 
               Eq, b, InEq, ib, QInEq, iq, qib,
               indic_lhs, indic_rhs, indic_var, indic_val, indic_sense,
               SOS_index, SOS_len, SOS_type,
               lowerBounds, upperBounds, sol, types, objVal);
}

int gurobiSolverWrapper(const Matrix &Q, const Matrix &c, 
            const Matrix &Eq, const Matrix &b,
            const Matrix &InEq, const Matrix &ib,
            const Matrix &lowerBounds, const Matrix &upperBounds,
            Matrix &sol, double *objVal)
{
  std::list<Matrix> QInEq;
  std::list<Matrix> iq;
  std::list<Matrix> qib;
  return gurobiSolverWrapper(Q, c, 
               Eq, b, InEq, ib, QInEq, iq, qib,
               lowerBounds, upperBounds, sol, objVal);
}
