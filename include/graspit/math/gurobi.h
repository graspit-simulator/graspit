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
// Author(s): Maximilian K. Haas
//
// $Id: gurobi.h,v 1.6 2016/02/17 21:48:55 m.haas Exp $
//
//######################################################################

#include <list>
#include <vector>

/*! \file
	A wrapper for using the Gurobi solver from within GraspIt! Currently
	only implemented for a specific type of binary optimization problem.
*/
class Matrix;

int gurobiSolverWrapper(const Matrix &Q, const Matrix &c, 
						const Matrix &Eq, const Matrix &b,
						const Matrix &InEq, const Matrix &ib,
						std::list<Matrix> &QInEq, std::list<Matrix> &iq, std::list<Matrix> &qib,
                        std::vector<int> SOS_index, std::vector<int> SOS_len, std::vector<int> SOS_type,
						const Matrix &lowerBounds, const Matrix &upperBounds,
						Matrix &sol, const Matrix &types, double *objVal);
