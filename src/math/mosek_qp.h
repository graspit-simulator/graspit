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
// $Id: mosek_qp.h,v 1.6 2009/06/30 16:45:19 cmatei Exp $
//
//######################################################################

/*! \file
	A wrapper for using the Mosek commercial QP solver from within GraspIt!
*/
class Matrix;

enum MosekObjectiveType{MOSEK_OBJ_QP, MOSEK_OBJ_LP};

int mosekNNSolverWrapper(const Matrix &Q, 
						 const Matrix &Eq, const Matrix &b,
						 const Matrix &InEq, const Matrix &ib,
						 const Matrix &lowerBounds, const Matrix &upperBounds,
						 Matrix &sol, double *objVal, MosekObjectiveType objType);
