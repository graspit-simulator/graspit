//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
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
