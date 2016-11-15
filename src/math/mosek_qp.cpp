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
// $Id: mosek_qp.cpp,v 1.11 2010/03/05 00:22:54 cmatei Exp $
//
//######################################################################

/*! \file
	A wrapper for using the Mosek commercial QP solver from within GraspIt!
*/

#include "mosek_qp.h"

#include <limits>
#include <math.h>

#include "matrix.h"
//#define GRASPITDBG
#include "debug.h"

#include "mosek.h"

static void MSKAPI printstr(void*, char str[])
{
	DBGP(str);
}

/*! Creates a Mosek environment singleton and deletes it when the instance
	of this goes out of scope (presumably at the end of the program */
class MosekEnvAutoPtr {
public:
	MSKenv_t     env;
	MosekEnvAutoPtr() {
		MSKrescodee r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
		if ( r!=MSK_RES_OK ) {
			DBGA("Failed to create Mosek environment");
			assert(0);
		}
		MSK_linkfunctoenvstream(env,MSK_STREAM_LOG,NULL,printstr);
		r = MSK_initenv(env);
		if ( r!=MSK_RES_OK ) {
			DBGA("Failed to initialize Mosek environment");
			assert(0);
		}
	}
	~MosekEnvAutoPtr() {
		DBGA("Mosek environment cleaned up");
		MSK_deleteenv(&env);
	}
};

MSKenv_t& getMosekEnv(){
	//the one and only instance of the Mosek environment
	static MosekEnvAutoPtr mskEnvPtr;
	return mskEnvPtr.env;
}

int mosekNNSolverWrapper(const Matrix &Q, const Matrix &Eq, const Matrix &b,
						 const Matrix &InEq, const Matrix &ib, 
						 const Matrix &lowerBounds, const Matrix &upperBounds,
						 Matrix &sol, double *objVal, MosekObjectiveType objType)
{
	DBGP("Mosek QP Wrapper started");
	MSKrescodee  r;
	MSKtask_t task = NULL;

	// Get the only instance of the mosek environment.
	MSKenv_t     env  = getMosekEnv();
    // Create the optimization task.
    r = MSK_maketask(env, 0, 0, &task);
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to create optimization task");
		return -1;
	}
    MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
	
	//---------------------------------------
	//start inputing the problem
	//prespecify number of variables to make inputting faster
	r = MSK_putmaxnumvar(task, sol.rows());
	//number of constraints (both equality and inequality)
	if (r == MSK_RES_OK) {
		r = MSK_putmaxnumcon(task, Eq.rows() + InEq.rows());
	}
	//make sure default value is 0 for sparse matrices
	assert(Q.getDefault() == 0.0);
	assert(Eq.getDefault() == 0.0);
	assert(InEq.getDefault() == 0.0);
	//number of non-zero entries in A
	if (r == MSK_RES_OK) {
		r = MSK_putmaxnumanz(task, Eq.numElements() + InEq.numElements() );
	}
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to input variables");
		MSK_deletetask(&task);
		return -1;
	}

	//solver is sensitive to numerical problems. Scale the problem down
	//we will use this value to scale down the right hand side of equality
	//and inequality constraints and lower and upper bounds
	//after solving, we must scale back up the solution and the value of the
	//objective
	double scale = b.absMax();
	if (scale < 1.0e2) {
		scale = 1.0;
	} else {
		DBGP("Mosek solver: scaling problem down by " << scale);
	}

	//---------------------------------------
	//insert the actual variables and constraints

	//append the variables
	MSK_append(task, MSK_ACC_VAR,sol.rows());
	//append the constraints. 
	MSK_append(task, MSK_ACC_CON, Eq.rows() + InEq.rows());

	int i,j;
	double value;
	if (objType == MOSEK_OBJ_QP) {
		//quadratic optimization objective
		//the quadratic term
		Q.sequentialReset();
		while (Q.nextSequentialElement(i, j, value)) {
			MSK_putqobjij(task, i, j, 2.0*value);
		}
	} else if (objType == MOSEK_OBJ_LP) {
		//linear objective
		for (j=0; j<Q.cols(); j++) {
			if ( fabs(Q.elem(0,j))>1.0e-5) {
				MSK_putcj(task, j, Q.elem(0,j));
			}
		}
	} else {
		assert(0);
	}

	//variable bounds
	assert(sol.rows() == lowerBounds.rows());
	assert(sol.rows() == upperBounds.rows());
	for (i=0; i<sol.rows(); i++) {
		if ( lowerBounds.elem(i,0) >= upperBounds.elem(i,0) ) {
			if ( lowerBounds.elem(i,0) > upperBounds.elem(i,0) ) {
				assert(0);
			}
			if (lowerBounds.elem(i,0) == -std::numeric_limits<double>::max()) {
				assert(0);
			}
			if (upperBounds.elem(i,0) == std::numeric_limits<double>::max()) {
				assert(0);
			}
			//fixed variable
			DBGP(i << ": fixed " << lowerBounds.elem(i,0)/scale);
			MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FX, 
						 lowerBounds.elem(i,0)/scale, upperBounds.elem(i,0)/scale );
		} else if ( lowerBounds.elem(i,0) != -std::numeric_limits<double>::max() ) {
			//finite lower bound
			if ( upperBounds.elem(i,0) != std::numeric_limits<double>::max() ) {
				//two finite bounds
				DBGP(i << ": finite bounds " << lowerBounds.elem(i,0)/scale 
					   << " " << upperBounds.elem(i,0)/scale);
				MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_RA, 
							 lowerBounds.elem(i,0)/scale, upperBounds.elem(i,0)/scale );
			} else {
				//lower bound
				DBGP(i << ": lower bound " << lowerBounds.elem(i,0)/scale);
				MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_LO, 
							 lowerBounds.elem(i,0)/scale, +MSK_INFINITY );

			}
		} else {
			//infinite lower bound
			if ( upperBounds.elem(i,0) != std::numeric_limits<double>::max() ) {
				//upper bound
				DBGP(i << ": upper bound " << upperBounds.elem(i,0)/scale);
				MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_UP, 
							 -MSK_INFINITY, upperBounds.elem(i,0)/scale );
			} else {
				//unbounded
				DBGP(i << ": unbounded");
				MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FR, 
							 -MSK_INFINITY, +MSK_INFINITY );

			}
		}
	}

	//constraints and constraint bounds
	//equality constraints
	Eq.sequentialReset();
	while(Eq.nextSequentialElement(i, j, value)) {
		MSK_putaij( task, i, j, value );
	}
	for (i=0; i<Eq.rows(); i++) {
		MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_FX, b.elem(i,0)/scale, b.elem(i,0)/scale);
	}
	//inequality constraints, <=
	InEq.sequentialReset();
	while(InEq.nextSequentialElement(i, j, value)) {
		int eqi = i + Eq.rows();
		MSK_putaij( task, eqi, j, value );
	}
	for (i=0; i<InEq.rows(); i++) {
		int eqi = i + Eq.rows();
		MSK_putbound(task, MSK_ACC_CON, eqi, MSK_BK_UP, -MSK_INFINITY, ib.elem(i,0)/scale);
	}
	//specify objective: minimize
	MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

	//give it 800 iterations, twice the default. 
	MSK_putintparam(task,MSK_IPAR_INTPNT_MAX_ITERATIONS,800);

	//----------------------------------

	//solve the thing
	DBGP("Optimization started");
	r = MSK_optimize(task);
	DBGP("Optimization returns");
	
	//write problem to file
	/*
	static int fileNum = 0;
	if (r != MSK_RES_OK) {
		char filename[50];
		sprintf(filename,"mosek_error_%d_%d.opf",fileNum++, r);
		MSK_writedata(task, filename);
		FILE *fp = fopen(filename,"a");
		fprintf(fp,"\n\nEquality matrix:\n");
		Eq.print(fp);
		fclose(fp);
	}
	*/

	if (r != MSK_RES_OK) {
		DBGA("Mosek optimization call failed, error code " << r);
		MSK_deletetask(&task);
		return -1;
	}
	DBGP("Optimization complete");
	//debug code, find out number of iterations used
	//int iter;
	//MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter); 
	//DBGA("Iterations used: " << iter);

	//find out what kind of solution we have
	MSKprostae pst;
	MSKsolstae sst;
	MSK_getsolutionstatus(task, MSK_SOL_ITR, &pst, &sst);
	int result;
	if (sst == MSK_SOL_STA_OPTIMAL || sst == MSK_SOL_STA_NEAR_OPTIMAL) {
		//success, we have an optimal problem
		if (sst == MSK_SOL_STA_OPTIMAL) {DBGP("QP solution is optimal");}
		else {DBGA("QP solution is *nearly* optimal");}
		result = 0;
	} else if (sst == MSK_SOL_STA_PRIM_INFEAS_CER) {
		//unfeasible problem
		DBGP("Mosek optimization: primal infeasible");
		result = 1;
	} else if (sst == MSK_SOL_STA_DUAL_INFEAS_CER) {
		//unfeasible problem
		DBGA("Mosek optimization: dual infeasible (primal unbounded?)");
		result = 1;
	} else if (sst == MSK_SOL_STA_PRIM_AND_DUAL_FEAS) {
		//i think this means feasible problem, but unbounded solution
		//this shouldn't happen as our Q is positive semidefinite
		DBGA("QP solution is prim and dual feasible, but not optimal");
		DBGA("Is Q positive semidefinite?");
		result = -1;
	} else {
		//unknown return status
		DBGA("QP fails with solution status " << sst << " and problem status " << pst);
		result = -1;
	}

	//MSK_SOL_STA_DUAL_FEAS;

	//retrieve the solutions
	if (!result) {
		//get the value of the objective function
		MSKrealt obj, foo;
		MSK_getsolutioninf(task, MSK_SOL_ITR, &pst, &sst, &obj,
						   &foo, &foo, &foo, &foo, &foo, &foo, &foo, &foo);
		if (objType == MOSEK_OBJ_QP) {
			*objVal = obj * scale * scale;
		} else if (objType == MOSEK_OBJ_LP) {
			*objVal = obj * scale;
		} else {
			assert(0);
		}
		double* xx = new double[sol.rows()];
		MSK_getsolutionslice(task, MSK_SOL_ITR, MSK_SOL_ITEM_XX,
							 0, sol.rows(), xx);
		for (i=0; i<sol.rows(); i++) {
			sol.elem(i,0) = scale * xx[i];
			DBGP("x" << i << ": " << xx[i]);
		}
		delete [] xx;	
	}
	MSK_deletetask(&task);
	return result;
}
