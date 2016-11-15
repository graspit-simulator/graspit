#include "cgal_qp.h"

#include "math/matrix.h"
//#define GRASPITDBG
#include "debug.h"

//CGAL includes
#include <iostream>
#include <cassert>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

/*! Wrapper for CGAL function to solve a non-negative quadratic program of the form:
	minimize solT * Q * sol subject to:
	Q symmetric and positive semidefinite
	sol >= 0
	Eq * sol = b
	InEq * sol <= ib
*/
int cgalNNQPSolverWrapper(const Matrix &Q, const Matrix &Eq, const Matrix &b,
						const Matrix &InEq, const Matrix &ib, Matrix &sol)
{
	DBGA("CGAL QP Wrapper");
	// by default, we have a nonnegative QP with Ax <= b
	Program qp (CGAL::SMALLER, true, 0, false, 0); 
  
	//set the equality constraints
	for (int i=0; i<Eq.rows(); i++) {
		for(int j=0; j<Eq.cols(); j++) {
			qp.set_a(j, i, Eq.elem(i,j));
		}
		qp.set_b(i, b.elem(i,0));
		qp.set_r(i, CGAL::EQUAL);
	}	
	//set the inequality constraints
	for (int i=0; i<InEq.rows(); i++) {
		int eqi = i + Eq.rows();
		for (int j=0; j<InEq.cols(); j++) {
			qp.set_a(j, eqi, InEq.elem(i,j));
		}
		qp.set_b(eqi, ib.elem(i,0));
		//CGAL::SMALLER is default
	}	
	//set the symmetric Q matrix; only upper triangle is needed
	//remember to double the value
	for (int i=0; i<Q.rows(); i++) {
		for (int j=0; j<=i; j++) {
			qp.set_d(i, j, 2.0*Q.elem(i,j));
		}
	}
	//c and c0 should be 0 by default

	// solve the program, using ET as the exact type
	Solution s = CGAL::solve_nonnegative_quadratic_program(qp, ET());
	assert (s.solves_nonnegative_quadratic_program(qp));
 
	// output solution
	std::cout << s; 
	return 1;
}