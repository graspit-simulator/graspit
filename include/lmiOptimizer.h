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
// Author(s): Andrew T. Miller
//
// $Id: lmiOptimizer.h,v 1.1 2009/07/21 19:30:27 cmatei Exp $
//
//######################################################################

#include <stdio.h>

class Grasp;

//! Encapsulates the Grasp Force Optimization (GFO) code based on Linear Matrix Inequalitties (LMI)
/*! This class contains the first iteration of the GFO code from GraspIt
	release 0.9. This code is based on the linear matrix inequality 
	technique and was adapted from code written by Li Han and Jeff Trinkle.  
	More information regarding their algorithm can be found in: L. Han, 
	J. Trinkle, and Z. Li, "Grasp Analysis as Linear Matrix Inequality 
	Problems," \e IEEE \e Transactions \e on \e Robotics \e and \e 
	Automation, Vol. 16, No. 6, pp. 663--674, December, 2000. 
	
	Note that this functionality is complete, but it has not been throroughly 
	used or tested. Is is also not completely documented, so you will have to 
	read the code and figure it out. Unfortunately, it has fallen into disrepair
	and has not been kept up-to-date with the rest of GraspIt. As such, it is
	not used anywhere else, and probably will not work out of the box.

	We also have a new way of doing GFO based on Quadratic Programming (QP). That 
	functionality is included with the Grasp class. It is up to date and tested.
	We recommend to use that version for your GFO code. The LMI version is 
	probably superior from the mathematical standpoint, but the QP version is
	easier to follow, completely documented and up to date. A good starting
	point is the function Grasp::computeQuasistaticForcesAndTorques(...)

	Note that this file and this version of the GFO code might be removed from
	future releases of GraspIt.
*/
class LMIOptimizer
{
private:
   //! GFO parameter
  static double GFO_WEIGHT_FACTOR;

  //! The grasp that the computations are performed for
  Grasp *mGrasp;

  //! Number of basis wrenches for grasp map (3 * numContacts for PCWF)
  int numWrenches; 

  //! A \a (6 * numWrenches) matrix containing basis wrenches for each contact vertex (stored in column-major format)
  double *graspMap; 

  //! Dimension of the null space of the grasp map
  int nullDim;

  //! The null space of the grasp map; (column-major)
  double *nullSpace;

  //! vector of size \a (numWrenches+1), the optimal grasp contact forces, computed from optmz0 and the objective value.
  double *optmx0;   

  //! vector of size \a numDOF storing the optimal torque for each DOF
  double *optTorques; 

  //! The grasp jacobian
  double *Jacobian;

  //! Min norm solution to \f$\mbox{GraspMap} * x = F_{ext}\f$
  double *minNormSln;

  //! The number of degrees of freedom of the hand
  int numDOF;

  //! temporary: should come from hand object
  double *externalTorques; 
  
  //! These variables are used by maxdet package.  Refer to maxdet manual for their explainations.
  int L, K, GPkRow, *F_blkszs,*G_blkszs;                 

  //! These variables are used by maxdet package.  Refer to maxdet manual for their explainations.
  double *F, *G, *Z, *W, *c, constOffset;

  //! Termination criteria used in maxdet algorithm. Refer to maxdet manual.
  double gamma, abstol, reltol; 

  //! If it is 1, then terminate the force feasibility phase whenever the objective value becomes negative, i.e. find one valid grasp force. Otherwise, use the regular maxdet termination.
  int negativeFlag;

  //! Is the grasp force optimization feasible?
  int feasible;

  //! On entry, the maximum number of total Newton iteraions, and on exit, the real number of Newton Iterations in the feasible phase.
  int feasNTiters;

  //! Used when saving output from grasp force optimization analysis
  int graspCounter;

  //! Vector of size \a m, the \a z value corrsponding to the initial feasible grasp forces computed in force feasibility phase.
  double *initz0;

  //! The counterpart to feasNTiters
  int optmNTiters;

  //! Vector of size \a m, the \a z value corresponding to optimal grasp forces, computed in force optimization phase.
  double *optmz0;

  //! Vector of size \a (m+3), \a optmz0, as well as the corresponding objective value, duality gap, and number of optimization iteration steps.
  double *extendOptmz0;

  //! array of dimension \a (m+3)*Number_of_Iterations_at_the_Feasibility_Phase, the history (iterative values) of z, objective value, duality gap and iteration number in force feasibility phase at each simulation step.
  double *feasZHistory; 

 // array of dimension \a (m+3)*Number_of_Iterations_at_the_Optimization_Phase, the counterpart to feasZHistory in the optimization phase.
  double *optmZHistory;

  //! array of dimension \a (m+3)*(Number_of_Iterations_at_Optimization_Phase+1), optmZHistory, + 1: the initial feasible grasp force, its objective value, duality gap and number of iterations 
  double *extendOptmZHistory; 

 //! array of dimension \a (NumWrenches+1)*Number_of_Iterations_at_the_Feasibility_Phase, the history of grasp forces x and objective value in the force feasibility phase at the last simulation step.
  double *feasXHistory;

 //! array of dimension \a (NumWrenches+1)*(Number_of_Iterations_at_the_Optimization_Phase+1), the counterpart to feasXHistory in the optimization phase at the last step.
  double *optmXHistory;

  //! Output file pointer for saving the results of a GFO analysis
  FILE *pRstFile;

  //! GFO routine
  void minimalNorm();
  //! GFO routine
  void computeNullSpace();

  //! Constructs a version of the hand jacobian 
  void buildJacobian();

  //! Builds the grasp map computing net object wrench from contact wrenches
  void buildGraspMap();

  //! Friction LMI helper function
  void lmiFL(double *lmi,int rowInit, int colInit, int totalRow);
  //! Friction LMI helper function
  void lmiPCWF(double cof, double *lmi,int rowInit, int colInit, int totalRow);
  //! Friction LMI helper function
  void lmiSFCE(double cof, double cof_t,double *lmi,int rowInit, int colInit,
	       int totalRow);
  //! Friction LMI helper function
  void lmiSFCL(double cof, double cof_t,double *lmi,int rowInit, int colInit,
	       int totalRow);
  //! Friction LMI helper function
  double *lmiTorqueLimits();
  //! Friction LMI helper function
  void lmiFL();
  //! Friction LMI helper function
  void lmiPWCF();
  //! Friction LMI helper function
  void lmiSFCE();
  //! Friction LMI helper function
  void lmiSFCL();
  //! Friction LMI helper function
  double *lmiFrictionCones();

  //! GFO routine
  double *weightVec();
  //! GFO routine
  void feasibilityAnalysis();
  //! GFO routine
  void optm_EffortBarrier();
  //! GFO routine
  void computeObjectives();
  //! GFO routine
  double *xzHistoryTransfrom(double *zHistory,int numIters);

public:
  LMIOptimizer(Grasp *g) : mGrasp(g) {}

  //! Main GFO routine
  int  findOptimalGraspForce();

};