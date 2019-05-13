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
// Author(s): Maximilian K.G. Haas-Heger
//
// $Id: graspSolver.h,v 1.9 2016/07/25 15:21:24 mkghaas Exp $
//
//######################################################################

// Defines the Grasp Solver class
#ifndef GRASP_SOLVER_H

#include <list>
#include <vector>
#include <string>
#include <fstream>
#include <tr1/functional>

#include "graspit/joint.h"
#include "graspit/grasp.h"
#include "graspit/robot.h"
#include "graspit/robots/humanHand.h"
#include "graspit/contact/contact.h"
#include "graspit/math/matrix.h"

// struct that holds all the elements necessary to completely define a 
// grasp problem
struct GraspStruct {

  // Pointers to objective Matrices
  Matrix Q;
  Matrix cj;

  // lists that contain matrices defining the constraints
  // Linear Equality
  std::list<Matrix> Eq_list;
  std::list<Matrix> b_list;
  // Linear Inequality
  std::list<Matrix> InEq_list;
  std::list<Matrix> ib_list;
  // Quadratic Inequality
  std::list<Matrix> QInEq;
  std::list<Matrix> iq;
  std::list<Matrix> qib;
  // Indicator constraints
  std::list<Matrix> Indic_lhs;
  std::list<Matrix> Indic_rhs;
  std::list<int> Indic_var;
  std::list<int> Indic_val;
  std::list<std::string> Indic_sense;
  // SOS constraints
  std::list<int> SOS_index;
  std::list<int> SOS_len;
  std::list<int> SOS_type;
  // Solution bounds
  std::list<Matrix> lb_list;
  std::list<Matrix> ub_list;

  // Variable types
  std::list<Matrix> types;

  // block indices for unknowns
  std::vector<int> block_cols;
  // map that allows translating variable names to matrix block indices
  std::map<std::string, int> var;
  // list of variable names as stored in the map to retain correct order
  std::list<std::string> varNames;
};

// struct that stores the solution of an optimization problem
struct SolutionStruct {
  // pointer to matrix containing the solution
  Matrix sol;
  // block indices for unknowns
  std::vector<int> block_cols;
  // map that allows translating variable names to matrix block indices
  std::map<std::string, int> var;
};

/*! This class contains the code for the analysis of the passive behavior of articulated
    robotic hands. The constraints governing the grasp are cast as an optimization problem, 
    which can be fed to a solver such as Gurobi.
*/
class GraspSolver {
public:
  GraspSolver(Grasp *grasp);

  // Check if a graps can withstand a given wrench using the refinement solver
  int checkWrenchRefinement(Matrix &preload, const Matrix &wrench, double coneTol = 1.0, double normUnc = 2.5);
  // Find maximum wrench a grasp can withstand along a given direction using the refinement solver
  double findMaximumWrenchRefinement(Matrix &preload, const Matrix &direction, double coneTol = 1.0, double normUnc = 2.5);
  // Find optimal joint preloads that balance given wrench
  Matrix optimalPreloads(Matrix &preload, const Matrix &wrench, double coneTol = 1.0, double normUnc = 2.5);
  // Creates a 2D map of maximum resistible wrenches with the refinement solver
  void create2DMapRefinement(Matrix &preload,
                             const Matrix &direction1,
                             const Matrix &direction2,
                             std::ofstream &file,
                             double coneTol = 1.0,
                             double normUnc = 2.5);

  // Check if a grasp can withstand a given wrench using the iterative solver
  int checkWrenchIterative(Matrix &preload, 
                           const Matrix &wrench,
                           bool single_step, 
                           bool cone_movement,
                           bool tendon=false,
                           bool rigid=false);
  // Find maximum wrench a grasp can withstand along a given direction using the iterative formulation
  double findMaximumWrenchIterative(Matrix &preload, 
                                    const Matrix &direction,
                                    bool single_step, 
                                    bool cone_movement,
                                    bool tendon=false,
                                    bool rigid=false);
  // Creates a 2D map of maximum resistible wrenches with the iterative solver
  void create2DMapIterative(Matrix &preload,
                            const Matrix &direction1,
                            const Matrix &direction2,
                            std::ofstream &file,
                            bool single_step = false, 
                            bool cone_movement = false,
                            bool tendon = false,
                            bool rigid = false);

  static const double kSpringStiffness;

  static const double kBetaMaxBase;
  static const double kTauMaxBase;
  static const double kAlphaMaxBase;
  static const double kQMaxBase;
  static const double kKMaxBase;

  static const double kMaxMotion;

  static const double kVirtualLimitTolerance;

private:
  Grasp *g;

  int numContacts;
  int numJoints;

  double BetaMax;
  double TauMax;
  double AlphaMax;
  double QMax;
  double KMax;

  // contacts and joints in the grasp
  std::list<Contact*> contacts;
  std::list<Joint*> joints;

  // optimization objectives
  // objective that minimizes the energy stored in virtual springs
  void springDeformationObjective(GraspStruct &P);
  // objective that minimizes the resultant wrench
  void resultantWrenchObjective(GraspStruct &P);
  // objective that minimizes how relatively close the worst case variable is to its virtual limit
  void virtualLimitsObjective(GraspStruct &P);
  // objective that minimizes a combination of the resultant wrench and the use of virtual limits
  void resultantAndVLObjective(GraspStruct &P);
  // objective that minimizes the preload torque required to withstand a given wrench
  void preloadTauObjective(GraspStruct &P);

  // constraints that model the grasp
  // contact forces must balance applied wrench
  void objectWrenchConstraint(GraspStruct &P, const Matrix &wrench);
  // one of the variables must have the resultant wrench on the object
  void resultantWrenchConstraint(GraspStruct &P, const Matrix &wrench);
  // movement must be in direction of residual wrench
  void objectMovementConstraint(GraspStruct &P, const Matrix &startingX, const Matrix &directions);
  // contact movement amplitudes must equal relative motion at contact
  void movementAmplitudesConstraint(GraspStruct &P);
  // constraint that makes the joints rigid
  void rigidJointsConstraint(GraspStruct &P);
  // constraint that forces the object to move a certain way (for debug)
  void objectMotionConstraint(GraspStruct &P, const Matrix &wrench);
  // constraint that makes sure resultant (and therefore object wrench) and applied wrench are collinear
  void resultantDirectionConstraint(GraspStruct &P, const Matrix &wrench);
  // normal forces at contacts are determined by the deformation of virtual springs
  void virtualSpringConstraint(GraspStruct &P, const Matrix &beta_p);
  // normal forces at contacts are determined by the deformation of virtual springs (expressed as indicator constraint)
  void virtualSpringIndicatorConstraint(GraspStruct &P, double normUnc);
  // joints may only move at their prescribed preload torque
  void nonBackdrivableJointConstraint(GraspStruct &P, const Matrix &preload, const Matrix &beta_p);
  // joints may only move at their prescribed preload torque (expressed as indicator constraint)
  void nonBackdrivableJointIndicatorConstraint(GraspStruct &P, const Matrix &preload);
  // constraint modeling tendon actuation
  void tendonConstraint(GraspStruct &P, const Matrix &preload, const Matrix &beta_p);
  // constraint forcing frictional forces inside the friction cone
  void frictionConeConstraint(GraspStruct &P, const Matrix &beta_p = Matrix(0,0));
  // constraint forcing frictional forces on the edge of the friction cone
  void frictionConeEdgeConstraint(GraspStruct &P, const Matrix &beta_p);
  // constraint forcing frictional forces on the edge of the friction cone (expressed as indicator constraint)
  void frictionConeEdgeIndicatorConstraint(GraspStruct &P);
  //  constraint that determines if a contact experiences relative motion
  void contactMovementConstraint(GraspStruct &P);
  //  constraint that determines if a contact experiences relative motion (expressed as indicator constraint)
  void contactMovementIndicatorConstraint(GraspStruct &P);
  // a SOS2 constraint to ensure friction opposes motion
  void amplitudesSOS2Constraint(GraspStruct &P, const Matrix &beta_p = Matrix(0,0));
  // constraint that keeps a variable v lower bounded by the largest preload torque
  void preloadTorqueLBConstraint(GraspStruct &P);
  // constraint that keeps a variable v lower bounded by the largest relative approach of a variable to its virtual limit
  void virtualLimitLBConstraint(GraspStruct &P, bool iterative=false);
  // constraint that limits the virtual object motion
  void objectMotionLimit(GraspStruct &P);

  // solution bounds and types
  void variableBoundsAndTypes(GraspStruct &P, const Matrix &beta_p = Matrix(0,0));

  // Methods that construct the problems to be solved by the optimization solver
  // Variables necessary for a base problem for the refinement solver
  void refinementBaseVariables(GraspStruct &P);
  // Constraints necessary for a base problem for the refinement solver
  void refinementBaseConstraints(GraspStruct &P, const Matrix &preload, double normUnc, bool rigid);
  // Construct an equilibrium problem for the refinement solver
  void refinementFormulationEquilibrium(GraspStruct &P, const Matrix &preload, double normUnc, const Matrix &wrench = Matrix::ZEROES<Matrix>(6,1), 
    bool rigid=false);
  // Construct a maximum wrench problem for the refinement solver
  void refinementFormulationFindMax(GraspStruct &P, const Matrix &preload, double normUnc, const Matrix &direction, bool rigid=false);
  // Construct an optimal preload problem for the refinement solver
  void refinementFormulationPreload(GraspStruct &P, const Matrix &preload, double normUnc, const Matrix &wrench = Matrix::ZEROES<Matrix>(6,1), 
    bool rigid=false);
  // Construct an optimization problem with constraints specific to a tendon actuated hand
  void nonIterativeTendonFormulation(GraspStruct &P, const Matrix &preload, const Matrix &wrench = Matrix::ZEROES<Matrix>(6,1), 
    const Matrix &beta = Matrix(0,0));
  // Construct an optimization problem with the iterative formulation
  void iterativeFormulation(GraspStruct &P, const Matrix &preload, const Matrix &startingX, const Matrix &movement_directions, 
    const Matrix &wrench, const Matrix &beta, bool rigid);
  // Construct an optimization problem with constraints spicific to a tendon actuated hand
  void iterativeTendonFormulation(GraspStruct &P, const Matrix &preload, const Matrix &startingX, const Matrix &movement_directions, 
    const Matrix &wrench, const Matrix &beta);

  // Refinement solver. Must be passed appropriately formulated problems
  int refinementSolver(SolutionStruct &S, std::tr1::function<void(GraspStruct&)> formulation, double coneTol);
  // Iterative solver. Must be passed appropriately formulated problems
  int iterativeSolver(SolutionStruct &S, bool cone_movement, 
    std::tr1::function<void(GraspStruct&,const Matrix&,const Matrix&)> formulation);

  // Functions that can be used by the binary search.
  int checkWrenchNonIterative(Matrix &preload, const Matrix &wrench, 
    const Matrix &beta_p, bool tendon, bool rigid) ;
  int checkWrenchIterative(Matrix &preload, const Matrix &wrench,
    const Matrix &beta_p, bool cone_movement, bool tendon, bool rigid);

  // solve a problem using the optimization solver
  int solveProblem(GraspStruct &P, SolutionStruct &S);

  // writes the solution to file
  void logSolution(SolutionStruct &S);

  void setVirtualLimits(const Matrix &preload, const Matrix &wrench = Matrix::ZEROES<Matrix>(6,1));

  void modifyFrictionEdges(double coneTol);
  void systemEnergyError(SolutionStruct &S, const Matrix &preload, const Matrix &wrench, double *e, bool findMax = false); 
  bool removeJointsBeyondContact(std::list<Joint*> &joints, const std::list<Contact*> &contacts, 
  Matrix &preload);
  bool removePassiveJoints(std::list<Joint*> &joints, Matrix &preload);

  void drawContactWrenches(const Matrix &beta);
  void drawObjectMovement(SolutionStruct &S);
  Matrix postProcessX(SolutionStruct &S);
  Matrix computeSignificantX(SolutionStruct &S);
};

//  --------------------------  Useful Matrices  -------------------------------  //

Matrix normalForceSelectionMatrix(const std::list<Contact*> &contacts);
Matrix normalDisplacementSelectionMatrix(int numContacts);
Matrix tangentialDisplacementSelectionMatrix(int numContacts);
Matrix tangentialDisplacementSummationMatrix(const std::list<Contact*> &contacts);
Matrix frictionConeEdgeMatrix(const std::list<Contact*> &contacts);

//  --------------------------  Useful Functions  ------------------------------  //

double angleBetweenEdges(const Matrix edge1, const Matrix edge2);
bool checkFrictionEdges(const std::list<Matrix> &frictionEdges);
void writeResultsToFile(SolutionStruct &S, const std::list<Contact*> &contacts);

#define GRASP_SOLVER_H
#endif