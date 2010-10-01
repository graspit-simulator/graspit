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
// Author(s):  Andrew T. Miller 
//
// $Id: puma560.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the Puma560 robot subclass that has an analytic IK solution.
*/

#ifndef PUMA560_H
#include "robot.h"

struct Puma560Solution;

//! Specifically for the Puma 560 Arm.  Contains analytical inverse kinematics solution
/*! This class was created specifically for the Puma 560 Arm because we can
    override the generic inverse kinematics routine with one that solves them
    analytically.  
*/
class Puma560 : public Robot {
  //! There are 8 possible solutions because of the different possible arm configurations
  static const int NUM_SOLUTIONS;

  /*! Helper routine to find which of the 8 solutions is closest to the current
      position of the arm.
  */
  int findClosestSol(Puma560Solution *candidates, Puma560Solution *current);

 public:
  /* Empty constructor */
  Puma560(World *w,const char *name) : Robot(w,name) {}

  /* Solves the inverse kinematics of the puma arm analytically. */
  virtual int invKinematics(const transf& endTran,double* dofVals,
			    int chainNum=0);
};

#define PUMA560_H
#endif
