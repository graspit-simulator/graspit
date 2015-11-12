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
