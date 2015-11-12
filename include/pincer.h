//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2004  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author:  Andrew T. Miller (amiller@cs.columbia.edu)
//
// $Id: pincer.h,v 1.2 2009/07/23 21:32:08 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %Pincer robot class
 */

#ifndef PINCER_H

#include "robot.h"

//! Created for a 2 fingered hand while testing a DOF controller specifically designed for this hand.
/*!  Created for a 2 fingered hand while testing a DOF controller specifically
  designed for this hand.
*/
class Pincer : public Hand {

 public:
  void DOFController(double timeStep);

};

#define PINCER_H
#endif
