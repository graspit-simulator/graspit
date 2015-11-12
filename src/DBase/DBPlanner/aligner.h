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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: aligner.h,v 1.2 2009/06/17 18:57:14 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %Aligner class
 */

#ifndef DB_PLANNER_ALIGNER_H
#define DB_PLANNER_ALIGNER_H

#include "model.h"

namespace db_planner {

//! Align a Model with some data.
/*! A class to align a Model to an Dest which may be a point 
    cloud, a mesh, depth images, or something else. 
    Transforms are left-multiply and column major. */
template <class Dest>
class Aligner {
 public:
  //!! Align a Model to a Dest (templated on Dest) using a given 4x4 transform.
  virtual bool Align(const Model& /*source*/, 
                     const Dest& /*dest*/, 
                     float /*transform*/[16]) const {
    return false;
  }
  virtual ~Aligner() {}
};

/*
class PCAAligner : public Aligner<Body? {
public:
  virtual bool Align(const Model& source,
                     const Body& dest, 
                     float transform[16]) const {
    cout << "PCA Aligner";
    return false;
  }
}
*/


}  // end namespace db_planner


#endif  // DB_PLANNER_ALIGNER_H