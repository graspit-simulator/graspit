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


}  // end namespace db_planner


#endif  // DB_PLANNER_ALIGNER_H