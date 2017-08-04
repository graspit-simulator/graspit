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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: matvec.cpp,v 1.12 2009/04/08 15:25:52 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the classes: vec3, position , mat3 , Quaternion , and transf
 */
#include "graspit/matvec3D.h"

//#define GRASPITDBG
#include "graspit/debug.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

/////////////////////////////////////////////////////////////////////////////////
////                       vec3  functions
/////////////////////////////////////////////////////////////////////////////////

/*!
  Sets the values in this vector with the values from the Inventor vector \a v.
 */
void SbVec3fTovec3(const SbVec3f &v, vec3 &vec)
{
  vec(0) = (double) v[0];
  vec(1) = (double) v[1];
  vec(2) = (double) v[2];
}

vec3 SbVec3fTovec3(const SbVec3f &v)
{
  vec3 vec;
  vec(0) = (double) v[0];
  vec(1) = (double) v[1];
  vec(2) = (double) v[2];

  return vec;
}

SbRotation QuaterniontoSbRotation(Quaternion q)
{
  return SbRotation((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w());
}

/*!
  Converts this vector into an Inventor vector and returns it.
*/
SbVec3f toSbVec3f(vec3 vec)
{
  return SbVec3f((float) vec[0], (float) vec[1], (float) vec[2]);
}

/*! This matrix becomes the cross product matrix for the given vector.
 In other words, multiplying a second vector v2 with this matrix is
 equivalent with perrforming the cross product v2 * v */
void setCrossProductMatrix(mat3 &mat, const vec3 &v)
{
  mat(0) = 0.0;
  mat(1)= v.z();
  mat(2) = -v.y();

  mat(3) = -v.z();
  mat(4) = 0.0;
  mat(5) = v.x();

  mat(6) = v.y();
  mat(7) = -v.x();
  mat(8) = 0.0;
}

