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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: matvec3D.h,v 1.17 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the classes: vec3, position, mat3, Quaternion, and transf.
 */
#ifndef _MATVEC_H_
#define _MATVEC_H_
#include <math.h>
#include <string.h>
#include <iostream>
#include <stack>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoTransform.h>

typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Matrix3d mat3;
typedef Eigen::Vector3d vec3;
typedef Eigen::Vector3d position;
typedef double col_Mat4[4][4];

#include "graspit/transform.h"




#define MACHINE_ZERO 1.0e-9
#define resabs 1.0e-5




/////////////////////////////////////////////////////////////////////////////////
////                       vec3  functions
/////////////////////////////////////////////////////////////////////////////////

/*!
  Sets the values in this vector with the values from the Inventor vector \a v.
 */
void SbVec3fTovec3(const SbVec3f &v, vec3 &vec);
vec3 SbVec3fTovec3(const SbVec3f &v);

SbRotation QuaterniontoSbRotation(Quaternion q);

/*!
  Converts this vector into an Inventor vector and returns it.
*/
SbVec3f
toSbVec3f(vec3 vec);

/*! This matrix becomes the cross product matrix for the given vector.
 In other words, multiplying a second vector v2 with this matrix is
 equivalent with perrforming the cross product v2 * v */
void
setCrossProductMatrix(mat3 &mat, const vec3 &v);

/*! Given two line segments, P1-P2 and P3-P4, returns the line segment
  Pa-Pb that is the shortest route between them. Calculates also the
  values of \a mua and \a mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Returns FALSE if no solution exists.
   adapted from http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
*/
inline int LineLineIntersect(vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3 *pa, vec3 *pb, double *mua, double *mub)
{
  vec3 p13, p43, p21;
  double d1343, d4321, d1321, d4343, d2121;
  double numer, denom;
  double EPS = 1.0e-5;

  p13.x() = p1.x() - p3.x();
  p13.y() = p1.y() - p3.y();
  p13.z() = p1.z() - p3.z();
  p43.x() = p4.x() - p3.x();
  p43.y() = p4.y() - p3.y();
  p43.z() = p4.z() - p3.z();
  if (fabs(p43.x())  < EPS && fabs(p43.y())  < EPS && fabs(p43.z())  < EPS) {
    return false;
  }

  p21.x() = p2.x() - p1.x();
  p21.y() = p2.y() - p1.y();
  p21.z() = p2.z() - p1.z();
  if (fabs(p21.x())  < EPS && fabs(p21.y())  < EPS && fabs(p21.z())  < EPS) {
    return false;
  }

  d1343 = p13.x() * p43.x() + p13.y() * p43.y() + p13.z() * p43.z();
  d4321 = p43.x() * p21.x() + p43.y() * p21.y() + p43.z() * p21.z();
  d1321 = p13.x() * p21.x() + p13.y() * p21.y() + p13.z() * p21.z();
  d4343 = p43.x() * p43.x() + p43.y() * p43.y() + p43.z() * p43.z();
  d2121 = p21.x() * p21.x() + p21.y() * p21.y() + p21.z() * p21.z();

  denom = d2121 * d4343 - d4321 * d4321;
  if (fabs(denom) < EPS) {
    return false;
  }
  numer = d1343 * d4321 - d1321 * d4343;

  *mua = numer / denom;
  *mub = (d1343 + d4321 * (*mua)) / d4343;

  pa->x() = p1.x() + (*mua) * p21.x();
  pa->y() = p1.y() + (*mua) * p21.y();
  pa->z() = p1.z() + (*mua) * p21.z();
  pb->x() = p3.x() + (*mub) * p43.x();
  pb->y() = p3.y() + (*mub) * p43.y();
  pb->z() = p3.z() + (*mub) * p43.z();

  return true;
}

#endif


