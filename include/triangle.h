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
// Author(s): Matei T. Ciocarlie
//
// $Id: triangle.h,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _triangle_h_
#define _triangle_h_

#include <vector>

#include "matvec3D.h"
#include "mytools.h"
/*! Very simple class, only holds 3 public vertices as positions */
class Triangle {
public:
	position v1,v2,v3;

	Triangle(const position &nv1, const position &nv2, const position &nv3) : v1(nv1), v2(nv2), v3(nv3) {}
	Triangle(const Triangle &t) : v1(t.v1), v2(t.v2), v3(t.v3) {}
	inline void applyTransform(const transf &t);
	inline double area() const;
	inline position centroid() const;
	inline vec3 normal() const;

	friend INLINE_RELEASE bool triangleIntersection(const Triangle &t1, const Triangle &t2);
};

INLINE_RELEASE bool 
triangleIntersection(const Triangle &t1, const Triangle &t2);

INLINE_RELEASE position 
closestPtTriangle(const Triangle &t, const position &p);

//! Returns the distance between the triangles as well as the two closest points on them
INLINE_RELEASE double 
triangleTriangleDistanceSq(const Triangle &t1, const Triangle &t2,
 						   position &p1, position &p2);

//! Returns all the points on the two triangles separated by less than the threshold
INLINE_RELEASE int
triangleTriangleContact(const Triangle &t1, const Triangle &t2, double threshSq, 
						std::vector< std::pair<position, position> >* contactPoints);

double 
Triangle::area() const
{
	return  0.5 * ((v2 - v1) * (v3 - v1)).len();
}

vec3 
Triangle::normal() const
{
	return normalise( (v2 - v1) * (v3 - v1) );
}

position 
Triangle::centroid() const
{
	return (1.0 / 3.0) * (v1 + v2 + v3);
}

void
Triangle::applyTransform(const transf &t)
{
	v1 = v1*t;
	v2 = v2*t;
	v3 = v3*t;
}

#ifdef GRASPIT_RELEASE
#include "triangle_inl.h"
#endif

#endif
