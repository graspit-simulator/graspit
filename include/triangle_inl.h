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
// $Id: triangle_inl.h,v 1.2 2009/04/09 16:03:02 cmatei Exp $
//
//######################################################################

/*! \file
	Triangle functions to be inlined during release compilation.
	Do not put any of the necessary includes here, put them in both
	triangle.h and triangle.cpp instead.
*/

inline double gmax(double d1, double d2, double d3)
{
	double m = d1;
	if (d2 > m) m = d2;
	if (d3 > m) m = d3;
	return m;
}

inline double gmin(double d1, double d2, double d3)
{
	double m = d1;
	if (d2 < m) m = d2;
	if (d3 < m) m = d3;
	return m;
}

inline int
project6(const vec3 &ax, 
         const vec3 &p1, const vec3 &p2, const vec3 &p3, 
         const vec3 &q1, const vec3 &q2, const vec3 &q3)
{
  double P1 = ax % p1;
  double P2 = ax % p2;
  double P3 = ax % p3;
  double Q1 = ax % q1;
  double Q2 = ax % q2;
  double Q3 = ax % q3;
  
  double mx1 = gmax(P1, P2, P3);
  double mn1 = gmin(P1, P2, P3);
  double mx2 = gmax(Q1, Q2, Q3);
  double mn2 = gmin(Q1, Q2, Q3);

  if (mn1 > mx2) return 0;
  if (mn2 > mx1) return 0;
  return 1;
}

inline
bool triangleIntersection(const Triangle &t1, const Triangle &t2)
{
  vec3 p1, p2, p3;
  vec3 q1, q2, q3;
  vec3 e1, e2, e3;
  vec3 f1, f2, f3;
  vec3 g1, g2, g3;
  vec3 h1, h2, h3;
  vec3 n1, m1;

  vec3 ef11, ef12, ef13;
  vec3 ef21, ef22, ef23;
  vec3 ef31, ef32, ef33;
  
  p1[0] = t1.v1[0] - t1.v1[0];  p1[1] = t1.v1[1] - t1.v1[1];  p1[2] = t1.v1[2] - t1.v1[2];
  p2[0] = t1.v2[0] - t1.v1[0];  p2[1] = t1.v2[1] - t1.v1[1];  p2[2] = t1.v2[2] - t1.v1[2];
  p3[0] = t1.v3[0] - t1.v1[0];  p3[1] = t1.v3[1] - t1.v1[1];  p3[2] = t1.v3[2] - t1.v1[2];
  
  q1[0] = t2.v1[0] - t1.v1[0];  q1[1] = t2.v1[1] - t1.v1[1];  q1[2] = t2.v1[2] - t1.v1[2];
  q2[0] = t2.v2[0] - t1.v1[0];  q2[1] = t2.v2[1] - t1.v1[1];  q2[2] = t2.v2[2] - t1.v1[2];
  q3[0] = t2.v3[0] - t1.v1[0];  q3[1] = t2.v3[1] - t1.v1[1];  q3[2] = t2.v3[2] - t1.v1[2];
  
  e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];
  
  n1= e1 * e2;
  m1= f1 * f2;

  g1= e1 * n1;
  g2= e2 * n1;
  g3= e3 * n1;
  h1= f1 * m1;
  h2= f2 * m1;
  h3= f3 * m1;

  ef11= e1 * f1;
  ef12= e1 * f2;
  ef13= e1 * f3;
  ef21= e2 * f1;
  ef22= e2 * f2;
  ef23= e2 * f3;
  ef31= e3 * f1;
  ef32= e3 * f2;
  ef33= e3 * f3;
  
  // now begin the series of tests

  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return 0;
  
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return 0;

  return 1;
}

/*! Code from REAL-TIME COLLISION DETECTION by Christer Ericson, 
	published by Elsevier.
*/
position closestPtTriangle(const Triangle &t, const position &p)
{
    // Check if P in vertex region outside t.v1
    vec3 ab = t.v2 - t.v1;
    vec3 ac = t.v3 - t.v1;
    vec3 ap = p - t.v1;
    double d1 = ab % ap;
    double d2 = ac % ap;
    if (d1 <= 0.0f && d2 <= 0.0f) return t.v1; // barycentric coordinates (1,0,0)

    // Check if P in vertex region outside t.v2
    vec3 bp = p - t.v2;
    double d3 = ab % bp;
    double d4 = ac % bp;
    if (d3 >= 0.0f && d4 <= d3) return t.v2; // barycentric coordinates (0,1,0)

    // Check if P in edge region of AB, if so return projection of P onto AB
    double vc = d1*d4 - d3*d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        double v = d1 / (d1 - d3);
        return t.v1 + v * ab; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside t.v3
    vec3 cp = p - t.v3;
    double d5 = ab % cp;
    double d6 = ac % cp;
    if (d6 >= 0.0f && d5 <= d6) return t.v3; // barycentric coordinates (0,0,1)

    // Check if P in edge region of AC, if so return projection of P onto AC
    double vb = d5*d2 - d1*d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        double w = d2 / (d2 - d6);
        return t.v1 + w * ac; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    double va = d3*d6 - d5*d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return t.v2 + w * (t.v3 - t.v2); // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    double denom = 1.0f / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return t.v1 + ab * v + ac * w; // = u*t.v1 + v*t.v2 + w*t.v3, u = va * denom = 1.0f - v - w
}

// Clamp n to lie within the range [min, max]
inline double Clamp(double n, double min, double max) {
    if (n < min) return min;
    if (n > max) return max;
    return n;
}

/*! Code from REAL-TIME COLLISION DETECTION by Christer Ericson, 
	published by Elsevier.
*/
inline double segmSegmDistanceSq(const position &p1, const position &q1, 
								 const position &p2, const position &q2,
								 position &c1, position &c2)
{
	double EPSILON = 1.0e-3;
    vec3 d1 = q1 - p1; // Direction vector of segment S1
    vec3 d2 = q2 - p2; // Direction vector of segment S2
    vec3 r = p1 - p2;
    double a = d1 % d1; // Squared length of segment S1, always nonnegative
    double e = d2 % d2; // Squared length of segment S2, always nonnegative
    double f = d2 % r;
	double s, t;

    // Check if either or both segments degenerate into points
    if (a <= EPSILON && e <= EPSILON) {
        // Both segments degenerate into points
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return (c1 - c2) % (c1 - c2);
    }
    if (a <= EPSILON) {
        // First segment degenerates into a point
        s = 0.0f;
        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
        t = Clamp(t, 0.0f, 1.0f);
    } else {
        double c = d1 % r;
        if (e <= EPSILON) {
            // Second segment degenerates into a point
            t = 0.0f;
            s = Clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
        } else {
            // The general nondegenerate case starts here
            double b = d1 % d2;
            double denom = a*e-b*b; // Always nonnegative

            // If segments not parallel, compute closest point on L1 to L2, and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0f) {
                s = Clamp((b*f - c*e) / denom, 0.0f, 1.0f);
            } else s = 0.0f;

            // Compute point on L2 closest to S1(s) using
            // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
            t = (b*s + f) / e;

            // If t in [0,1] done. Else clamp t, recompute s for the new value
            // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
            // and clamp s to [0, 1]
            if (t < 0.0f) {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = Clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return (c1 - c2) % (c1 - c2);
}

/*! Does all 6 vertex-face tests and all 9 edge-edge tests and adds to the report
	all distinct pairs of resulting points separated by less than the threshold.
	Returns the number of contact points, 0 if the triangles are not in contact
	and -1 if the triangles are in collision.

	I have not tested all the obscure cases, and it seems that more involved
	collision detection engines put more intelligence into this. There might
	be strange (or degenerate) pieces of geometry where this had an unexpected
	result.
*/
int
triangleTriangleContact(const Triangle &t1, const Triangle &t2, double threshSq, 
						std::vector< std::pair<position, position> >* contactPoints)
{
	if (triangleIntersection(t1,t2)) return -1.0;
	position p1, p2;
	//vertices on triangle 2 and face on triangle 1
	//--
	p1 = closestPtTriangle(t1, t2.v1);
	p2 = t2.v1;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		contactPoints->push_back( std::pair<position, position>(p1,p2));
	}
	//--
	p1 = closestPtTriangle(t1, t2.v2);
	p2 = t2.v2;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		contactPoints->push_back( std::pair<position, position>(p1,p2));
	}
	//--
	p1 = closestPtTriangle(t1, t2.v3);
	p2 = t2.v3;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		contactPoints->push_back( std::pair<position, position>(p1,p2));
	}
	//vertices on triangle 1 and face on triangle 2
	//if closest point on triangle 2 is a vertex, skip this pair as we have already done
	//the vertices from triangle 2
	p2 = closestPtTriangle(t2, t1.v1);
	p1 = t1.v1;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		if ( !(p2==t2.v1) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	p2 = closestPtTriangle(t2, t1.v2);
	p1 = t1.v2;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		if ( !(p2==t2.v1) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	p2 = closestPtTriangle(t2, t1.v3);
	p1 = t1.v3;
	if (((p2 - p1) % (p2 - p1)) < threshSq) {
		if ( !(p2==t2.v1) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//------------------------------------------------------------------

	//nine edge-edge tests
	//if one of the closest points on an edge is actually a vertex, skip this pair
	//as we have already done the vertex closest points
	double d;
	d = segmSegmDistanceSq(t1.v1, t1.v2, t2.v1, t2.v2, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v1) && !(p1==t1.v2) && !(p2==t2.v1) && !(p2==t2.v2) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v1, t1.v2, t2.v2, t2.v3, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v1) && !(p1==t1.v2) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v1, t1.v2, t2.v3, t2.v1, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v1) && !(p1==t1.v2) && !(p2==t2.v3) && !(p2==t2.v1) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//------------------
	d = segmSegmDistanceSq(t1.v2, t1.v3, t2.v1, t2.v2, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v2) && !(p1==t1.v3) && !(p2==t2.v1) && !(p2==t2.v2) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v2, t1.v3, t2.v2, t2.v3, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v2) && !(p1==t1.v3) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v2, t1.v3, t2.v3, t2.v1, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v2) && !(p1==t1.v3) && !(p2==t2.v3) && !(p2==t2.v1) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//------------------
	d = segmSegmDistanceSq(t1.v3, t1.v1, t2.v1, t2.v2, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v3) && !(p1==t1.v1) && !(p2==t2.v1) && !(p2==t2.v2) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v3, t1.v1, t2.v2, t2.v3, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v3) && !(p1==t1.v1) && !(p2==t2.v2) && !(p2==t2.v3) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	//--
	d = segmSegmDistanceSq(t1.v3, t1.v1, t2.v3, t2.v1, p1, p2);
	if (d < threshSq) {
		if ( !(p1==t1.v3) && !(p1==t1.v1) && !(p2==t2.v3) && !(p2==t2.v1) ) {
			contactPoints->push_back( std::pair<position, position>(p1,p2));
		}
	}
	return contactPoints->size();
}

double triangleTriangleDistanceSq(const Triangle &t1, const Triangle &t2,
								  position &p1, position &p2)
{
	if (triangleIntersection(t1,t2)) return -1.0;

	double dtmp, dmin;
	position tmp1, tmp2;
	//six vertex - face tests
	//--
	p1 = closestPtTriangle(t1, t2.v1);
	p2 = t2.v1;
	dmin = (p2 - p1) % (p2 - p1);
	//--
	tmp1 = closestPtTriangle(t1, t2.v2);
	tmp2 = t2.v2;
	dtmp = (tmp2 - tmp1) % (tmp2 - tmp1);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	tmp1 = closestPtTriangle(t1, t2.v3);
	tmp2 = t2.v3;
	dtmp = (tmp2 - tmp1) % (tmp2 - tmp1);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	tmp2 = closestPtTriangle(t2, t1.v1);
	tmp1 = t1.v1;
	dtmp = (tmp2 - tmp1) % (tmp2 - tmp1);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	tmp2 = closestPtTriangle(t2, t1.v2);
	tmp1 = t1.v2;
	dtmp = (tmp2 - tmp1) % (tmp2 - tmp1);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	tmp2 = closestPtTriangle(t2, t1.v3);
	tmp1 = t1.v3;
	dtmp = (tmp2 - tmp1) % (tmp2 - tmp1);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//------------------------------------------------------------------

	//nine edge-edge tests
	dtmp = segmSegmDistanceSq(t1.v1, t1.v2, t2.v1, t2.v2, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v1, t1.v2, t2.v2, t2.v3, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v1, t1.v2, t2.v3, t2.v1, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//-------------------------------
	dtmp = segmSegmDistanceSq(t1.v2, t1.v3, t2.v1, t2.v2, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v2, t1.v3, t2.v2, t2.v3, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v2, t1.v3, t2.v3, t2.v1, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//-----------------------------------
	dtmp = segmSegmDistanceSq(t1.v3, t1.v1, t2.v1, t2.v2, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v3, t1.v1, t2.v2, t2.v3, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--
	dtmp = segmSegmDistanceSq(t1.v3, t1.v1, t2.v3, t2.v1, tmp1, tmp2);
	if (dtmp < dmin) {
		dmin = dtmp;
		p1 = tmp1; p2 = tmp2;
	}
	//--------


	return dmin;
}
