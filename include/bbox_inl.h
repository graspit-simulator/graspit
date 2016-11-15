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
// $Id: bbox_inl.h,v 1.2 2009/04/09 16:02:44 cmatei Exp $
//
//######################################################################

/*! \file
	Bounding box functions to be inlined during release compilation.
	Do not put any of the necessary includes here, put them in both
	bbox.h and bbox.cpp instead.
*/

double 
pointBoxDistanceSq(const BoundingBox &box, const position &p)
{
	vec3 v = (p - position::ORIGIN) - box.getTran().translation();
	mat3 RMat;
	box.getTran().rotation().ToRotationMatrix(RMat);

    double sqDist = 0.0;
    for (int i=0; i<3; i++) {
		double d = v % RMat.row(i);
		double excess = 0.0;
        // Project vector from box center to p on each axis, getting the distance
        // of p along that axis, and count any excess distance outside box extents
		if (d < -box.halfSize[i])
			excess = d + box.halfSize[i];
		else if (d > box.halfSize[i])
			excess = d - box.halfSize[i];
		sqDist += excess * excess;
    }
    return sqDist;
}

/*! Code from REAL-TIME COLLISION DETECTION by Christer Ericson, 
	published by Elsevier.
*/
position 
closestPtBbox(const BoundingBox &bbox, const position &p)
{
	vec3 d = (p - position::ORIGIN) - bbox.getTran().translation();
	mat3 RMat;
	bbox.getTran().rotation().ToRotationMatrix(RMat);
    // Start result at center of box; make steps from there
	vec3 q = bbox.getTran().translation();
    // For each OBB axis...
    for (int i=0; i<3; i++) {
        // project d onto that axis to get the distance
        // along the axis of d from the box center
		double dist = d % RMat.row(i);
        // If distance farther than the box extents, clamp to the box
		if (dist > bbox.halfSize[i]) dist = bbox.halfSize[i];
		if (dist < -bbox.halfSize[i]) dist = -bbox.halfSize[i];
        // Step that distance along the axis to get world coordinate
		q += dist * RMat.row(i);
    }
	return position( q.x(), q.y(), q.z() );
}

void 
BoundingBox::applyTransform(const transf &t)
{
	mTran = mTran * t;
	mTranInv = mTran.inverse();
}

bool
bboxOverlap(const BoundingBox &bb1, const BoundingBox &bb2, const transf &tran2To1)
{
	int i, k;

	transf BtoA = bb2.getTran() * tran2To1 * bb1.getTranInv();
	mat3 RMat;
	BtoA.rotation().ToRotationMatrix(RMat);

	double B[3][3];
	for( i=0; i<3 ; i++ ) {
		for( k=0; k<3 ; k++ ) {
			B[i][k] = RMat.element(k,i);
		}
	}

	//translation between box centers
	vec3 T = BtoA.translation();

	vec3 a = bb1.halfSize;
	vec3 b = bb2.halfSize;

  double t, s;
  register int r;
  double Bf[3][3];
  const double reps = 1e-6;
  
  // Bf = fabs(B)
  Bf[0][0] = fabs(B[0][0]);  Bf[0][0] += reps;
  Bf[0][1] = fabs(B[0][1]);  Bf[0][1] += reps;
  Bf[0][2] = fabs(B[0][2]);  Bf[0][2] += reps;
  Bf[1][0] = fabs(B[1][0]);  Bf[1][0] += reps;
  Bf[1][1] = fabs(B[1][1]);  Bf[1][1] += reps;
  Bf[1][2] = fabs(B[1][2]);  Bf[1][2] += reps;
  Bf[2][0] = fabs(B[2][0]);  Bf[2][0] += reps;
  Bf[2][1] = fabs(B[2][1]);  Bf[2][1] += reps;
  Bf[2][2] = fabs(B[2][2]);  Bf[2][2] += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint
  r = 1;

  // A1 x A2 = A0
  t = fabs(T[0]);
  
  r &= (t <= 
	  (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]));
  if (!r) return false;
  
  // B1 x B2 = B0
  s = T[0]*B[0][0] + T[1]*B[1][0] + T[2]*B[2][0];
  t = fabs(s);

  r &= ( t <=
	  (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]));
  if (!r) return false;
    
  // A2 x A0 = A1
  t = fabs(T[1]);
  
  r &= ( t <= 
	  (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]));
  if (!r) return false;

  // A0 x A1 = A2
  t = fabs(T[2]);

  r &= ( t <= 
	  (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]));
  if (!r) return false;

  // B2 x B0 = B1
  s = T[0]*B[0][1] + T[1]*B[1][1] + T[2]*B[2][1];
  t = fabs(s);

  r &= ( t <=
	  (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]));
  if (!r) return false;

  // B0 x B1 = B2
  s = T[0]*B[0][2] + T[1]*B[1][2] + T[2]*B[2][2];
  t = fabs(s);

  r &= ( t <=
	  (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]));
  if (!r) return false;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = fabs(s);
  
  r &= ( t <= 
	(a[1] * Bf[2][0] + a[2] * Bf[1][0] +
	 b[1] * Bf[0][2] + b[2] * Bf[0][1]));
  if (!r) return false;
  
  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = fabs(s);

  r &= ( t <=
	(a[1] * Bf[2][1] + a[2] * Bf[1][1] +
	 b[0] * Bf[0][2] + b[2] * Bf[0][0]));
  if (!r) return false;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = fabs(s);

  r &= ( t <=
	  (a[1] * Bf[2][2] + a[2] * Bf[1][2] +
	   b[0] * Bf[0][1] + b[1] * Bf[0][0]));
  if (!r) return false;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = fabs(s);

  r &= ( t <=
	  (a[0] * Bf[2][0] + a[2] * Bf[0][0] +
	   b[1] * Bf[1][2] + b[2] * Bf[1][1]));
  if (!r) return false;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = fabs(s);

  r &= ( t <=
	  (a[0] * Bf[2][1] + a[2] * Bf[0][1] +
	   b[0] * Bf[1][2] + b[2] * Bf[1][0]));
  if (!r) return false;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = fabs(s);

  r &= (t <=
	  (a[0] * Bf[2][2] + a[2] * Bf[0][2] +
	   b[0] * Bf[1][1] + b[1] * Bf[1][0]));
  if (!r) return false;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = fabs(s);

  r &= (t <=
	  (a[0] * Bf[1][0] + a[1] * Bf[0][0] +
	   b[1] * Bf[2][2] + b[2] * Bf[2][1]));
  if (!r) return false;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = fabs(s);

  r &= ( t <=
	  (a[0] * Bf[1][1] + a[1] * Bf[0][1] +
	   b[0] * Bf[2][2] + b[2] * Bf[2][0]));
  if (!r) return false;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = fabs(s);

  r &= ( t <=
	  (a[0] * Bf[1][2] + a[1] * Bf[0][2] +
	   b[0] * Bf[2][1] + b[1] * Bf[2][0]));
  if (!r) return false;

  return true;  // should equal 0
}

double bboxDistanceApp(const BoundingBox &bb1, const BoundingBox &bb2)
{
	//for now a placeholder: return the distance as if they were bounding spheres
	//uses three sqrt's, maybe could use less

	vec3 t = bb1.getTran().translation() - bb2.getTran().translation();
	double dist = t.len();
	dist -= bb1.halfSize.len();
	dist -= bb2.halfSize.len();
	return dist;
}

double bboxDistanceSq(const BoundingBox &bb1, const BoundingBox &bb2, const transf &tran2To1)
{
	//trying to use the separating axis theorem to get a better distance bound
	int i, k;

	transf BtoA = bb2.getTran() * tran2To1 * bb1.getTranInv();
	mat3 RMat;
	BtoA.rotation().ToRotationMatrix(RMat);

	double B[3][3];
	for( i=0; i<3 ; i++ ) {
		for( k=0; k<3 ; k++ ) {
			B[i][k] = RMat.element(k,i);
		}
	}

	//translation between box centers
	vec3 T = BtoA.translation();

	vec3 a = bb1.halfSize;
	vec3 b = bb2.halfSize;

	double Bf[3][3];
	const double reps = 1e-6;
  
	// Bf = fabs(B)
	Bf[0][0] = fabs(B[0][0]);  Bf[0][0] += reps;
	Bf[0][1] = fabs(B[0][1]);  Bf[0][1] += reps;
	Bf[0][2] = fabs(B[0][2]);  Bf[0][2] += reps;
	Bf[1][0] = fabs(B[1][0]);  Bf[1][0] += reps;
	Bf[1][1] = fabs(B[1][1]);  Bf[1][1] += reps;
	Bf[1][2] = fabs(B[1][2]);  Bf[1][2] += reps;
	Bf[2][0] = fabs(B[2][0]);  Bf[2][0] += reps;
	Bf[2][1] = fabs(B[2][1]);  Bf[2][1] += reps;
	Bf[2][2] = fabs(B[2][2]);  Bf[2][2] += reps;

	double d, maxD = -1.0e10;
	double t, s;
	double a0,a1,a2,b0,b1,b2;


	// A1 x A2 = A0
	t = fabs(T[0]);  
	d = t - (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]);
	a0 = std::max(0.0, d); a0*=a0;
     
	// A2 x A0 = A1
	t = fabs(T[1]);
	d = t - (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]);
	a1 = std::max(0.0, d); a1*=a1;

	// A0 x A1 = A2
	t = fabs(T[2]);
	d = t - (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]);
	a2 = std::max(0.0, d); a2*=a2;

	// B1 x B2 = B0
	s = T[0]*B[0][0] + T[1]*B[1][0] + T[2]*B[2][0];
	t = fabs(s);
	d = t - (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]);
	b0 = std::max(0.0, d); b0*=b0;

	// B2 x B0 = B1
	s = T[0]*B[0][1] + T[1]*B[1][1] + T[2]*B[2][1];
	t = fabs(s);
	d = t - (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]);
	b1 = std::max(0.0, d); b1*=b1;

	// B0 x B1 = B2
	s = T[0]*B[0][2] + T[1]*B[1][2] + T[2]*B[2][2];
	t = fabs(s);
	d = t - (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]);
	b2 = std::max(0.0, d); b2*=b2;

	maxD = std::max(a0+a1+a2, b0+b1+b2);
//	if (maxD == 0.0) return -1;
//	return maxD;

	//---- edge cross product tests

	// A0 x B0
	s = T[2] * B[1][0] - T[1] * B[2][0];
	t = fabs(s);
    d = t - (a[1] * Bf[2][0] + a[2] * Bf[1][0] + b[1] * Bf[0][2] + b[2] * Bf[0][1]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a0+d, b0+d));
  
	// A0 x B1
	s = T[2] * B[1][1] - T[1] * B[2][1];
	t = fabs(s);
	d = t - (a[1] * Bf[2][1] + a[2] * Bf[1][1] + b[0] * Bf[0][2] + b[2] * Bf[0][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a0+d, b1+d));

	// A0 x B2
	s = T[2] * B[1][2] - T[1] * B[2][2];
	t = fabs(s);
	d = t - (a[1] * Bf[2][2] + a[2] * Bf[1][2] + b[0] * Bf[0][1] + b[1] * Bf[0][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a0+d, b2+d));

	// A1 x B0
	s = T[0] * B[2][0] - T[2] * B[0][0];
	t = fabs(s);
	d = t - (a[0] * Bf[2][0] + a[2] * Bf[0][0] + b[1] * Bf[1][2] + b[2] * Bf[1][1]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a1+d, b0+d));

	// A1 x B1
	s = T[0] * B[2][1] - T[2] * B[0][1];
	t = fabs(s);
	d = t - (a[0] * Bf[2][1] + a[2] * Bf[0][1] + b[0] * Bf[1][2] + b[2] * Bf[1][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a1+d, b1+d));

	// A1 x B2
	s = T[0] * B[2][2] - T[2] * B[0][2];
	t = fabs(s);
	d = t - (a[0] * Bf[2][2] + a[2] * Bf[0][2] + b[0] * Bf[1][1] + b[1] * Bf[1][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a1+d, b2+d));

	// A2 x B0
	s = T[1] * B[0][0] - T[0] * B[1][0];
	t = fabs(s);
	d = t - (a[0] * Bf[1][0] + a[1] * Bf[0][0] + b[1] * Bf[2][2] + b[2] * Bf[2][1]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a2+d, b0+d));

	// A2 x B1
	s = T[1] * B[0][1] - T[0] * B[1][1];
	t = fabs(s);
	d = t - (a[0] * Bf[1][1] + a[1] * Bf[0][1] + b[0] * Bf[2][2] + b[2] * Bf[2][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a2+d, b1+d));

	// A2 x B2
	s = T[1] * B[0][2] - T[0] * B[1][2];
	t = fabs(s);
	d = t - (a[0] * Bf[1][2] + a[1] * Bf[0][2] + b[0] * Bf[2][1] + b[1] * Bf[2][0]);
	d = std::max(0.0, d); d*=d;
	maxD = std::max(maxD, std::max(a2+d, b2+d));

	if (maxD == 0.0) return -1;
	return maxD;
}
