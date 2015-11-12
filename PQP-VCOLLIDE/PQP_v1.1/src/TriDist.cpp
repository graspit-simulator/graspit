/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

//--------------------------------------------------------------------------
// File:   TriDist.cpp
// Author: Eric Larsen
// Description:
// contains SegPoints() for finding closest points on a pair of line
// segments and TriDist() for finding closest points on a pair of triangles
//--------------------------------------------------------------------------

#include "MatVec.h"
#include "TriDist.h"
#include <list>

//#define PQPDEBUG

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

#define ZEROTOL 1.0e-11
#define SLACKTOL 1.0e-5

//--------------------------------------------------------------------------
// SegPoints() 
//
// Returns closest points between an segment pair.
// Implemented from an algorithm described in
//
// Vladimir J. Lumelsky,
// On fast computation of distance between line segments.
// In Information Processing Letters, no. 21, pages 55-61, 1985.   
//--------------------------------------------------------------------------

bool debug = false;

void
SegPoints(PQP_REAL X[3], PQP_REAL Y[3],             // closest points
	  TriDistCaseT &topo,                       // topology info
          const PQP_REAL P[3], const PQP_REAL A[3], // seg 1 origin, vector
          const PQP_REAL Q[3], const PQP_REAL B[3]) // seg 2 origin, vector
{
  PQP_REAL T[3], A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;

  VmV(T,Q,P);
  A_dot_A = VdotV(A,A);
  B_dot_B = VdotV(B,B);
  A_dot_B = VdotV(A,B);
  A_dot_T = VdotV(A,T);
  B_dot_T = VdotV(B,T);

  // t parameterizes ray P,A 
  // u parameterizes ray Q,B 

  PQP_REAL t,u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  PQP_REAL denom = A_dot_A*B_dot_B - A_dot_B*A_dot_B;

  t = (A_dot_T*B_dot_B - B_dot_T*A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if (t < 0) t = 0; else if (t > 1) t = 1; 
  else if (!(t >= 0 && t <= 1)) t = 0; // for NAN
  if (denom == 0.0) t=0;  //ATM:  I added this.  the above case doesn't seem
                          //      to catch NAN

  // find u for point on ray Q,B closest to point at t

  u = (t*A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to 
  // closest points, otherwise, clamp u, recompute and
  // clamp t 

  if (u < 0) 
  {
    u = 0;
    t = A_dot_T / A_dot_A;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    else if (!(t >= 0 && t <= 1)) t = 0; // for NAN
  }
  else if (u > 1) 
  {
    u = 1;
    t = (A_dot_B + A_dot_T) / A_dot_A;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    else if (!(t >= 0 && t <= 1)) t = 0; // for NAN
  }
  else if (!(u >= 0 && u <= 1))  // for NAN
  {
    u = 0;
    t = A_dot_T / A_dot_A;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    else if (!(t >= 0 && t <= 1)) t = 0; // for NAN
  }
  
  // ATM: I added checks on the parameter values to record the topology
  // of the closest points
  if ((u < ZEROTOL || u > 1-ZEROTOL)) {
    if ((t < ZEROTOL || t > 1-ZEROTOL)) topo = VERTEX_VERTEX;
    else topo = EDGE_VERTEX;
  }
  else {
    if ((t < ZEROTOL || t > 1-ZEROTOL)) topo = VERTEX_EDGE;
    else
      topo = EDGE_EDGE;
  }
  VpVxS(X,P,A,t);
  VpVxS(Y,Q,B,u);
}

//--------------------------------------------------------------------------
// TriDist() 
//
// Computes the closest points on two triangles, and returns the 
// distance between them.
// 
// S and T are the triangles, stored tri[point][dimension].
//
// If the triangles are disjoint, P and Q give the closest points of 
// S and T respectively. However, if the triangles overlap, P and Q 
// are basically a random pair of points from the triangles, not 
// coincident points on the intersection of the triangles, as might 
// be expected.
//--------------------------------------------------------------------------

PQP_REAL 
TriDist(PQP_REAL P[3], PQP_REAL Q[3], TriDistCaseT &topo,
        const PQP_REAL S[3][3], const PQP_REAL T[3][3],int &SedgeNum,
	int &TedgeNum)  
{
  // Compute vectors along the 6 sides

  PQP_REAL Sv[3][3], Tv[3][3];

  VmV(Sv[0],S[1],S[0]);
  VmV(Sv[1],S[2],S[1]);
  VmV(Sv[2],S[0],S[2]);

  VmV(Tv[0],T[1],T[0]);
  VmV(Tv[1],T[2],T[1]);
  VmV(Tv[2],T[0],T[2]);

  // For each edge pair, the vector connecting the closest points 
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of 
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  PQP_REAL V[3];
  PQP_REAL Z[3];
  PQP_REAL minP[3], minQ[3], mindd;
  int shown_disjoint = 0;
  TriDistCaseT currTopo;

  mindd = VdistV2(S[0],T[0]) + 1;  // Set first minimum safely high

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {

      // Find closest points on edges i & j, plus the 
      // vector (and distance squared) between these points
      
      SegPoints(P,Q,currTopo,S[i],Sv[i],T[j],Tv[j]);
	       
      VmV(V,Q,P);
      PQP_REAL dd = VdotV(V,V);

#ifdef PQPDEBUG
      printf("Sedge %d, Tedge %d) dist^2 = %15.12lf\n",i,j,dd);
#endif
      // Verify this closest point pair only if the distance 
      // squared is less than the minimum found thus far.
      
      if (dd <= mindd) // <= ?
      {
        VcV(minP,P);
        VcV(minQ,Q);
        mindd = dd;
		topo = currTopo;
		SedgeNum = i;
		TedgeNum = j;

        VmV(Z,S[(SedgeNum+2)%3],P);
        PQP_REAL a = VdotV(Z,V);
        VmV(Z,T[(TedgeNum+2)%3],Q);
        PQP_REAL b = VdotV(Z,V);

#ifdef PQPDEBUG
		printf("      a,b: %15.12lf  %15.12lf\n",a,b);
#endif
		// ATM: make this strict so extra vertices must lie outside the planes
	    if ((a < -ZEROTOL) && (b > ZEROTOL)) {
       // if ((a < -SLACKTOL) && (b > SLACKTOL)) {
			return sqrt(VdotV(V,V));
		}
	// if ((a <= 0) && (b >= 0)) return sqrt(VdotV(V,V));
        
        if (a <= 0) a = 0;
        else if (b > 0) b = 0;
        if ((mindd - a + b) > 0) {
			shown_disjoint = 1;
		}
      }
    }
  }
 
  // No edge pairs contained the closest points.  
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the 
  //    triangle points are nearly colinear or coincident, one 
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  PQP_REAL Sn[3], Snl;       
  VcrossV(Sn,Sv[0],Sv[1]); // Compute normal to S triangle
  Snl = VdotV(Sn,Sn);      // Compute square of length of normal
  
  // If cross product is long enough,

  if (Snl > ZEROTOL)  
  {
    // Get projection lengths of T points

    PQP_REAL Tp[3]; 

    VmV(V,S[0],T[0]);
    Tp[0] = VdotV(V,Sn);

    VmV(V,S[0],T[1]);
    Tp[1] = VdotV(V,Sn);

    VmV(V,S[0],T[2]);
    Tp[2] = VdotV(V,Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
    {
      if (Tp[0] < Tp[1]) point = 0; else point = 1;
      if (Tp[2] < Tp[point]) point = 2;
    }
    else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
    {
      if (Tp[0] > Tp[1]) point = 0; else point = 1;
      if (Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction, 

    if (point >= 0 && mindd > 0) 
    {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the 
      // other triangle, lies within the face.
    
      VmV(V,T[point],S[0]);
      VcrossV(Z,Sn,Sv[0]);
      if (VdotV(V,Z) > 0)
      {
        VmV(V,T[point],S[1]);
        VcrossV(Z,Sn,Sv[1]);
        if (VdotV(V,Z) > 0)
        {
          VmV(V,T[point],S[2]);
          VcrossV(Z,Sn,Sv[2]);
          if (VdotV(V,Z) > 0)
          {
            // T[point] passed the test - it's a closest point for 
            // the T triangle; the other point is on the face of S

            VpVxS(P,T[point],Sn,Tp[point]/Snl);
            VcV(Q,T[point]);
	    topo = FACE_VERTEX;
            return sqrt(VdistV2(P,Q));
          }
        }
      }
    }
  }

  PQP_REAL Tn[3], Tnl;       
  VcrossV(Tn,Tv[0],Tv[1]); 
  Tnl = VdotV(Tn,Tn);      
  
  if (Tnl > ZEROTOL)  
  {
    PQP_REAL Sp[3]; 

    VmV(V,T[0],S[0]);
    Sp[0] = VdotV(V,Tn);

    VmV(V,T[0],S[1]);
    Sp[1] = VdotV(V,Tn);

    VmV(V,T[0],S[2]);
    Sp[2] = VdotV(V,Tn);

    int point = -1;
    if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
    {
      if (Sp[0] < Sp[1]) point = 0; else point = 1;
      if (Sp[2] < Sp[point]) point = 2;
    }
    else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
    {
      if (Sp[0] > Sp[1]) point = 0; else point = 1;
      if (Sp[2] > Sp[point]) point = 2;
    }

    if (point >= 0 && mindd > 0) 
    { 
      shown_disjoint = 1;

      VmV(V,S[point],T[0]);
      VcrossV(Z,Tn,Tv[0]);
      if (VdotV(V,Z) > 0)
      {
        VmV(V,S[point],T[1]);
        VcrossV(Z,Tn,Tv[1]);
        if (VdotV(V,Z) > 0)
        {
          VmV(V,S[point],T[2]);
          VcrossV(Z,Tn,Tv[2]);
          if (VdotV(V,Z) > 0)
          {
            VcV(P,S[point]);
            VpVxS(Q,S[point],Tn,Sp[point]/Tnl);
	    topo = VERTEX_FACE;
            return sqrt(VdistV2(P,Q));
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2, 
  // that the triangles overlap.
  
  if (shown_disjoint)
  {
    PQP_REAL dist = sqrt(mindd);
    VcV(P,minP);
    VcV(Q,minQ);
    VmV(V,Q,P);

#ifdef PQPDEBUG
    printf("Contact normal: %15.12lf %15.12lf %15.12lf\n",V[0],V[1],V[2]);
    printf("Normal dot S-Plane normal: %15.12lf\n",fabs(VdotV(V,Sn)) / (dist*sqrt(Snl)));
    printf("Normal dot T-Plane normal: %15.12lf\n",fabs(VdotV(V,Tn)) / (dist*sqrt(Tnl)));
#endif
    
    if (fabs(VdotV(V,Sn)) / (dist*sqrt(Snl)) > 1-SLACKTOL)
      topo = FACE_EDGE;
    else if (fabs(VdotV(V,Tn)) / (dist*sqrt(Tnl)) > 1-SLACKTOL)
      topo = EDGE_FACE;
    else {
      topo = UNKNOWN;
    }

    return dist;
  }
  else {
    topo = INTERSECT;
    return 0;
  }
}


struct simpleEdge2D {
  PQP_REAL head[2],tail[2];
};

//
// This is adapted from a routine by Patrick Xavier
// It cuts a polygon with a halfplane defined by Eqn
//

int
CutPolygon(std::list<simpleEdge2D> &edgeList,PQP_REAL Eqn[3])
{
  std::list<simpleEdge2D>::iterator ep,htc_p,thc_p,nextp;
  bool someVertsAbove,someVertsBelow;
  PQP_REAL *head,*tail;
  double headdist,taildist;  // perp distance of the head and tail vertices
                             // from the cutting line

  // cut each edge of the current polygon with a line of T
  someVertsAbove = someVertsBelow = false;
  for (ep=nextp=edgeList.begin(),htc_p=thc_p=edgeList.end();
       ep!=edgeList.end();ep=nextp) {

    nextp++;

    headdist = Eqn[0] * ep->head[0] + Eqn[1] * ep->head[1] + Eqn[2];
    taildist = Eqn[0] * ep->tail[0] + Eqn[1] * ep->tail[1] + Eqn[2];
    
    if (headdist > 0) someVertsAbove = true;
    else someVertsBelow = true;
    
    // if edge is totally on the wrong side of the halfplane, remove it
    if (headdist > 0 && taildist > 0) {
      edgeList.erase(ep);
      continue;
    }
    
    if (headdist <= 0 && taildist > 0)
      htc_p = ep;
    else if (taildist <=0 && headdist > 0)
      thc_p = ep;
  }

  if (!someVertsAbove) return 0;

  // the entire polygon is cut out by the halfplane
  if (!someVertsBelow) {
    edgeList.clear();
    return -1;
  }

  bool erase_htc_p = false;
  bool erase_thc_p = false;
  double frac;
  PQP_REAL ev[2];
  
  head = htc_p->head;
  tail = htc_p->tail;
  
  headdist = Eqn[0] * head[0] + Eqn[1] * head[1] + Eqn[2];
  
  // So halfplane must have cut 2 of the polygon edges
  if (headdist < -ZEROTOL) {
    // Don't eliminate edge; just shorten it so that its tail lies on line.
    taildist = Eqn[0] * tail[0] + Eqn[1] * tail[1] + Eqn[2];
    frac = headdist / (headdist - taildist);
    ev[0] = frac * (tail[0] - head[0]);
    ev[1] = frac * (tail[1] - head[1]);
    htc_p->tail[0] = head[0] + ev[0];
    htc_p->tail[1] = head[1] + ev[1];
  }
  else erase_htc_p = true;
  head = thc_p->head;
  tail = thc_p->tail;
  
  taildist = Eqn[0] * tail[0] + Eqn[1] * tail[1] + Eqn[2];
  
  if (taildist < -ZEROTOL) {
    headdist = Eqn[0] * head[0] + Eqn[1] * head[1] + Eqn[2];
    frac = taildist / (taildist - headdist);
    ev[0] = frac * (head[0] - tail[0]);
    ev[1] = frac * (head[1] - tail[1]);
    thc_p->head[0] = tail[0] + ev[0];
    thc_p->head[1] = tail[1] + ev[1];
  }
  else erase_thc_p = true;

  // create the new closing edge
  
  // Head of the new closing edge will be the head or the tail of the edge
  // whose head is on or below the cutting line and whose tail is above it.
  if (erase_htc_p) head = htc_p->head;
  else head = htc_p->tail;
 
  // Tail  of the new closing edge will be the head or the tail of the edge
  // whose tail is on or below the cutting line and whose head is above it.
  if (erase_thc_p) tail = thc_p->tail;
  else tail = thc_p->head;
 
  // Create new closing edge and insert it.  Then erase remaining edges that
  // need to be erased.
  PQP_REAL tmp[2];
  tmp[0] = tail[0] - head[0];
  tmp[1] = tail[1] - head[1];

  if (sqrt((tmp[0]*tmp[0])+(tmp[1]*tmp[1])) > ZEROTOL) {
    simpleEdge2D closingEdge;
    closingEdge.head[0] = head[0]; closingEdge.head[1] = head[1];
    closingEdge.tail[0] = tail[0]; closingEdge.tail[1] = tail[1];

    edgeList.insert(htc_p, closingEdge);
  }
  else if (!erase_htc_p && !erase_thc_p) {
    // sharp angle cut
    htc_p->tail[0] = thc_p->head[0] = (head[0] + tail[0])/2.0;
    htc_p->tail[1] = thc_p->head[1] = (head[1] + tail[1])/2.0;
  }
  else if (erase_htc_p != erase_thc_p) {
  fprintf(stderr,"Uh oh. in CutPolygon, looks like we missed a case.");
  }
  
  if (erase_htc_p) edgeList.erase(htc_p);   
  if (erase_thc_p) edgeList.erase(thc_p);

  return(1);
    
}

#define DIST_THRESH 1.0e-6

// This projects the points of T to S and finds the polygon of intersection.
// It returns only the point(s) of the intersection that is(are) closest
// to S when projected back to T.  The function returns the number of
// closest intersection points, P contains the intersection point(s) on
// the plane of S and Q contains the intersection points(s) on the plane of T.
//
int
TriOverlap(PQP_REAL P[][3], PQP_REAL Q[][3], PQP_REAL N[3],
	   const PQP_REAL S[3][3], const PQP_REAL T[3][3])  
{
  int i,j;
  PQP_REAL Sv[3][3], Tv[3][3];

  // Compute vectors along the 6 sides
  VmV(Sv[0],S[1],S[0]);
  VmV(Sv[1],S[2],S[1]);
  VmV(Sv[2],S[0],S[2]);

  VmV(Tv[0],T[1],T[0]);
  VmV(Tv[1],T[2],T[1]);
  VmV(Tv[2],T[0],T[2]);

  // Compute normal to S triangle
  PQP_REAL Sn[3],unitSn[3],Soffset;       
  VcrossV(Sn,Sv[0],Sv[1]); 
  VcV(unitSn,Sn);
  Vnormalize(unitSn);
  N[0] = -unitSn[0]; N[1] = -unitSn[1]; N[2] = -unitSn[2];

#ifdef PQPDEBUG
printf("unitSn: %15.12lf %15.12lf %15.12lf\n",unitSn[0],unitSn[1],unitSn[2]);
#endif
  
  Soffset = VdotV(S[0],unitSn);

  // Compute normal of T triangle
  PQP_REAL Tn[3],unitTn[3];       
  VcrossV(Tn,Tv[0],Tv[1]); 
  VcV(unitTn,Tn);
  Vnormalize(unitTn);

#ifdef PQPDEBUG
printf("unitTn: %15.12lf %15.12lf %15.12lf\n",unitTn[0],unitTn[1],unitTn[2]);
#endif

  // compute the origin of the projection frame  
  PQP_REAL origin_pr[3];
  VxS(origin_pr,unitSn,Soffset);

  // compute 2 other axes along the plane of S
  PQP_REAL axis1[3],axis2[3];
  VcV(axis1,Sv[0]);
  Vnormalize(axis1);
  VcrossV(axis2,unitSn,axis1);

  // The two triangles projected to a 2D plane parallel to S
  PQP_REAL S_2D[3][2], T_2D[3][2];

  // project both triangles to the (axis1,axis2,unitSn) frame
  for (i=0;i<3;i++) {
    S_2D[i][0] = VdotV(S[i],axis1);
    S_2D[i][1] = VdotV(S[i],axis2);
  }

  if (VdotV(Tn,Sn) > 0)
    for(i=0;i<3;i++) {
      T_2D[i][0] = VdotV(T[i],axis1);
      T_2D[i][1] = VdotV(T[i],axis2);
    }
  else
    for(i=0;i<3;i++) {
      T_2D[i][0] = VdotV(T[2-i],axis1);
      T_2D[i][1] = VdotV(T[2-i],axis2);
    }

  PQP_REAL tmp[3],p[6][3],q[6][3];
  PQP_REAL maxDist,tmpDist,dist[6];
  int numIntersections;

  PQP_REAL denom = VdotV(unitTn,unitSn);
  if (denom > -1e-7 && denom < 1e-7) {
    // triangles are perpendicular 

#ifdef PQPDEBUG
    printf("perpendicular\n");
#endif
    bool edgeCrossed = false;
    PQP_REAL t,s,Tx1,Tx2,Ty1,Ty2,Sx1,Sx2,Sy1,Sy2;

#ifdef PQPDEBUG
    for (i=0;i<3;i++)
      printf("S_2D[%d]: %15.12lf %15.12lf\n",i,S_2D[i][0],S_2D[i][1]);
    for (i=0;i<3;i++)
      printf("T_2D[%d]: %15.12lf %15.12lf\n",i,T_2D[i][0],T_2D[i][1]);
#endif

    // if a T edge crosses an S edge, move the exterior vertex to the S edge
    for (i=0;i<3;i++) {
      Tx1 = T_2D[i][0];       Ty1 = T_2D[i][1];
      Tx2 = T_2D[(i+1)%3][0]; Ty2 = T_2D[(i+1)%3][1];
      
      for (j=0;j<3;j++) {
	Sx1 = S_2D[j][0];       Sy1 = S_2D[j][1];
	Sx2 = S_2D[(j+1)%3][0]; Sy2 = S_2D[(j+1)%3][1];

	// find the intersection of the line segment with the edge of S
	t = ((Ty1 - Sy1)*(Sx2 - Sx1) - (Tx1 - Sx1)*(Sy2 - Sy1))/
	  ((Tx2 - Tx1)*(Sy2 - Sy1) - (Ty2 - Ty1)*(Sx2 - Sx1));
	
	s = ((Ty1 - Sy1)*(Tx2 - Tx1) - (Tx1 - Sx1)*(Ty2 - Ty1))/
	  ((Tx2 - Tx1)*(Sy2 - Sy1) - (Ty2 - Ty1)*(Sx2 - Sx1));
	
	if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
	  edgeCrossed = true;
#ifdef PQPDEBUG
	  printf("edge %d cuts edge %d\n",i,j);
#endif
	  // if the tail of T is exterior to this halfplane, move it
	  if ((Tx1-Sx1)*(Sy2-Sy1) - (Ty1-Sy1)*(Sx2-Sx1) > 0) {
	    Tx1 = T_2D[i][0] += t*(Tx2-Tx1);
	    Ty1 = T_2D[i][1] += t*(Ty2-Ty1);
#ifdef PQPDEBUG
	    printf("moving T tail to: T_2D[%d] = %15.12lf %15.12lf\n",i,T_2D[i][0],T_2D[i][1]);
#endif
	  }
	  // if the head of T is exterior to this halfplane, move it
	  else if ((Tx2-Sx1)*(Sy2-Sy1) - (Ty2-Sy1)*(Sx2-Sx1) > 0) {
	    Tx2 = T_2D[(i+1)%3][0] -= (1-t)*(Tx2-Tx1);
	    Ty2 = T_2D[(i+1)%3][1] -= (1-t)*(Ty2-Ty1);
#ifdef PQPDEBUG
	    printf("moving T head to: T_2D[%d] = %15.12lf %15.12lf\n",(i+1)%3,T_2D[(i+1)%3][0],T_2D[(i+1)%3][1]);
#endif
	  }	      
	}
      }
    }
    
    // check if the T points are entirely exterior to S
    if (!edgeCrossed) {
      
      // just check one of the vertices
      Tx1 = T_2D[0][0];       Ty1 = T_2D[0][1];

      for (i=0;i<3;i++) {
	Sx1 = S_2D[i][0];       Sy1 = S_2D[i][1];
	Sx2 = S_2D[(i+1)%3][0]; Sy2 = S_2D[(i+1)%3][1];
	
	// check if the T vertex is exterior to the S-edge halfplane
	// if so then they all are
	if ((Tx1-Sx1)*(Sy2-Sy1) - (Ty1-Sy1)*(Sx2-Sx1) > 0)
	  return 0;
      }
    }

    PQP_REAL C[3];
    PQP_REAL Sn_dot_Sn,Sn_dot_Tv,Sn_dot_C,Tv_dot_Tv,Tv_dot_C;

    numIntersections = 3;

    //
    // Next, project the planar intersection points back up to the plane of S
    // and onto the edges of T.  Keep track of the distance of the from the
    // point on S to the point on T.

    // make the distance the maximum it could be
    // use the distance of furthest T vertex from S plane
    maxDist = 0.0;
    for (i=0;i<3;i++) {
      tmpDist = VdotV(T[i],unitSn) - Soffset;
      maxDist = ( tmpDist > maxDist ? tmpDist : maxDist );
    }

    for (i=0;i<3;i++) {
      dist[i] = maxDist;

      // Project the planar intersection vertices back up to the plane of S
      p[i][0] = T_2D[i][0] * axis1[0] + T_2D[i][1] * axis2[0] + origin_pr[0];
      p[i][1] = T_2D[i][0] * axis1[1] + T_2D[i][1] * axis2[1] + origin_pr[1];
      p[i][2] = T_2D[i][0] * axis1[2] + T_2D[i][1] * axis2[2] + origin_pr[2];
#ifdef PQPDEBUG
      printf("p[%d]: %15.12lf %15.12lf %15.12lf\n",i,p[i][0],p[i][1],p[i][2]);
#endif

      // examine each edge of T
      for (j=0;j<3;j++) {
	
	// C is vector from tail of j-th T edge to i-th 
	// intersection point on S	
	VmV(C,T[j],p[i]);

	if (fabs(VdotV(Tv[j],unitSn)/Vlength(Tv[j])) >= 1.0 - ZEROTOL) {
	  // edge is perpendicular to the plane
	  
	  // projectection only intersects Tv if C is also perpendicular
	  if (fabs(VdotV(C,unitSn)/Vlength(C)) >= 1.0 - ZEROTOL) {
	  // ray and edge are coincident so use distance to edge head or tail
	  
	  // check distance to tail
	  VmV(tmp,T[j],p[i]);
	  tmpDist = Vlength(tmp);
	  dist[i] = (tmpDist < dist[i] ? tmpDist : dist[i]);
	  
	  // check distance to head
	  VmV(tmp,T[(j+1)%3],p[i]);
	  tmpDist = Vlength(tmp);
	  dist[i] = (tmpDist < dist[i] ? tmpDist : dist[i]);

#ifdef PQPDEBUG
	  printf("dist[%d] (edge perp): %le",i,dist[i]);
#endif
	  
	  }
	}
	else {	  
#ifdef PQPDEBUG
      printf("T[%d]: %15.12lf %15.12lf %15.12lf  Tv[%d]: %15.12lf %15.12lf %15.12lf\n",i,T[i][0],T[i][1],T[i][2],i,Tv[i][0],Tv[i][1],Tv[i][2]);
#endif
	  Sn_dot_Sn = VdotV(unitSn,unitSn);
	  Tv_dot_Tv = VdotV(Tv[j],Tv[j]);
	  Sn_dot_Tv = VdotV(unitSn,Tv[j]);
	  Sn_dot_C = VdotV(unitSn,C);
	  Tv_dot_C = VdotV(Tv[j],C);	  

	  // compute parameter for the closest point on ray p[i],Sn to
	  // ray T[j],Tv[j], t will be actual distance because we used unitSn
	  
	  // this divide should be ok because the Sn and Tv should not
	  // be parallel in this section of code	  

	  s = (Sn_dot_C*Tv_dot_Tv - Tv_dot_C*Sn_dot_Tv) / 
	    (Sn_dot_Sn*Tv_dot_Tv - Sn_dot_Tv*Sn_dot_Tv);

	  t = (s*Sn_dot_Tv - Tv_dot_C) / Tv_dot_Tv;
	
#ifdef PQPDEBUG
	  printf("s: %18.15le  t: %18.15le\n",s,t);
#endif  
	  if (t>-ZEROTOL && t<=1+ZEROTOL) {  //Sn actually intersects the T edge
	    if (s < -ZEROTOL) {
#ifdef PQPDEBUG
	      printf("The T edge is touching the S plane??? s=%15.12le\n",s);
#endif
	    }
	    else {
	    dist[i] = (s < dist[i] ? s : dist[i]);
	    }
	  }
#ifdef PQPDEBUG
	  printf("dist[%d]: %le\n",i,dist[i]);
#endif
	}	
      }
      debug = true;

      // record the actual point on T as q[i]
      VxS(tmp,unitSn,dist[i]);
      VpV(q[i],tmp,p[i]);	
      
    }

  }
  
  else {  // planes of S and T are not perpendicular

    // record the edges of S as they will be cut by the lines of T
    std::list<simpleEdge2D> edgeList;
    std::list<simpleEdge2D>::iterator ep;
    
    for (i=0;i<3;i++) {
      simpleEdge2D tmpEdge;
      tmpEdge.head[0] = S_2D[(i+1)%3][0];
      tmpEdge.head[1] = S_2D[(i+1)%3][1];
      tmpEdge.tail[0] = S_2D[i][0];
      tmpEdge.tail[1] = S_2D[i][1];
   
      edgeList.push_back(tmpEdge);
    }
    
    PQP_REAL Eqn[3];  // equation of the cutting line
    
    // Cut the Polygon with each of the lines of T
    for (i=0;i<3;i++) {
      PQP_REAL dy = T_2D[(i+1)%3][1] - T_2D[i][1];
      PQP_REAL dx = T_2D[(i+1)%3][0] - T_2D[i][0];
      PQP_REAL len = sqrt(dx*dx + dy*dy);
      
      Eqn[0] = dy / len;
      Eqn[1] = -dx / len;
      Eqn[2] = -(Eqn[0] * T_2D[i][0] + Eqn[1] * T_2D[i][1]);  

      if (CutPolygon(edgeList,Eqn) < 0) return 0;  // the projected triangles
      // are disjoint
    }

    numIntersections = edgeList.size();

    // Now we have to find the distance from each intersection point to the
    // plane of T.
    
    
    // project the vertices back up to the plane of S and check how far each
    // is away from the plane of T (measured along the normal of S).
    
    for (ep=edgeList.begin(),i=0;ep!=edgeList.end();ep++,i++) { 
      // Project the planar intersection vertices back up to the plane of S
      p[i][0] = ep->tail[0] * axis1[0] + ep->tail[1] * axis2[0] + origin_pr[0];
      p[i][1] = ep->tail[0] * axis1[1] + ep->tail[1] * axis2[1] + origin_pr[1];
      p[i][2] = ep->tail[0] * axis1[2] + ep->tail[1] * axis2[2] + origin_pr[2];

      // compute the distance from the intersection point back up to the
      // plane of T
      VmV(tmp,T[0],p[i]);
      dist[i] = VdotV(unitTn,tmp)/denom;
      if (dist[i] < 0) {
#ifdef PQPDEBUG
	printf("-------------------NEGATIVE DIST!!!!!-----------------------\n");
	printf("T[0]: %15.12lf %15.12lf %15.12lf, tmp: %15.12lf %15.12lf %15.12lf  denom: %15.12lf\n",T[0][0],T[0][1],T[0][2],tmp[0],tmp[1],tmp[2],denom);
	printf("unitSn: %15.12lf %15.12lf %15.12lf, unitTn: %15.12lf %15.12lf %15.12lf\n",unitSn[0],unitSn[1],unitSn[2],unitTn[0],unitTn[1],unitTn[2]);
#endif
	VmV(tmp,T[1],p[i]);
	dist[i] = VdotV(unitTn,tmp)/denom;
#ifdef PQPDEBUG
	printf("dist[%d] try 2: %15.12lf\n",i,dist[i]);
#endif
	if (dist[i] < 0) {
	  VmV(tmp,T[2],p[i]);
	  dist[i] = VdotV(unitTn,tmp)/denom;
#ifdef PQPDEBUG
	  printf("dist[%d] try 3: %15.12lf\n",i,dist[i]);
#endif
	}
      }

      // record the actual point on T as q[i]
      VxS(tmp,unitSn,dist[i]);
      VpV(q[i],tmp,p[i]);	
#ifdef PQPDEBUG
      printf("dist[%d]: %le  p[%d]: %15.12lf %15.12lf %15.12lf   q[%d]: %15.12lf %15.12lf %15.12lf\n",i,dist[i],i,p[i][0],p[i][1],p[i][2],i,q[i][0],q[i][1],q[i][2]);
#endif

    }
  }
  
  // Find the minimum distance(s)
  // if only one is minumum (within threshold) it's a point contact
  // if two are minumum (within threshold) it's a line contact
  // if more than two are all minumum (within threshold) it's a plane contact
  //   (or it could be a line contact if the points are colinear)
  
  int minDistIdx = 0;
  int numMin = 1;
  for (i=1;i<numIntersections;i++)
    if (dist[i] < dist[minDistIdx])
      minDistIdx = i;

  VcV(P[0],p[minDistIdx]);
  VcV(Q[0],q[minDistIdx]);
  
  for (i=0;i<numIntersections;i++) {
    if (i != minDistIdx && (dist[i] < dist[minDistIdx] + DIST_THRESH)) {
      VcV(P[numMin],p[i]);
      VcV(Q[numMin],q[i]);
      numMin++;
    }
  }
debug = false;
  return numMin;
}

int
TriContactRegion(PQP_REAL P[][3],PQP_REAL Q[][3],PQP_REAL N[3],
		 TriDistCaseT topo, PQP_REAL A[3][3],PQP_REAL B[3][3])
{
  int num;
  switch (topo) {
    case VERTEX_VERTEX:
#ifdef PQPDEBUG
printf("VERTEX_VERTEX\n");
#endif
      // num = 0; 
      num = 1; 
      VmV(N,P[0],Q[0]);
      Vnormalize(N);
      break; 

    case VERTEX_EDGE : 
#ifdef PQPDEBUG
printf("VERTEX_EDGE\n");
#endif
      // num = 0; 
      num = 1; 
      VmV(N,P[0],Q[0]);
      Vnormalize(N); 
      break;


    case EDGE_VERTEX : 
#ifdef PQPDEBUG
printf("EDGE_VERTEX\n");
#endif
      //num = 0; 
      num = 1; 
      VmV(N,P[0],Q[0]);
      Vnormalize(N);  
      break;


  case EDGE_EDGE   :
#ifdef PQPDEBUG
printf("EDGE_EDGE\n");
#endif
      num = 1; 
      VmV(N,P[0],Q[0]);
      Vnormalize(N);
      break;


  case FACE_VERTEX : 
#ifdef PQPDEBUG
printf("FACE_VERTEX\n");
#endif
      num = TriOverlap(P,Q,N,A,B); break;


  case FACE_EDGE   : 
#ifdef PQPDEBUG
printf("FACE_EDGE\n");
#endif
      num = TriOverlap(P,Q,N,A,B); break;


  case VERTEX_FACE : 
#ifdef PQPDEBUG
printf("VERTEX_FACE\n");
#endif
      num = TriOverlap(Q,P,N,B,A);
      N[0] = -N[0]; N[1] = -N[1]; N[2] = -N[2];
      break;

  case EDGE_FACE   : 
#ifdef PQPDEBUG
printf("EDGE_FACE\n");
#endif
      num = TriOverlap(Q,P,N,B,A);
      N[0] = -N[0]; N[1] = -N[1]; N[2] = -N[2];
      break;

  case UNKNOWN     : 
#ifdef PQPDEBUG
printf("UNKNOWN\n");
#endif
      num = 0; break;//TriOverlap(P,Q,A,B); break;   // Take a guess


  case INTERSECT: 
#ifdef PQPDEBUG
printf("INTERSECT\n");
#endif
    num = 0;
  }

  return num;
}
