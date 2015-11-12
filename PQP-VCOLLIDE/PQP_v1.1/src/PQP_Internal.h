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

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include "Tri.h"
#include "TriDist.h"
#include "BV.h"
#include <set>
#include <list>
#include <vector>
#include <functional>

//cnl- Exchange Tri to Tri3B, so that GetNghbhd can be used, se Tri.h for definition
//of Tri3B, changed throughout PQP code, functionally its the same, it just contains
//more information about the triangle

class PQP_Model
{

public:

  int build_state;

  Tri3B *tris;  
  int num_tris;
  int num_tris_alloced;

  int *triIndex;

  BV *b;
  int num_bvs;
  int num_bvs_alloced;

  Tri3B *last_tri;       // closest tri on this model in last distance test
  
  BV *child(int n) { return &b[n]; }

  PQP_Model();
  ~PQP_Model();

  int BeginModel(int num_tris = 8); // preallocate for num_tris triangles;
                                    // the parameter is optional, since
                                    // arrays are reallocated as needed
  int AddTri(const PQP_REAL *p1, const PQP_REAL *p2, const PQP_REAL *p3, 
             int id);
  int MoveTri(const PQP_REAL *p1, const PQP_REAL *p2, const PQP_REAL *p3, 
             int id);
  
  Tri3B *GetTri(int id) {return &tris[triIndex[id]];}

  int EndModel(bool ExpectEmpty = false);
  int MovesFinished();
  int MemUsage(int msg);  // returns model mem usage.  
                          // prints message to stderr if msg == TRUE

  void getBvs(int desiredDepth, std::vector<BV*> *bvs);
};

struct CollisionPair
{
  int id1;
  int id2;
};

struct PQP_CollideResult  
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  int num_pairs_alloced;
  int num_pairs;
  CollisionPair *pairs;

  void SizeTo(int n);    
  void Add(int i1, int i2); 

  PQP_CollideResult();
  ~PQP_CollideResult();

  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // free the list of contact pairs; ordinarily this list is reused
  // for each query, and only deleted in the destructor.

  void FreePairsList(); 

  // query results

  int Colliding() { return (num_pairs > 0); }
  int NumPairs() { return num_pairs; }
  int Id1(int k) { return pairs[k].id1; }
  int Id2(int k) { return pairs[k].id2; }
};

#if PQP_BV_TYPE & RSS_TYPE // distance/tolerance are only available with RSS


struct DistancePair
{
  int id1;
  int id2;
  TriDistCaseT topo;  // Topology info for closest points
  PQP_REAL p[3];
  PQP_REAL q[3];
};

struct PQP_DistanceResult 
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  PQP_REAL rel_err; 
  PQP_REAL abs_err; 

  PQP_REAL distance;
  PQP_REAL p1[3]; 
  PQP_REAL p2[3];
  int qsize;

  // ATM: added this to hold the pairs of triangles that are within the
  // distance threshhold from eachother
  int num_pairs_alloced;
  int num_pairs;
  DistancePair *pairs;
  
  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // The following distance and points established the minimum distance
  // for the models, within the relative and absolute error bounds 
  // specified.
  // Points are defined: PQP_REAL p1[3], p2[3];

  PQP_REAL Distance() { return distance; }
  const PQP_REAL *P1() { return p1; }
  const PQP_REAL *P2() { return p2; }


  // ATM: added this to hold the pairs of triangles that are within the
  // distance threshhold from eachother
  void SizeTo(int n);    
  void Add(int i1, int i2, TriDistCaseT topo,PQP_REAL p[3], PQP_REAL q[3]); 
  void FreePairsList(); 

  int WithinThresh() { return (num_pairs > 0); }
  int NumPairs() { return num_pairs; }
  int Id1(int k) { return pairs[k].id1; }
  int Id2(int k) { return pairs[k].id2; }
  TriDistCaseT Topology(int k) { return pairs[k].topo; }
  PQP_REAL *P1(int k) { return pairs[k].p; }
  PQP_REAL *P2(int k) { return pairs[k].q; }

  PQP_DistanceResult();
  ~PQP_DistanceResult();
};

//cnl-modified contact pair to contain addresses of BV elements that directly
//contain contacting triangles.  This is used to navigate the BV tree after
//contacts have been found
struct ContactPair
{
  int id1;
  int id2;
  BV *bv1;
  BV *bv2;
  int numVerts;
  PQP_REAL p[6][3];
  PQP_REAL q[6][3];
};

struct EdgeContact
{
  PQP_REAL pt1[3],pt2[3];  // the endpoints of the edge
  int otherTriID;    // the triange this edge is contacting
};


struct PQPContactPt
{
  PQP_REAL b1_pos[3],b1_normal[3];
  PQP_REAL b2_pos[3],b2_normal[3];
};

// comparison function for two 6 vectors,
struct ltContact : public std::binary_function<PQPContactPt,PQPContactPt,bool>
{
  bool operator()(const PQPContactPt &v1, const PQPContactPt &v2) const
  {
    if (v1.b1_pos[2] + MY_RESABS < v2.b1_pos[2]) return true;
    else if (v1.b1_pos[2] - v2.b1_pos[2] < MY_RESABS) {
      if (v1.b1_pos[1] + MY_RESABS < v2.b1_pos[1]) return true;
      else if (v1.b1_pos[1] - v2.b1_pos[1] < MY_RESABS) {
        if (v1.b1_pos[0] + MY_RESABS < v2.b1_pos[0]) return true;
	else if (v1.b1_pos[0] - v2.b1_pos[0] < MY_RESABS) {
	  if (v1.b1_normal[2] + MY_RESABS < v2.b1_normal[2]) return true;
	  else if (v1.b1_normal[2] - v2.b1_normal[2] < MY_RESABS) {
	    if (v1.b1_normal[1] + MY_RESABS < v2.b1_normal[1]) return true;
	    else if (v1.b1_normal[1] - v2.b1_normal[1] < MY_RESABS) {
	      if (v1.b1_normal[0] + MY_RESABS < v2.b1_normal[0]) return true;
	    }
	  }
	}
      }
    }
    return false;
  }
};

//A set imposes const properties that we can not follow in the code
//therefore this has been changed to a list and ltContact has been dropped
//everywhere in code where this change has been made we mark with STOL
//typedef std::set<PQPContactPt,ltContact> ContactSetT;

//STOL
typedef std::list<PQPContactPt> ContactSetT;

struct PQP_ContactResult 
{
  // stats
  int body1Id,body2Id;
  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  PQP_REAL rel_err; 
  PQP_REAL abs_err; 

  PQP_REAL distance;
  PQP_REAL p1[3]; 
  PQP_REAL p2[3];
  int qsize;

  // holds the pairs of triangles that are "in contact" with eachother
  int num_pairs_alloced;
  int num_pairs;
  ContactPair *pairs;
  

  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // The following distance and points established the minimum distance
  // for the models, within the relative and absolute error bounds 
  // specified.
  // Points are defined: PQP_REAL p1[3], p2[3];

  PQP_REAL Distance() { return distance; }
  const PQP_REAL *P1() { return p1; }
  const PQP_REAL *P2() { return p2; }


  // ATM: added this to hold the pairs of triangles that are within the
  // distance threshhold from eachother
  void SizeTo(int n);    
  void Add(int i1, int i2, BV *b1, BV *b2, int numVerts, PQP_REAL p[][3], PQP_REAL q[][3]); 
  void FreePairsList(); 

  int NumPairs() { return num_pairs; }
  int Id1(int k) { return pairs[k].id1; }
  int Id2(int k) { return pairs[k].id2; }

  ContactSetT contactSet;
  std::list<EdgeContact> edgeContactList;

  PQP_ContactResult();
  ~PQP_ContactResult();
};

struct PQP_ToleranceResult 
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  int    closer_than_tolerance;   
  PQP_REAL tolerance;      

  PQP_REAL distance;
  PQP_REAL p1[3]; 
  PQP_REAL p2[3]; 
  int qsize;

  // statistics

  int NumBVTests() { return num_bv_tests; }
  int NumTriTests() { return num_tri_tests; }
  double QueryTimeSecs() { return query_time_secs; }

  // If the models are closer than ( <= ) tolerance, these points 
  // and distance were what established this.  Otherwise, 
  // distance and point values are not meaningful.

  PQP_REAL Distance() { return distance; }
  const PQP_REAL *P1() { return p1; }
  const PQP_REAL *P2() { return p2; }

  // boolean says whether models are closer than tolerance distance

  int CloserThanTolerance() { return closer_than_tolerance; }
};

#endif
