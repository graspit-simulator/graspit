/*********************************************************
PointDist.h
Claire Lackner
Contains functions for finding the distance between a point and a 
triangle and a point and a retangle
These functions are called by GetNeghbd in PQP.cpp in order to find
triangles within a certain distance of a point
**********************************************************/

#include <math.h>
#include "MatVec.h"
#include "Tri.h"
#include "PQP_Compile.h"
//#define PQPDEBUG

using namespace std;

inline
PQP_REAL ptRectDist( PQP_REAL len[2], PQP_REAL R[3][3], 
				  PQP_REAL T[3], PQP_REAL pt[3])
{
	PQP_REAL ptTrans[3];
	PQP_REAL p[3];

	//transform point into coordinate system of rectangle
	VmV(p, pt, T);
	MTxV(ptTrans, R, p);

	if( ptTrans[0] >= 0.0 && ptTrans[0] <= len[0] )
	{
		if( ptTrans[1] < 0.0 )
			return sqrt( ptTrans[1]*ptTrans[1] + 
						 ptTrans[2]*ptTrans[2] );
		else if( ptTrans[1] <= len[1] )
		{
			if( ptTrans[2] != 0.0 )
				return fabs(ptTrans[2]);
//			else
				return 0.0;
		}
//		else
			return sqrt( (ptTrans[1]-len[1])*(ptTrans[1]-len[1]) + 
						 ptTrans[2]*ptTrans[2] );
	}
	else if( ptTrans[0] < 0.0 )
	{
		if( ptTrans[1] < 0.0 )
			return sqrt(ptTrans[0]*ptTrans[0] + 
						ptTrans[1]*ptTrans[1] + 
						ptTrans[2]*ptTrans[2]);

		else if( ptTrans[1] <= len[1] )
			return sqrt( ptTrans[0]*ptTrans[0] +
						 ptTrans[2]*ptTrans[2] );
//		else
			return sqrt( ptTrans[0]*ptTrans[0] +
						(ptTrans[1]-len[1])*(ptTrans[1]-len[1]) + 
						 ptTrans[2]*ptTrans[2] );
	}
//	else
//	{
		if( ptTrans[1] < 0.0 )
			return sqrt((ptTrans[0]-len[0])*(ptTrans[0]-len[0]) + 
						ptTrans[1]*ptTrans[1] + 
						ptTrans[2]*ptTrans[2]);

		else if( ptTrans[1] <= len[1] )
			return sqrt((ptTrans[0]-len[0])*(ptTrans[0]-len[0]) + 
						 ptTrans[2]*ptTrans[2] );
//		else
			return sqrt( (ptTrans[0]-len[0])*(ptTrans[0]-len[0]) + 
						(ptTrans[1]-len[1])*(ptTrans[1]-len[1]) + 
						 ptTrans[2]*ptTrans[2] );
//	}
}

inline 
PQP_REAL ptRSSDist( PQP_REAL len[2], PQP_REAL r, PQP_REAL R[3][3], 
				  PQP_REAL T[3], PQP_REAL pt[3] )
{
	PQP_REAL dist = ptRectDist( len, R, T, pt ) - r;
	
	if( dist <= 0.0 )
		dist = 0.0;

	return dist;
}

inline
PQP_REAL ptTriDist( Tri3B *Tri, PQP_REAL R[3][3], 
				 PQP_REAL T[3], PQP_REAL pt[3], PQP_REAL closest_pt[3])
{
	PQP_REAL ptTrans[3];
	PQP_REAL nHeight;
	PQP_REAL ptPr[3], ptPr1[3], ptPr2[3], ptPr3[3];
	PQP_REAL t1;
	PQP_REAL temp[3];
	PQP_REAL len12, len23, len31;
	PQP_REAL dPlane;


	//transform point into coordinate system of triangle
	VmV(temp, pt, T);
	MTxV(ptTrans, R, temp);

	//move point such that origin is in triangle, take
	//the first coordinate of the triangle as the origin
	VmV( temp, ptTrans, Tri->p1 );

	nHeight = VdotV( temp, Tri->n );

	//ptPr is projection of ptTrans onto the plane of the triangle
	VxS( temp, Tri->n, nHeight );
	VmV( ptPr, ptTrans, temp );

	//inside/outside check
	//need to place point in coordinate frame
	//of each vertex as we go
	VmV(ptPr1, ptPr, Tri->p1);
	VmV(ptPr2, ptPr, Tri->p2);
	VmV(ptPr3, ptPr, Tri->p3);

	VcrossV( temp, Tri->s12, ptPr1 );
	if( VdotV( temp,  Tri->n ) >= 0.0 )
	{	
		VcrossV( temp, Tri->s23, ptPr2 );
		if( VdotV( temp, Tri->n ) >= 0.0 )
		{
			VcrossV( temp, Tri->s31, ptPr3 );
			if( VdotV( temp, Tri->n ) >= 0.0 ) {
				VcV(closest_pt, ptPr);
				//fprintf(stderr,"Closest pt inside triangle!\n");
				return fabs( nHeight );
			}
		}
	}

	//now that we are sure its on the outside, we need the distance
	VmV(temp, Tri->p1, Tri->p2);
	len12 = Vlength( temp );
	VmV(temp, Tri->p2, Tri->p3);
	len23 = Vlength( temp );
	VmV(temp, Tri->p1, Tri->p3);
	len31 = Vlength( temp );

	
	PQP_REAL dPlane1, dPlane2, dPlane3;
	PQP_REAL closest_pt1[3], closest_pt2[3], closest_pt3[3];
	t1 = VdotV( ptPr1, Tri->s12 );
	if( t1 >= 0.0 && t1 <= len12 ) {
		dPlane1= fabs(VdotV( ptPr1, Tri->ns12 ));
		VxS( temp, Tri->ns12, dPlane1);
		VmV( closest_pt1, ptPr1, temp);
		VpV( closest_pt1, Tri->p1, closest_pt1);
	}
	else {
		if ( Vlength(ptPr1) < Vlength(ptPr2) ) {
			dPlane1= Vlength(ptPr1);
			VcV (closest_pt1, Tri->p1);
		} else {
			dPlane1= Vlength(ptPr2);
			VcV (closest_pt1, Tri->p2);
		}
	}
	t1 = VdotV( ptPr2, Tri->s23 );
	if( t1 >= 0.0 && t1 <= len23 ) {
		dPlane2= fabs(VdotV( ptPr2, Tri->ns23 ));
		VxS( temp, Tri->ns23, dPlane2);
		VmV( closest_pt2, ptPr2, temp);
		VpV( closest_pt2, Tri->p2, closest_pt2);
	}
	else {
		if ( Vlength(ptPr2) < Vlength(ptPr3) ) {
			dPlane2= Vlength(ptPr2);
			VcV (closest_pt2, Tri->p2);
		} else {
			dPlane2= Vlength(ptPr3);
			VcV (closest_pt2, Tri->p3);
		}
	}
	t1 = VdotV( ptPr3, Tri->s31 );
	if( t1 >= 0.0 && t1 <= len31 ) {
		dPlane3= fabs(VdotV( ptPr3, Tri->ns31 ));
		VxS( temp, Tri->ns31, dPlane3);
		VmV( closest_pt3, ptPr3, temp);
		VpV( closest_pt3, Tri->p3, closest_pt3);
	}
	else {
		if ( Vlength(ptPr3) < Vlength(ptPr1) ) {
			dPlane3= Vlength(ptPr3);
			VcV (closest_pt3,Tri->p3);
		} else {
			dPlane3= Vlength(ptPr1);
			VcV (closest_pt3,Tri->p1);
		}
	}

	/*
	PQP_REAL d1,d2,d3;
	t1 = VdotV( ptPr2, Tri->s23 );
	if( t1 >= 0.0 && t1 <= len23 )
		d2 = fabs(VdotV( ptPr2, Tri->ns23 ));
	else
		d2 = min( Vlength(ptPr2), Vlength(ptPr3) );

	t1 = VdotV( ptPr3, Tri->s31 );
	if( t1 >= 0.0 && t1 <= len31 )
		d3 = fabs(VdotV( ptPr3, Tri->ns31 ));
	else
		d3 = min( Vlength(ptPr3), Vlength(ptPr1) );
	
	dPlane = min( d1, min( d2, d3 ) );
	*/

	dPlane = min( dPlane1, min( dPlane2, dPlane3 ) );
	VcV(closest_pt, closest_pt3);
	if ( dPlane2 < dPlane3 )VcV(closest_pt, closest_pt2);
	if ( dPlane1 < dPlane2 && dPlane1 < dPlane3) VcV(closest_pt, closest_pt1);

	return sqrt( nHeight*nHeight + dPlane*dPlane );
}
