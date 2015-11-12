/**********************************
Tri3B.cpp
Claire Lackner
takes the Tri structure from PQP and expands it to a class 
that contains the normal vector to the triangle
the unit vector along each side of the triangle
the normal in the plane of the triangle to each side of the triangle
the class is in Tri.h
This implements the constructor/destructor and the move method
***********************************/
#include "Tri.h"
#include <string.h>
#include <iostream>

Tri3B::Tri3B( )
{
	//sets all vercotrs to the zero vector and id to zero

	id = 0;
	Videntity( p1 );
	Videntity( p2 );
	Videntity( p3 );
	Videntity( s12 );
	Videntity( s23 );
	Videntity( s31 );
	Videntity( ns12 );
	Videntity( ns23 );
	Videntity( ns31 );
	Videntity( n );

}

Tri3B::Tri3B( const PQP_REAL *pt1, const PQP_REAL *pt2, 
			 const PQP_REAL *pt3, int iden )
{
	Set( pt1, pt2, pt3, iden );	
}

//sets all the members of Tri3B given the values of the vertices
void Tri3B::Set( const PQP_REAL *pt1, const PQP_REAL *pt2, 
				const PQP_REAL *pt3, int iden )
{
	id = iden;

	//vertices of triangle in model's coord. frame
	p1[0] = pt1[0];
	p1[1] = pt1[1];
	p1[2] = pt1[2];
	p2[0] = pt2[0];
	p2[1] = pt2[1];
	p2[2] = pt2[2];
	p3[0] = pt3[0];
	p3[1] = pt3[1];
	p3[2] = pt3[2];

	//normalized vectors along side of triangle
	VmV( s12, p2, p1 );
	Vnormalize( s12 );

	VmV( s23, p3, p2 );
	Vnormalize( s23 );

	VmV( s31, p1, p3 );
	Vnormalize( s31 );

	//normal vector to plane of triangle
	VcrossV( n, s31, s12 );
	Vnormalize( n );

	//normal vectors to each side in the plane of the triangle
	//from cross product of side-vector and normal
	VcrossV( ns12, s12, n );
	Vnormalize( ns12 );

	VcrossV( ns23, s23, n );
	Vnormalize( ns23 );

	VcrossV( ns31, s31, n );
	Vnormalize( ns31 );	
}
//moves a triangle
void Tri3B::Move( const PQP_REAL *pt1, const PQP_REAL *pt2, 
				 const PQP_REAL *pt3 )
{
	//vertices of triangle in model's coord. frame
	p1[0] = pt1[0];
	p1[1] = pt1[1];
	p1[2] = pt1[2];
	p2[0] = pt2[0];
	p2[1] = pt2[1];
	p2[2] = pt2[2];
	p3[0] = pt3[0];
	p3[1] = pt3[1];
	p3[2] = pt3[2];

	//normalized vectors along side of triangle
	VmV( s12, p2, p1 );
	Vnormalize( s12 );

	VmV( s23, p3, p2 );
	Vnormalize( s23 );

	VmV( s31, p1, p3 );
	Vnormalize( s31 );

	//normal vector to plane of triangle
	VcrossV( n, s31, s12 );
	Vnormalize( n );

	//normal vectors to each side in the plane of the triangle
	//from cross product of side-vector and normal
	VcrossV( ns12, s12, n );
	Vnormalize( ns12 );

	VcrossV( ns23, s23, n );
	Vnormalize( ns23 );

	VcrossV( ns31, s31, n );
	Vnormalize( ns31 );	
}

//cnl-ExtractPts function takes a list of triangles and extracts the points from that list
//it is used to return points from list of triangles returned by PQP_Model::GetNghbd()
//does not remove duplicates

//Resize resizes the array of points to the given parameters
//cnl - not used
void Resize( double **ptList, int oldSize, int newSize )
{
	double **temp;
	temp = new double *[newSize];

	for( int j = 0; j < oldSize; j++ )
	{
		temp[j] = new double[3];

		temp[j][0] = ptList[j][0];
		temp[j][1] = ptList[j][1];
		temp[j][2] = ptList[j][2];
	}
	delete[] ptList;
	ptList = temp;
}

//RemoveCopies removes extra copies of a point from a list
void RemoveCopies( std::vector< PQP_Vec > *ptList )
{
	//sort the list
	sort( ptList->begin(), ptList->end() );

	std::vector< PQP_Vec >::iterator itr1, itr2;

	itr1 = ptList->begin();

	if ( (int)ptList->size() < 2 ) return;

	while( itr1 < ( ptList->end() - 1) )
	{
		itr2 = itr1 + 1;
		while( itr2 != ptList->end() && *itr1 == *itr2 )
		{
			itr2 = ptList->erase( itr2 );
		}
		itr1++;
	}
}

void ExtractPts( TriPtrList *tris, std::vector< PQP_Vec > *ptList )
{
   	TriPtrList::iterator itTri;
	Tri3B *crTri;
	int count = 0;
	PQP_Vec temp1, temp2, temp3;

	ptList->clear();

	for( itTri = tris->begin(); itTri != tris->end(); itTri++ )
	{
		crTri = *itTri;

		for( int i = 0; i < 3; i++ )
		{
			temp1.d[i] = crTri->p1[i];
			temp2.d[i] = crTri->p2[i];
			temp3.d[i] = crTri->p3[i];
		}

		ptList->push_back( temp1 );
		ptList->push_back( temp2 );
		ptList->push_back( temp3 );

		count += 3;

	}

//	std::cerr<<"number of points in neighborhood before "<<count<<std::endl;
	
	RemoveCopies( ptList );


//	std::cerr<<"number of points in neighborhood after "<<ptList->size()<<std::endl;
}