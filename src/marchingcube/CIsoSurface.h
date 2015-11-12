#ifndef CISOSURFACE_H
#define CISOSURFACE_H
// File Name: CIsoSurface.h
// Last Modified: 5/8/2000
// Author: Raghavendra Chandrashekara (basesd on source code
// provided by Paul Bourke and Cory Gene Bloyd)
// Email: rc99@doc.ic.ac.uk, rchandrashekara@hotmail.com
//
// Description: This is the interface file for the CIsoSurface class.
// CIsoSurface can be used to construct an isosurface from a scalar
// field.

#include <map>
#include <vector>
#include "Vectors.h"

struct POINT3DID {
	unsigned int newID;
	float x, y, z;
};

typedef std::map<unsigned int, POINT3DID> ID2POINT3DID;

struct TRIANGLE {
	unsigned int pointID[3];
};

typedef std::vector<TRIANGLE> TRIANGLEVECTOR;

template <class T> class CIsoSurface {
public:
	// Constructor and destructor.
	CIsoSurface();
	~CIsoSurface();
	
	// Generates the isosurface from the scalar field contained in the
	// buffer ptScalarField[].
	void GenerateSurface(const T* ptScalarField, T tIsoLevel, unsigned int nCellsX, unsigned int nCellsY,  unsigned int nCellsZ, float fCellLengthX, float fCellLengthY, float fCellLengthZ);

	// Returns true if a valid surface has been generated.
	bool IsSurfaceValid();

	// Deletes the isosurface.
	void DeleteSurface();

//	TRIANGLEVECTOR getMesh() {return

	// Returns the length, width, and height of the volume in which the
	// isosurface in enclosed in.  Returns -1 if the surface is not
	// valid.
	int GetVolumeLengths(float& fVolLengthX, float& fVolLengthY, float& fVolLengthZ);

//protected:
	// The number of vertices which make up the isosurface.
	unsigned int m_nVertices;

	// The vertices which make up the isosurface.
	POINT3D* m_ppt3dVertices;

	// The number of triangles which make up the isosurface.
	unsigned int m_nTriangles;

	// The indices of the vertices which make up the triangles.
	unsigned int* m_piTriangleIndices;

	// The number of normals.
	unsigned int m_nNormals;

	// The normals.
	VECTOR3D* m_pvec3dNormals;

	// List of POINT3Ds which form the isosurface.
	ID2POINT3DID m_i2pt3idVertices;

	// List of TRIANGLES which form the triangulation of the isosurface.
	TRIANGLEVECTOR m_trivecTriangles;

	// Returns the edge ID.
	unsigned int GetEdgeID(unsigned int nX, unsigned int nY, unsigned int nZ, unsigned int nEdgeNo);

	// Returns the vertex ID.
	unsigned int GetVertexID(unsigned int nX, unsigned int nY, unsigned int nZ);

	// Calculates the intersection point of the isosurface with an
	// edge.
	POINT3DID CalculateIntersection(unsigned int nX, unsigned int nY, unsigned int nZ, unsigned int nEdgeNo);

	// Interpolates between two grid points to produce the point at which
	// the isosurface intersects an edge.
	POINT3DID Interpolate(float fX1, float fY1, float fZ1, float fX2, float fY2, float fZ2, T tVal1, T tVal2);
 
	// Renames vertices and triangles so that they can be accessed more
	// efficiently.
	void RenameVerticesAndTriangles();

	// Calculates the normals.
	void CalculateNormals();

	// No. of cells in x, y, and z directions.
	unsigned int m_nCellsX, m_nCellsY, m_nCellsZ;

	// Cell length in x, y, and z directions.
	float m_fCellLengthX, m_fCellLengthY, m_fCellLengthZ;

	// The buffer holding the scalar field.
	const T* m_ptScalarField;

	// The isosurface value.
	T m_tIsoLevel;

	// Indicates whether a valid surface is present.
	bool m_bValidSurface;

	// Lookup tables used in the construction of the isosurface.
	static const unsigned int m_edgeTable[256];
	static const unsigned int m_triTable[256][16];
};
#endif // CISOSURFACE_H

