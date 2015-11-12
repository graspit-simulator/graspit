//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: scanSimulator.h,v 1.5 2009/04/01 13:52:34 cmatei Exp $
//
//######################################################################

#ifndef _scansimulator_h_
#define _scansimulator_h_

#include <vector>

#include "matvec3D.h"

//QT defines DEBUG as a macro, which interferes with ROS when this is used
#undef DEBUG

/*! A raw scan ray information, defining a ray (as horizontal and vertical
	angle from the scanner reference position, and the distance along the 
	ray that an object was hit.
*/
struct RawScanPoint {
	float hAngle,vAngle;
	float dx, dy, dz;
	float distance;
};

/*! A scan simulator can simulate a laser scanner scanning the GraspIt world.
	The virtual scanner is placed at a position in the world and some scanning
	parameters are set. It then uses Coin's ray intersection to simulate the
	scan, and returns the resulting point cloud of whatever was in the GraspIt
	world in its field of view. It can also return the "raw" scan data, with
	each ray and the hit distance along the ray, rather than a point cloud.

	The results can be returned either in GraspIt world coordinates, or in 
	scanner coordinates.
*/
class ScanSimulator {
 public:
	enum Type{SCANNER_COORDINATES,WORLD_COORDINATES};

 private:
	 //! The location of the scanner in the GraspIt world
	position mPosition;
	//! The orientation of the scanner, what it is aimed at
	vec3 mDirection;
	//! The "up" direction of the scanner
	vec3 mUp;
	//! The "horizontal" direction; together with "up" and "direction" forms a right-handed coord system.
	vec3 mHorizDirection;

	//! The scanner's transform in the GraspIt world
	/*! Set so that the z axis is pointing "up" and the x axis id pointing
		in the scanning direction.*/
	transf mTran;
	//! The inverse of the scanner tranforms, precomputed and stored to save time
	transf mTranInv;

	Type mType;

	float mHMin, mHMax;
	int mHLines; 
	float mVMin, mVMax;
	int mVLines;

	bool shootRay(const vec3 &rayDirection, position &rayPoint);
	void computeTransform();
 public:
	ScanSimulator();
	void setPosition(position p, vec3 v, vec3 u);
	void setPosition(vec3 p, vec3 v, vec3 u){position pp(p.x(), p.y(), p.z()); setPosition(pp,v,u);}
	void getPosition(position &p, vec3 &v, vec3 &u){
		p = mPosition; v = mDirection; u = mUp;}
	void setOptics( float hMin, float hMax, int hLines, 
			float vMin, float vMax, int vLines) {
		mHMin = hMin; mHMax = hMax; mHLines = hLines;
		mVMin = vMin; mVMax = vMax; mVLines = vLines;}
	void setType(Type t){mType=t;}
	Type getType() const {return mType;}
	void computeRayDirection(float hAngle, float vAngle, vec3 &rayDirection);

	//! The main function for scanning. 
	/*! Returns the full result as a point cloud and, if wanted 
		(\a rawData is not NULL), also as raw data. */
	void scan(std::vector<position> *cloud, std::vector<RawScanPoint> *rawData = NULL);
};

#endif
