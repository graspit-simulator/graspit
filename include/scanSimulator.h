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
// $Id: scanSimulator.h,v 1.6 2010/08/11 21:35:22 cmatei Exp $
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
        //! Convention for specifying the scanner's axes relative to the optical frame and "up"
        //! directions. Useful when specifying the scanner position as a transform, or when requesting
        //! the scan back in scanner coordinates.
        enum AxesConvention{
          //! positive Z is optical axis, Y is "down"
          STEREO_CAMERA
        };

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
 public:
	ScanSimulator();
        void setPosition(transf tr, AxesConvention convention);
	void setPosition(position p, vec3 optical_axis, vec3 up_axis, AxesConvention convention);
	void setPosition(vec3 p, vec3 optical_axis, vec3 up_axis, AxesConvention convention){
          position pp(p.x(), p.y(), p.z()); 
          setPosition(pp, optical_axis, up_axis, convention);
        }
	void getPosition(position &p, vec3 &optical_axis, vec3 &up_axis){
		p = mPosition; optical_axis = mDirection; up_axis = mUp;
        }
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
