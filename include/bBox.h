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
// $Id: bBox.h,v 1.3 2009/04/21 14:53:07 cmatei Exp $
//
//######################################################################

#ifndef _bbox_h_
#define _bbox_h_

#include "matvec3D.h"
#include "mytools.h"

#include <iostream>

/*! A bounding box. Transform holds the translation to the center of the box and 
	the rotation to make the box axes align with coordinate axes. HalfSize is the extents
	of half of the box along each axis.
*/
class BoundingBox
{
private:
	transf mTran;
	transf mTranInv;
public:
	vec3 halfSize;
	//for debugging purposes
	mutable bool mMark;
	BoundingBox(const transf &t, const vec3 &v) : mTran(t), mTranInv(t.inverse()), 
												  halfSize(v), mMark(false) {}
	BoundingBox(const BoundingBox &b) : mTran(b.getTran()), mTranInv(b.getTranInv()), 
										halfSize(b.halfSize), mMark(b.mMark) {}
	BoundingBox() : mTran(), mTranInv(), halfSize(), mMark(false){}
	INLINE_RELEASE void applyTransform(const transf &t);
	void setTran(const transf &t) {
		mTran = t;
		mTranInv = mTran.inverse();
	}
	const transf& getTran() const {return mTran;}
	const transf& getTranInv() const {return mTranInv;}
};

INLINE_RELEASE bool 
bboxOverlap(const BoundingBox &bb1, const BoundingBox &bb2, const transf &tran2To1);

INLINE_RELEASE double 
bboxDistanceSq(const BoundingBox &bb1, const BoundingBox &bb2, const transf &tran2To1);

INLINE_RELEASE double 
bboxDistanceApp(const BoundingBox &bb1, const BoundingBox &bb2);

INLINE_RELEASE double 
pointBoxDistanceSq(const BoundingBox &bbox, const position &p);

INLINE_RELEASE
position closestPtBbox(const BoundingBox &bbox, const position &p);

#ifdef GRASPIT_RELEASE
#include "bbox_inl.h"
#endif

#endif
