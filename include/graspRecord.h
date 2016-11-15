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
// $Id: graspRecord.h,v 1.6 2009/04/21 14:53:07 cmatei Exp $
//
//######################################################################

#ifndef _GRASPRECORD_H_
#define _GRASPRECORD_H_

#include <QString>
#include <vector>

#include "matvec3D.h"
#include "gloveInterface.h"

class CalibrationPose;

//! Obsolete class used for saving a hand pose and posture for a grasp of an object
/*!	This class records a grasp. It just knows a hand pose and a transform.
	It uses the CalibrationPose to record the pose, but makes use of very 
	little from that class. It keeps the pose in DOF space, not in 
	EigenGrasp space.

	This class is obsolete, only used by the GraspCaptureDlg. It is in a very
	poor state, not recommended for use.
*/
class GraspRecord
{
private:
	int mSize;
public:
	GraspRecord(int size);
	~GraspRecord();

	CalibrationPose *mPose;
	QString mObjectName;
	QString mRobotName;
	transf mTran;

	//modified to DBase project
	double quality;
	int originalIndex;

	void writeToFile(FILE *fp);
	void readFromFile(FILE *fp);
};

void loadGraspListFromFile(std::vector<GraspRecord*> *list, const char *filename);
void writeGraspListToFile (std::vector<GraspRecord*> *list, const char *filename);

#endif
