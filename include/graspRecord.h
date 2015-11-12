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
