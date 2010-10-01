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
// Author(s):  Matei T. Ciocarlie
//
// $Id: dbase_grasp.h,v 1.15 2009/05/08 23:41:43 hao Exp $
//
//######################################################################
/*! \file 
  \brief Defines the %DBaseBatchPlanner class
 */

#ifndef _DBASE_GRASP_H_
#define _DBASE_GRASP_H_

#include "stdio.h"
#include "qobject.h"
#include <fstream>

#include "scanSimulator.h"

class IVmgr;
class EGPlanner;
class Hand;
class Body;
class GraspableBody;
class GraspItGUI;
class GraspPlanningState;
class SoSensor;
class SoTimerSensor;

class DBaseBatchPlanner : public QObject
{
	Q_OBJECT
private:
	// a dexterous search will use child threads with autograsp energy
	// it is geared for multi-fingered hands
	// a gripper search is geared for a simpler SimAnn search even in the child threads
	// as it cares more about positioning the gripper than force-closure
	// (because for now force-closure is hard to compute correctly with only 2 contacts)
	enum Type{DEXTEROUS=0, GRIPPER=1};
	
	Type mType;
	GraspableBody *mObject;
	Hand *mHand;
	IVmgr *ivmgr;
	GraspItGUI *mGui;
	EGPlanner *mPlanner;
	//maybe one day we'll use streams here...
	FILE *mResultFile;
	//yes - breaking new ground! stream!
	std::fstream mLogStream;
	char *mScanDirectory;
	double mMaxTime;
	void usage();
	void processSolution(const GraspPlanningState *sol);
	void writeContactsToFile(Hand *hand, Body *object);
	void takeScans();
	static void sensorCB(void *data,SoSensor*);
	SoTimerSensor *mTimerSensor;
	int numOfGrasps;
	int numOfGraspsGoal;
	double energyConstraint;

	void writeCloudToFile( int i, int j, const std::vector<position> &cloud);
	void writeRawToFile( int i, int j, const std::vector<RawScanPoint> &rawData, 
			     vec3 loc, vec3 dir, vec3 up);
	void writeSolutionsToFile(FILE *f);


public slots:
	//this slot gets called every time the inner planner calls update
	void plannerUpdate();
	//this one gets called when the inner planner stops
	void plannerComplete();
public:
	DBaseBatchPlanner(IVmgr *mgr, GraspItGUI *gui);
	~DBaseBatchPlanner();
	bool processArguments(int argc, char **argv);
	bool startPlanner();
};

#endif
