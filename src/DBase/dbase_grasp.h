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
class GraspitDBGrasp;

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
	bool completed;
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
	//the vector that stores the output grasps
	std::vector<GraspitDBGrasp*> mGraspList;
	//indicates whether to store the output grasps to CGDB
	bool mToStore;
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
	bool exitSignalSet;

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
	bool isActive();
	bool processArguments(int argc, char **argv, GraspableBody * gb, Hand * h);
	void setExitSignalSlot(){exitSignalSet=true;};
signals:
	void plannerExit();

};

#endif
