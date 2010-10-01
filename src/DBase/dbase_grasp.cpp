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
// $Id: dbase_grasp.cpp,v 1.26 2010/08/11 21:35:22 cmatei Exp $
//
//######################################################################
/*! \file 
  \brief Defines the %DBaseBatchPlanner class
 */

#include "dbase_grasp.h"
#include "ivmgr.h"
#include "graspitGUI.h"
#include "world.h"
#include "searchState.h"
#include "egPlanner.h"
#include "guidedPlanner.h"
#include "body.h"
#include "robot.h"
#include "searchEnergy.h"
#include "loopPlanner.h"

#include "scanSimulator.h"

#include "debug.h"
#include <Inventor/sensors/SoTimerSensor.h>

DBaseBatchPlanner::DBaseBatchPlanner(IVmgr *mgr, GraspItGUI *gui)
{
	ivmgr = mgr;
	mGui = gui;
	mObject = NULL;
	mHand = NULL;
	mPlanner = NULL;
	mTimerSensor = NULL;
	numOfGrasps = 0;
	energyConstraint = -3.0;

}

DBaseBatchPlanner::~DBaseBatchPlanner()
{
	if (mPlanner) delete mPlanner;
	if (mTimerSensor) delete mTimerSensor;
	DBGAF(mLogStream,"Planner successfully deleted");
	mLogStream.close();
}

bool DBaseBatchPlanner::processArguments(int argc, char **argv)
{
	if (argc < 8) {
		DBGA("Not enough arguments to DBasePlanner");
		usage();
		return false;
	}

	if ( !strcmp(argv[1],"dbase") ) {
		mType = DEXTEROUS;
	} else if ( !strcmp(argv[1],"dbase_gripper") ) {
		mType = GRIPPER;
		if (argc <  9) {
			DBGA("Not enough arguments to DBasePlanner GRIPPER");
			usage();
			return false;
		}
	} else {
		DBGA("dbase specifier missing??");
		usage();
		return false;
	}

	//open logfile first so we can spit out errors
	mLogStream.open(argv[6],std::fstream::out | std::fstream::app);
	if (mLogStream.fail()) {
		DBGA("Failed to open log file " << argv[6]);
		return false;
	}

	QString filename;
	QString graspitRoot = QString(getenv("GRASPIT"));
	if (graspitRoot.isNull()) {
		DBGAF(mLogStream,"GRASPIT environment variable not set");
		return false;
	}

	filename = graspitRoot + QString("/models/robots/") + QString(argv[2]) + QString("/") 
		               + QString(argv[2]) + QString(".xml");
	mHand = (Hand*)ivmgr->getWorld()->importRobot(filename);
	if ( !mHand ) {
		DBGAF(mLogStream,"DBase planner: failed to load robot name " << argv[2] << ", or it is not a Hand");
		return false;
	}
	if (mHand->getNumVirtualContacts()==0) {
		DBGAF(mLogStream,"Specified hand does not have virtual contacts defined");
		DBGAF(mLogStream,"Hint: make sure capitalization in robot name is correct");
		DBGAF(mLogStream,"(i.e. HumanHand20DOF, Barrett)");
		getchar();
		return false;
	}
	filename = QString(argv[3]);
	mObject = static_cast<GraspableBody*>(ivmgr->getWorld()->importBody("GraspableBody",filename));
	((GraspableBody*)mObject)->showAxes(false);

	if (!mObject) {
		DBGAF(mLogStream,"Failed to load graspable body from file " << argv[3]);
		return false;
	}
	mMaxTime = atof(argv[4]);
	if (mMaxTime <=0) {
		DBGAF(mLogStream,"Wrong time budget specification: " << mMaxTime );
		return false;
	}
	DBGAF(mLogStream,"Time: " << mMaxTime << " sec");

	// define the number of grasps to look for
	numOfGraspsGoal = atoi(argv[7]);
	if (numOfGraspsGoal <=0) {
		DBGAF(mLogStream,"Wrong number of grasps specification: " << numOfGraspsGoal );
		return false;
	}
	DBGAF(mLogStream,"numOfGraspsGoal: " << numOfGraspsGoal << " grasps");

	energyConstraint = atof(argv[8]);
	DBGAF(mLogStream,"energyConstraint: " << energyConstraint);

	if (mType == GRIPPER) {
		mScanDirectory = new char[1000];
		strcpy(mScanDirectory, argv[9]);
		QString testfile = QString(mScanDirectory) + QString("scan_log.txt");
		FILE *tf = fopen(testfile.latin1(),"a");
		if (!tf) {
			DBGAF(mLogStream,"Failed scan directory test: " << testfile.latin1());
			return false;
		}
		fprintf(tf,"Starting object: %s\n",mObject->getName().latin1());
		fclose(tf);
	}

	if ( !strcmp(argv[5],"stderr") ) mResultFile = stderr;
	else if (!strcmp(argv[5],"stdout")) mResultFile = stdout;
	else mResultFile = fopen(argv[5],"w");
	if (!mResultFile) {
		DBGAF(mLogStream,"Failed to open result file " << argv[5]);
		return false;
	}
	DBGAF(mLogStream,"DBasePlanner init successfull. Robot " << argv[2] << " and object " << argv[3]);
	ivmgr->getViewer()->viewAll();
	return true;
}

void DBaseBatchPlanner::usage()
{
	DBGA("DataBaseBatch planner usage:");
	DBGA("  graspit dbase robot_name body_file time_budget_in_seconds result_file log_file max_num_grasps min_grasp_energy [scan_sim_dir]");
}

bool DBaseBatchPlanner::startPlanner()
{
	if (!mHand || !mObject || mMaxTime <= 0) {
		DBGA("Can not start batch planner; Hand or Object or Time not set");
		return false;
	}

	GraspPlanningState seed(mHand);
	seed.setObject(mObject);
	seed.setPositionType(SPACE_AXIS_ANGLE);
	seed.setPostureType(POSE_EIGEN);
	seed.setRefTran(mObject->getTran());
	seed.reset();

	//sim ann planner can be used for testing
	//mPlanner = new SimAnnPlanner(mHand);
	
	if (mType == DEXTEROUS) {
		//the real multi-threaded planner
		mPlanner = new GuidedPlanner(mHand);

		int numChildren;
		//I think for best results setMaxChildren(# of CPU cores - 1)
#ifdef WIN32
		QString numCPU = QString(getenv("NUMBER_OF_PROCESSORS"));
		if (numCPU.isNull()) {
			DBGAF(mLogStream,"NUMBER_OF_PROCESSORS env var. not specified; using 1 child thread");
			numChildren = 1;
		} else {
			numChildren = numCPU.toInt();
		}
		if (numChildren <= 0) {
			DBGAF(mLogStream,"Can not understand NUMBER_OF_PROCESSORS: " << numCPU.latin1());
		}
#else
		//on Linux we hard-code 2 CPU's for now
		numChildren = 2;
#endif
		
		if (numChildren <= 0) {
			numChildren = 1;
		} else {
			numChildren --;
			if (!numChildren) numChildren = 1;
		}
		
		//can be overriden here if you want to hard-code some other value in
		DBGAF(mLogStream,"Using up to " << numChildren << " child threads");
		((GuidedPlanner*)mPlanner)->setMaxChildren(numChildren);
	} else if (mType == GRIPPER) {
		//simple looping sim ann planner
		mPlanner = new LoopPlanner(mHand);
		mPlanner->setEnergyType(ENERGY_CONTACT);
	}

	mPlanner->setContactType(CONTACT_PRESET);
	mPlanner->setMaxSteps(65000);
	mPlanner->setRepeat(true);
	mPlanner->setMaxTime(mMaxTime);

	static_cast<SimAnnPlanner*>(mPlanner)->setModelState(&seed);
	if (!mPlanner->resetPlanner()) {
		DBGA("Failed to reset planner");
		return false;
	}

	QObject::connect(mPlanner, SIGNAL(update()), this, SLOT(plannerUpdate()));
	QObject::connect(mPlanner, SIGNAL(complete()), this, SLOT(plannerComplete()));
	mPlanner->startPlanner();
	return true;
}

void DBaseBatchPlanner::plannerUpdate()
{
	static int lastSolution = 0;
	bool newSol = false;

	//let's print solutions as they are discovered, so we don't lose all if there's a crash
	for (int i=lastSolution; i<mPlanner->getListSize();i++){
		processSolution(mPlanner->getGrasp(i));
		newSol = true;
	}

	if (newSol) {
		//if we seg-fault we don't want to lose what we have already written
		fflush(mResultFile);
		if (mType == DEXTEROUS) {
			//we don't need already saved solutions
			mPlanner->clearSolutions();
			lastSolution = 0;
		} else if (mType == GRIPPER) {
			//in this case we do need them as they will be avoided by future searches
			lastSolution = mPlanner->getListSize();
		}
	}
}

void DBaseBatchPlanner::plannerComplete()
{
	if (mPlanner->isActive()) {
		DBGAF(mLogStream,"Planner is not finished!");
		return;
	}
	//this prints the remainins solutions
	plannerUpdate();
	
	DBGAF(mLogStream,"Planner completed; starting shutdown");

	if (mType == GRIPPER) {
		ivmgr->getWorld()->destroyElement(mHand,false);
		fprintf(stderr,"Taking scans...\n");
		takeScans();
	}

	if (mResultFile!=stderr && mResultFile!=stdout) fclose(mResultFile);

	//schedule the sensor to give us the exit signal in 3 seconds
	//this should give the planner time to finish
	mTimerSensor = new SoTimerSensor(sensorCB, this);
	mTimerSensor->setInterval( SbTime( 3.0 ));
	mTimerSensor->schedule();
}

void DBaseBatchPlanner::sensorCB(void *data, SoSensor*)
{
	DBaseBatchPlanner *planner = (DBaseBatchPlanner*)data;
	GraspItGUI *gui = planner->mGui;
	DBGAF(planner->mLogStream,"Shutdown signal received");
	delete planner;
	gui->exitMainLoop();
}

void DBaseBatchPlanner::processSolution(const GraspPlanningState *s)
{
	//we will write this solution to the result file
	//if it's a poor solution don't even bother
	
	if (s->getEnergy() > energyConstraint){
		DBGAF(mLogStream,"Solution with energy to be thrown: " << s->getEnergy());
		return;
	}
	//we need a SearchEnergy calculator in order to do the autoGrasp in the exact same way that the planner does it
	static SearchEnergy *se = NULL; //don't create it each time
	if (!se) {
		se = new SearchEnergy();
		//this is the same type used by the loop planner
		switch(mType) {
		case DEXTEROUS:
			se->setType(ENERGY_STRICT_AUTOGRASP);
			break;
		case GRIPPER: 
			se->setType(ENERGY_CONTACT);
			se->setContactType(CONTACT_PRESET);
			break;
		}
	}
	DBGAF(mLogStream,"Solution with energy: " << s->getEnergy());
	//first, copy it to a new one so it's not const and we can modify it
	GraspPlanningState *sol = new GraspPlanningState(s);

	//convert it's tranform to the Quaternion__Translation format
	//make sure you pass it sticky=true, otherwise information is lost in the conversion
	sol->setPositionType(SPACE_COMPLETE,true);
	//we will want to save exact DOF positions, not eigengrasp values
	//again, make sure sticky=true
	sol->setPostureType(POSE_DOF,true);

	//we can write it to a file
	fprintf(mResultFile,"pre-grasp\n");
	sol->writeToFile(mResultFile);
	//at this point, we are saving each DOF individually, but it should be an eigengrasp posture

	if (mType == DEXTEROUS) {

		//now close the fingers and perform the autograsp
		bool legal; double energy;
		//make sure to pass it noChange = false and it will leave the hand in the posture AFTER the autograsp
		//otherwise, analyzeState will revert the state to what it was on its entry
		se->analyzeState(legal,energy,sol,false);
		if (!legal) {
			DBGAF(mLogStream,"buru Illegal solution! This should not be!");
		}
		//store the hand posture resulting from the autograsp
		//careful: this only works if sol is of the type SPACE_COMPLETE (as set above)
		sol->saveCurrentHandState();
		
		//write this one to the file as well
		fprintf(mResultFile,"grasp\n");
		sol->writeToFile(mResultFile);
		//this posture is probably not in eigengrasp space
		
		//write the contacts to a file
		fprintf(mResultFile,"contacts\n");
		writeContactsToFile(sol->getHand(), sol->getObject());
	}

	numOfGrasps++;
	printf("\nup to now, %d out of %d grasps have been found\n",numOfGrasps, numOfGraspsGoal);

	//add some codes here to limit the grasps
	if(numOfGrasps == numOfGraspsGoal)
	{
		mMaxTime = 10;// this is a trick, maybe not good
		printf("begin to leave...");
		DBGAF(mLogStream, numOfGraspsGoal);
		mPlanner->setMaxTime(mMaxTime);
	}

	//we are done
	delete sol;
}

void DBaseBatchPlanner::writeContactsToFile(Hand *hand, Body *object)
{
  int f,l;
  std::list<Contact *>::iterator cp;
  std::list<Contact *> contactList;

  contactList = hand->getPalm()->getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++){
    if ((*cp)->getBody2()==object) {
		//should really use streams and let contacts write themselves instead of these hacks....
		position pos = (*cp)->getMate()->getPosition();
		fprintf(mResultFile,"%f %f %f\n",pos.x(), pos.y(), pos.z());
    }
  }

  for(f=0;f<hand->getNumFingers();f++) {
    for (l=0;l<hand->getFinger(f)->getNumLinks();l++) {
      contactList = hand->getFinger(f)->getLink(l)->getContacts();
      for (cp=contactList.begin();cp!=contactList.end();cp++){
		if ((*cp)->getBody2()==object) {
			position pos = (*cp)->getMate()->getPosition();
			fprintf(mResultFile,"%f %f %f\n",pos.x(), pos.y(), pos.z());
		}
	  }
	}
  } 
  //fprintf(mResultFile,"\n");
}

/* This function simulates a number of scans around the object, then writes the scans to files 
   along with all the grasps found
*/
void DBaseBatchPlanner::takeScans()
{
	ScanSimulator sim;

	sim.setOptics(-45.0 , 45.0 , 400 ,
		      -45.0 , 45.0 , 400 );
	sim.setType(ScanSimulator::WORLD_COORDINATES);

	int numAltitudes = 3;
	float altitudes[3] = {(float)M_PI/6.0f, (float)M_PI/3.0f, (float)M_PI/2.0f };
	int numSamples[3] = {8, 4, 1};

	float distance = 1000; //1 meter

	std::vector<position> cloud;
	std::vector<RawScanPoint> rawData;

	for (int i=0; i<numAltitudes; i++) {
		float theta = altitudes[i];
		float phi_step = M_PI / numSamples[i];

		for (int j=0; j<numSamples[i]; j++) {
			float phi = phi_step * j;
			
			vec3 loc( distance * cos(theta) * cos(phi), 
				  distance * cos(theta) * sin(phi),
				  distance * sin(theta) );
			//looking back towards the origin
			vec3 dir = -1 * normalise(loc);
			vec3 up(0,0,1);
			sim.setPosition(loc, dir, up, ScanSimulator::STEREO_CAMERA);

			cloud.clear();
			rawData.clear();

			sim.scan(&cloud, &rawData);

			writeCloudToFile(i, j, cloud);
			vec3 foo, trueUp;
			position foop;
			sim.getPosition(foop,foo,trueUp);
			writeRawToFile(i, j, rawData, loc, dir, trueUp);
		}
	}
}

void DBaseBatchPlanner::writeCloudToFile( int i, int j, const std::vector<position> &cloud)
{
	QString filename = QString(mScanDirectory) + mObject->getName() + QString("_cloud");
	QString n;
	n.setNum(i);
	filename = filename + n;
	n.setNum(j);
	filename = filename + QString("_") + n + QString(".txt");

	FILE *f = fopen(filename.latin1(),"w");
	if (!f) {
		DBGAF(mLogStream,"Failed to open file " << filename.latin1());
		fprintf(stderr,"Failed to open scan file\n");
		return;
	}

	fprintf(f,"%d\n",(int)cloud.size());
	for (int k=0; k<(int)cloud.size(); k++) {
		fprintf(f,"%f %f %f -1\n",cloud[k].x(), cloud[k].y(), cloud[k].z());
	}

	writeSolutionsToFile(f);
	fclose(f);
}

void DBaseBatchPlanner::writeRawToFile( int i, int j, const std::vector<RawScanPoint> &rawData, 
					vec3 loc, vec3 dir, vec3 up)
{
	QString filename = QString(mScanDirectory) + mObject->getName() + QString("_raw");
	QString n;
	n.setNum(i);
	filename = filename + n;
	n.setNum(j);
	filename = filename + QString("_") + n + QString(".txt");

	FILE *f = fopen(filename.latin1(),"w");
	if (!f) {
		DBGAF(mLogStream,"Failed to open file " << filename.latin1());
		fprintf(stderr,"Failed to open scan file\n");
		return;
	}

	fprintf(f,"%f %f %f\n",loc.x(), loc.y(), loc.z());
	fprintf(f,"%f %f %f\n",dir.x(), dir.y(), dir.z());
	fprintf(f,"%f %f %f\n",up.x(), up.y(), up.z());
	fprintf(f,"%d\n",(int)rawData.size());

	for (int k=0; k<(int)rawData.size(); k++) {
		fprintf(f,"%f %f ",rawData[k].hAngle, rawData[k].vAngle);
		fprintf(f,"%f %f %f ",rawData[k].dx, rawData[k].dy, rawData[k].dz);
		fprintf(f,"%f\n",rawData[k].distance);
	}

	writeSolutionsToFile(f);
	fclose(f);
}

void DBaseBatchPlanner::writeSolutionsToFile(FILE *f)
{
	//hacking reaches new lows
	numOfGrasps = 0;
	FILE *temp = mResultFile;
	mResultFile = f;
	for (int i=0; i<mPlanner->getListSize();i++){
		processSolution(mPlanner->getGrasp(i));
	}
	mResultFile = temp;

}
