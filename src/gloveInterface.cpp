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
// $Id: gloveInterface.cpp,v 1.14 2009/05/04 14:28:11 cmatei Exp $
//
//######################################################################

#include "gloveInterface.h"

#include "robot.h"

#ifdef HARDWARE_LIB
#include "CyberGlove.h"
#else
#define N_SENSOR_VALUES 24
#endif

#include "world.h"
#include "matvec3D.h"
#include "jacobian.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include "debug.h"

//--------------CalibrationPose------------
void CalibrationPose::init(int size)
{
	if (size < 0) {
		fprintf(stderr,"Wrong size of calibration pose\n");
		mSize = 0;
	} else {
		mSize = size;
	}
	if (mSize>0) {
		jointValues = new double[size];
		sensorValues = new int[size];
		sensorMap = new int[size];
	}
	recordedDistance = 0;
	mSize = size;
	jointsSet = false;
	sensorsSet = false;
	mapSet = false;
	poseSet = false;
	mTransf = transf::IDENTITY;
}

CalibrationPose::CalibrationPose(int size)
{
	init(size);
}

CalibrationPose::CalibrationPose(int size, double *joints, int *map)
{
	init(size);
	setAllJointValues(joints);
	for (int i=0; i<size; i++) {
		jointValues[i] = jointValues[i] * 3.14159 / 180.0;
	}
	setAllMaps(map);
}

CalibrationPose::~CalibrationPose()
{
	delete [] jointValues;
	delete [] sensorValues;
	delete [] sensorMap;
}

void CalibrationPose::readFromFile(FILE *fp)
{
	int val;
	float floatVal;
	int j;

	if(fscanf(fp,"%d",&mSize) <=0){
	  DBGA("CalibrationPose::readFromFile - Failed to read calibration size");	 
	  return;
	}
	init(mSize);

	if(fscanf(fp,"%f",&floatVal) <= 0) {
	  DBGA("CalibrationPose::readFromFile - Failed to read recorded distance");	 
	  return;
	}
	recordedDistance = floatVal;
	if(fscanf(fp,"%d",&val)) {
	  DBGA("CalibrationPose::readFromFile - Failed to read sensor number");	 
	  return;
	}
	
	if (val) {
	  for (j=0; j<mSize; j++) {
	    if(fscanf(fp,"%d",&val) <= 0) {
	      DBGA("CalibrationPose::readFromFile - Failed to read sensor value");	 
	      return;
	    }
	    setSensorValue(j,val);
	  }
	  sensorsSet = true;
	} else sensorsSet = false;

	if (fscanf(fp,"%d",&val) <= 0) {
	  DBGA("CalibrationPose::readFromFile - Failed to read map size");	 
	  return;
	}
	
	if (val) {
	  for (j=0; j<mSize; j++) {
	    if(fscanf(fp,"%d",&val) <= 0) {
	      DBGA("CalibrationPose::readFromFile - Failed to read map value");	 
	      return;
	    }
	    
	    setMap(j,val);	
	  }
	  mapSet = true;
	} 
	else mapSet = false;
	
	if(fscanf(fp,"%d",&val) <= 0) {
	  DBGA("CalibrationPose::readFromFile - Failed to read joint number");	 
	  return;
	}
	
	if (val) {
	  for (j=0; j<mSize; j++) {
	    if(fscanf(fp,"%f",&floatVal) >= 0) {
	      DBGA("CalibrationPose::readFromFile - Failed to read joint value");	 
	      return;
	    }
	    
	    setJointValue(j,floatVal);
	  }
	  jointsSet = true;
	} else jointsSet = false;
	
	
	//transform rotation
	float x,y,z,w;
	if(fscanf(fp,"%f %f %f %f",&x, &y, &z, &w)) {
	  DBGA("CalibrationPose::readFromFile - Failed to read calibration orientation");	 
	  return;
	}
	Quaternion q(w,x,y,z);
	if(fscanf(fp,"%f %f %f",&x,&y,&z) <= 0) {
	  DBGA("CalibrationPose::readFromFile - Failed to read calibration location");	 
	  return;
	}
	
	vec3 t(x,y,z);
	mTransf.set(q,t);
}

void CalibrationPose::writeToFile(FILE *fp)
{
	fprintf(fp,"%d\n",getSize());
	fprintf(fp,"%f\n",recordedDistance);

	if ( sensorsSet ) {
		fprintf(fp,"1\n");
		for (int i=0; i<getSize(); i++)
			fprintf(fp,"%d ",getSensorValue(i));
		fprintf(fp,"\n");
	} else fprintf(fp,"0\n");
	if (mapSet) {
		fprintf(fp,"1\n");
		for (int i=0; i<getSize(); i++)
			fprintf(fp,"%d ",getMap(i) );
		fprintf(fp,"\n");			
	} else fprintf(fp,"0\n");
	if (jointsSet) {
		fprintf(fp,"1\n");
		for (int i=0; i<getSize(); i++)
			fprintf(fp,"%f ",getJointValue(i) );
		fprintf(fp,"\n");			
	} else fprintf(fp,"0\n");
	//transform rotation
	fprintf(fp,"%f %f %f %f\n",mTransf.rotation().x, mTransf.rotation().y, 
		    mTransf.rotation().z, mTransf.rotation().w);
	//transform translation
	fprintf(fp,"%f %f %f\n",mTransf.translation().x(), mTransf.translation().y(), 
			mTransf.translation().z());
}

//! Loads a list of calibration poses from a file
/*! Helper function loads all the calibration poses found in a file and 
	stores them in a given list of poses. The poses have usually been
	save using \writePoseListToFile(...)
*/
void loadPoseListFromFile(std::list<CalibrationPose*> *list, const char *filename)
{
	FILE *fp = fopen(filename,"r");
	if (!fp) {
		fprintf(stderr,"Unable to open calibration file!\n");
		return;
	}

	int nPoses;
	CalibrationPose *pose;
	if(fscanf(fp,"%d",&nPoses) <= 0) {
	  DBGA("loadPoseListFromFile - Failed to read number of poses");	 
	  return;
	
	}
	fprintf(stderr,"Total of %d poses\n",nPoses);
	for (int i=0; i<nPoses; i++) {
		pose = new CalibrationPose(0);
		pose->readFromFile(fp);
		list->push_back(pose);
	}
	fclose(fp);
}

//! Saves a list of poses to a file
/*! The poses can be read back by loadPosesListFromFile(...)*/
void writePoseListToFile(std::list<CalibrationPose*> *list, const char *filename)
{
	FILE *fp = fopen(filename,"w");
	if (!fp) {
		fprintf(stderr,"Unable to open calibration file!\n");
		return;
	}
	fprintf(fp,"%d\n",list->size());
	std::list<CalibrationPose*>::iterator it;
	for (it = list->begin(); it!=list->end(); it++) {
		(*it)->writeToFile(fp);
	}
	fprintf(stderr,"Calibration poses saved\n");
	fclose(fp);
}

bool CalibrationPose::isSet()
{
	return poseSet;
}

void CalibrationPose::setAllJointValues(double *jv)
{
	for (int i=0; i<mSize; i++) {
		jointValues[i] = jv[i];
	}
	jointsSet = true;
	if (sensorsSet && mapSet) poseSet = true;
}

void CalibrationPose::setJointValue(int j, double jv)
{
	jointValues[j] = jv;
	jointsSet = true;
}

void CalibrationPose::setAllSensorValues(int *sv)
{
	for (int i=0; i<mSize; i++) {
		sensorValues[i] = sv[i];
	}
	sensorsSet = true;
	if (jointsSet && mapSet) poseSet = true;
}

void CalibrationPose::setSensorValue(int i, int sv)
{
	if (i>=mSize) {
		fprintf(stderr,"Error attempting to set calibration pose sensor value\n");
		return;
	}
	sensorValues[i] = sv;
	sensorsSet = true;
}

void CalibrationPose::setAllMaps(int *m)
{
	for (int i=0; i<mSize; i++) {
		sensorMap[i] = m[i];
	}
	mapSet = true;
	if (jointsSet && sensorsSet) poseSet = true;
}

void CalibrationPose::setMap(int i, int mv)
{
	if (i>=mSize) {
		fprintf(stderr,"Error attempting to set calibration pose map value\n");
		return;
	}
	sensorMap[i] = mv;
	mapSet = true;
}

//--------------------------------Conversion Data -------------------------------

CData::CData(int nd, int ns)
{
	nDOF = nd;
	nSensors = ns;
	slopes = new double[nDOF*nSensors];
	intercepts = new double[nDOF];
	reset();
}

CData::~CData()
{
	delete [] slopes;
	delete [] intercepts;
}

void CData::reset()
{
	int i;
	for (i=0; i<nDOF*nSensors; i++)
		slopes[i] = 0;
	for (i=0; i<nDOF; i++)
		intercepts[i] = 0;
}

void CData::setSlope(int d, int s, double val)
{
	if ( d>= nDOF || s >= nSensors) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return;
	}
	slopes[d+s*nDOF] = val;
}

void CData::setIntercept(int d, double val)
{
	if ( d >= nDOF ) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return;
	}
	intercepts[d] = val;
}

void CData::addToSlope(int d, int s, double val)
{
	if ( d>= nDOF || s >= nSensors) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return;
	}
	slopes[d+s*nDOF] += val;
}

void CData::addToIntercept(int d, double val)
{
	if ( d >= nDOF ) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return;
	}
	intercepts[d] += val;
}
double CData::getSlope(int d, int s)
{
	if ( d>= nDOF || s >= nSensors) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return 0;
	}
	return slopes[d+s*nDOF];
}

double CData::getIntercept(int d)
{
	if ( d >= nDOF ) {
		fprintf(stderr,"Wrong addressing in Conversion Data\n");
		return 0;
	}
	return intercepts[d];
}

//---------------------------------GloveInterface--------------------------------

GloveInterface::GloveInterface(Robot *robot)
{
	mRobot = robot;
	rawGlove = NULL;

	savedDOFVals = new double[mRobot->getNumDOF()];
	mRobot->getDOFVals(savedDOFVals);

	mData = new CData( mRobot->getNumDOF(), N_SENSOR_VALUES);
	currentPose = cPoses.begin();
}

GloveInterface::~GloveInterface()
{
	delete mData;
	delete [] savedDOFVals;
}

void GloveInterface::setGlove(CyberGlove *glove)
{
	rawGlove = glove;
}

void GloveInterface::saveRobotPose()
{
	mRobot->getDOFVals(savedDOFVals);
}

void GloveInterface::revertRobotPose()
{
	mRobot->forceDOFVals(savedDOFVals);
}

int GloveInterface::instantRead()
{
#ifdef HARDWARE_LIB
	if (!rawGlove)
		return 0;
	if (!rawGlove->instantRead() )
		return 0;
	return 1;
#else
	return 0;
#endif
}

/*! Sets the calibration paramteres given a min-max range for a sensor as 
	well as the DOF it governs. Should soon be private.
*/
void GloveInterface::setParameters(int s, int d, float sMin, float sMax, float dMin, float dMax)
{
	fprintf(stderr,"sensor %d to DOF %d -- sMin %f sMax %f dMin %f dMax %f\n",s,d,sMin,sMax,dMin,dMax);

	double slope = ( dMax - dMin ) / ( sMax - sMin );
	double intercept = dMin - sMin * slope;
	fprintf(stderr,"  Slope %f and intercept %f \n",slope,intercept);
	mData->setSlope(d, s, slope);
	mData->setIntercept(d, intercept);
}


void GloveInterface::setParameters(int s, int d, float slope, float intercept)
{
	fprintf(stderr,"sensor %d to DOF %d -- slope %f and intercept %f \n", s, d, slope, intercept);
	mData->setSlope(d, s, slope);
	mData->setIntercept(d, intercept);
}

int GloveInterface::getNumSensors()
{
	return N_SENSOR_VALUES;
}

bool GloveInterface::isDOFControlled(int d)
{
	for (int s=0; s<N_SENSOR_VALUES; s++)
		if ( mData->getSlope(d,s) != 0 )
			return true;
	return false;
}

/*!	Asks the glove for the value of all sensors, then uses them to convert
	to a dof value.	In general, the value of the dof #d is obtained by:
	dof_d = intercepts_d + sum_over_all_sensors_s[slope(s,d) * raw_sensor(s)]
*/
float GloveInterface::getDOFValue(int d)
{
#ifdef HARDWARE_LIB
	float dofVal = 0;
	for ( int s=0; s<N_SENSOR_VALUES; s++) {
		int rawValue = rawGlove->getSensorValue ( s );
		dofVal += rawValue * mData->getSlope(d,s);
	}
	dofVal += mData->getIntercept(d);
	return dofVal;
#else
	assert(0);
	d = d;
	return 0;
#endif
}

/*! Sort of ugly code... This does the exact same as getDOFValue(int), 
	but assuming raw values are not actually read from the cyberglove 
	but passed as a parameter.
*/
float GloveInterface::getDOFValue(int d, int *rawValues)
{
	float dofVal = 0;
	for ( int s=0; s<N_SENSOR_VALUES; s++) {
		int rawValue = rawValues[s];
		dofVal += rawValue * mData->getSlope(d,s);
	}
	dofVal += mData->getIntercept(d);
	return dofVal;
}

//------------------------------------ CALIBRATION FUNCTIONS --------------------------------------------

void GloveInterface::initCalibration(int type)
{
	if ( type == FIST || type == SIMPLE_THUMB  || type == ABD_ADD) {
		//potential memory leaks!
		cPoses.clear();
		if (type == FIST ) {
			//sets up a simple calibration list with two hard-coded poses
			//one has all finger extended as if palm was lying on a table
			//the other one has all fingers curled into the palm
			//NOTE: THIS DOES NOT CONSIDER THE THUMB AT ALL
			//int map[] =  {-1,-1,-1,-1, -1, -1,   -1,   -1, -1, -1,-1,   -1,   -1, -1,-1,   -1,   -1,    -1,-1,-1,-1,-1,-1,-1};
			//			   0  1  2  3   4   5     6     7   8   9 10    11    12  13 14    15    16     17 18 19 20 21 22 23
			int map[] =  {-1,-1,-1,-1,  1,  2,    3,    5,  6,  7,-1,    9,   10, 11,-1,   13,   14,    15,-1,-1,-1,-1,-1,-1};
			double cp1[] = {0, 0, 0, 0,  0, -6, -4.5, -5.4,  0,  0, 0, -5.6, -3.7,  2, 0, -9.2, -2.0, -4.5, 0, 0, 0, 0, 0, 0};
			double cp2[] = {0, 0, 0, 0, 90, 90,   90,   90, 90, 90, 0,   90,   90, 90, 0,   90,   90,   90, 0, 0, 0, 0, 0, 0};
			cType = FIST;
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp1, map));
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp2, map));
		} else if (type == SIMPLE_THUMB) {
			//Very simple calibration that measures cross-talk between thumb CMC sensors from to "L" finger shapes
			//			     0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23
			int map[] =    {16, -1, -1, 17, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
			double cp1[] = { 70, 0,  0,-50,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
			double cp2[] = {-10, 0,  0,-50,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
			cType = SIMPLE_THUMB;
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp1, map));
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp2, map));
		} else if (type == ABD_ADD) {
			//attempts to calibrate abd-add angles from two hard-coded poses, with finger together and spread out
			//since we only have three sensors, it assumes middle finger is fixed and there is a 1-to-1 mapping
			//between sensors and fingers
			//			     0  1  2  3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23
			int map[] =    {-1,-1,-1,-1, -1, -1, -1, -1, -1, -1,  0, -1, -1, -1,  8, -1, -1, -1, 12, -1, -1, -1, -1, -1};
			double cp1[] = { 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
			double cp2[] = { 0, 0, 0, 0,  0,  0,  0,  0,  0,  0, 20,  0,  0,  0,-20,  0,  0,  0,-30,  0,  0,  0,  0,  0};
			cType = ABD_ADD;
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp1, map));
			cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES, cp2, map));
		}
	} else if (type == COMPLEX_THUMB) {
		cType = COMPLEX_THUMB;
	} else if (type == MEAN_POSE) {
		// this one just records a bunch of poses (both sensors and robot joint values), so some calibration
		// is assumed to already be in place. Then it compute the mean DOF values and writes them to a file
		cType = MEAN_POSE;
		cPoses.clear();
	}
	mCalibrated = false;
	currentPose = cPoses.begin();
}

void GloveInterface::nextPose(int direction)
{

	if ( cPoses.empty() ) {
		fprintf(stderr,"No calibration poses recorded!\n");
		return;
	}

	if (direction > 0) {
		currentPose++;
		if ( currentPose == cPoses.end() ) currentPose = cPoses.begin();
	} else {
		if ( currentPose == cPoses.begin() ) currentPose = cPoses.end();
		currentPose--;
	}
	getPoseError();
}

//asks the robot to replicate the current calibration pose
void GloveInterface::showCurrentPose()
{
	if ( cPoses.empty() ) {
		fprintf(stderr,"No poses recorded!\n");
		return;
	}

	double *desiredVals = new double[mRobot->getNumDOF()];
	int *map = (*currentPose)->getAllMaps();
	double *jointVals = (*currentPose)->getAllJointValues();
	int *sensorVals = (*currentPose)->getAllSensorValues();

	int s,d;

	//first, set all DOFs to current positions
	for (d=0; d<mRobot->getNumDOF(); d++) {
		desiredVals[d] = mRobot->getDOF(d)->getVal();
	}
	
	//then see which ones are actually set by the pose
	for (s=0; s<(*currentPose)->getSize(); s++) {
		d = map[s];
		if ( d >= 0) {
			if ( (*currentPose)->jointsSet )
				desiredVals[d] = jointVals[s];
			else if ( (*currentPose)->sensorsSet)
				desiredVals[d] = getDOFValue(d, sensorVals) ;
			else fprintf(stderr,"Can not show pose - neither joints nor sensors are set!\n");

		}
	}
	mRobot->forceDOFVals(desiredVals);
	transf t = mRobot->getFlockTran()->getAbsolute( (*currentPose)->getTransf() ); 
	//transf t = (*currentPose)->getTransf() ; 
	mRobot->setTran( t );
}

void GloveInterface::recordPoseFromGlove(int d)
{
#ifdef HARDWARE_LIB
	if (!instantRead()) {
		fprintf(stderr,"Can not read glove for calibration\n");
		return;
	}

	int sv[N_SENSOR_VALUES];
	double* jv = new double[mRobot->getNumDOF()];

	for (int i=0; i<N_SENSOR_VALUES; i++) {
		sv[i] = rawGlove->getSensorValue(i);
	}

	if (cType==FIST || cType==SIMPLE_THUMB || cType == ABD_ADD) {
		if ( cPoses.empty() ) {
			fprintf(stderr,"No poses recorded!\n");
			return;
		}
		(*currentPose)->setAllSensorValues(sv);
	} else if (cType == COMPLEX_THUMB) {
		cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES) );
		currentPose = cPoses.end();
		currentPose--;

		(*currentPose)->setAllSensorValues(sv);
		//			  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23
		int map[] = {16, 18, 19, 17,  1,  2,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
		(*currentPose)->setAllMaps(map);
		(*currentPose)->recordedDistance = d;
		fprintf(stderr,"Distance: %d\n",d);
	} else if (cType == MEAN_POSE) {
		cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES) );
		currentPose = cPoses.end();
		currentPose--;
		(*currentPose)->setAllSensorValues(sv);
		for (int d=0; d<mRobot->getNumDOF(); d++) {
			jv[d] = getDOFValue(d,sv);
			mRobot->checkSetDOFVals(jv);
		}
		(*currentPose)->setAllJointValues(jv);
		int map[] =  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,-1,-1,-1,-1};
		(*currentPose)->setAllMaps(map);

		//also save flock of birds location
		transf t = mRobot->getFlockTran()->getMount() * mRobot->getTran();
		//transf t = mRobot->getTran();
		(*currentPose)->setTransf(t);
	
		fprintf(stderr,"Pose for mean recorded!\n");
	} else {
		fprintf(stderr,"No calibration type initialized!\n");
	}
	delete [] jv;
#else
	d = d;
	assert(0);
#endif
}

bool GloveInterface::poseSet()
{
	if ( cPoses.empty() )
		return false;

	return (*currentPose)->isSet();
}

//if at least two calibration poses have been set we are ready to calibrate
bool GloveInterface::readyToCalibrate()
{
	if (cType == FIST || cType == SIMPLE_THUMB || cType == ABD_ADD) {
		int numSet = 0;
		std::list<CalibrationPose*>::iterator it;
		for (it=cPoses.begin(); it!=cPoses.end(); it++) {
			if ( (*it)->isSet() ) numSet++;
		}
		if (numSet>=2) return true;
	} else if (cType == COMPLEX_THUMB) {
		if ( (int)cPoses.size() > 5 )return true;
	} else if (cType == MEAN_POSE) {
		if (!cPoses.empty()) return true;
	}
	return false;
}

bool GloveInterface::performCalibration()
{
	switch (cType) {
		case FIST:
			return performSimpleCalibration();
			break;
		case ABD_ADD:
			return performSimpleCalibration();
			break;
		case SIMPLE_THUMB:
			return performThumbCalibration();
			break;
		case COMPLEX_THUMB:
			performComplexCalibration();
			mCalibrated = true;
			//saveCalibrationPoses();
			return true;
			break;
		case MEAN_POSE:
			computeMeanPose();
			break;
		default:
			fprintf(stderr,"Unknown calibration type requested\n");
			break;
	}
	return false;
}

//performs a simple calibration looking only at first two calibration poses in list
//computed slopes and intercepts from those poses. No fitting or anything fancy
bool GloveInterface::performSimpleCalibration()
{
	if (!readyToCalibrate() ){
		return false;
	}

	CalibrationPose *cp1, *cp2;
	std::list<CalibrationPose*>::iterator it;
	it = cPoses.begin();
	cp1 = (*it);
	it++;
	cp2 = (*it);

	float sMin, sMax, dMin, dMax;
	for (int i=0; i<N_SENSOR_VALUES;i++){
		//check if both poses constrain this sensor value
		if ( cp1->getMap(i) < 0 || cp2->getMap(i) < 0 ){
			fprintf(stderr,"Sensor %d masked\n",i);
			continue;
		}
		int d = cp1->getMap(i);
		if ( d != cp2->getMap(i) ) {
			fprintf(stderr,"Error! Sensor %d has different maps in poses!\n",i);
			continue;
		}
		sMin = cp1->getSensorValue(i); sMax = cp2->getSensorValue(i);
		dMin = cp1->getJointValue(i);  dMax = cp2->getJointValue(i);
		setParameters(i, d, sMin, sMax, dMin, dMax);
	}

	mCalibrated = true;
	return true;
}

bool GloveInterface::performThumbCalibration()
{
	if (!readyToCalibrate() ){
		return false;
	}

	CalibrationPose *cp1, *cp2;
	std::list<CalibrationPose*>::iterator it;
	it = cPoses.begin();
	cp1 = (*it);
	it++;
	cp2 = (*it);

	int R11 = cp1->getSensorValue(3);
	int R12 = cp2->getSensorValue(3);

	double S1 = mData->getSlope(17, 3);
	double alpha = S1 * ( R11 - R12 );

	int R21 = cp1->getSensorValue(0);
	int R22 = cp2->getSensorValue(0);

	double S2 = alpha / ( R22 - R21 );
	double I2 = - S2 * R21;

	mData->setSlope(17, 0, S2);
	mData->setIntercept(17, mData->getIntercept(17)+I2);
	fprintf(stderr,"Result: slope %f and intercept %f \n",S2,I2);

	mCalibrated = true;
	return true;
}

bool GloveInterface::performComplexCalibration()
{
	for (int i=0; i<100; i++) {
		complexCalibrationStep();
	}
	return true;
}

bool GloveInterface::complexCalibrationStep()
{
	//assemble equation
	int i;

	int nPoses = 0;
	for (currentPose = cPoses.begin(); currentPose!=cPoses.end(); currentPose++) {
		showCurrentPose();
		//when negative error has greater magnitude than our threshold, we don't have a reliable
		//estimate of the error VECTOR, so we don't use the pose at all
		if ( getPoseError() > -7.8)
			nPoses ++;
	}
	double *JD = new double[nPoses * 30];
	int ldd = nPoses * 3;

	double *P = new double[nPoses * 3];
	double *fg = new double[10];

	i=0;
	for (currentPose = cPoses.begin(); currentPose!=cPoses.end(); currentPose++) {
		showCurrentPose();
		if (getPoseError() <= -7.8)
			continue;
		assembleJMatrix( JD + i*3, ldd);
		assemblePMatrix( P + i*3 );
		i++;
	}
/*
	int j,k;
	for (k=0; k<nPoses; k++) {
		for (i=0; i<3; i++) {
			for (j=0; j<10; j++) {
				fprintf(stderr,"%.2f ",JD[k*3 + i+j*ldd]);
				if (j == 3) fprintf(stderr,"  ");
			}
			fprintf(stderr,"\n");
		}
		fprintf(stderr,"\n\n");
	}

	for (i=0; i<nPoses; i++) {
		fprintf(stderr,"%.2f %.2f %.2f \n",P[3*i], P[3*i+1], P[3*i+2]);
	}
*/
//	fprintf(stderr,"Number of poses: %d\n",nPoses);

	//solve using pseudo-inverse
	int info,rank;
	int lwork = 100 * 3 * nPoses;
	double *work = new double[lwork];
	double *SVDs = new double[MIN(3*nPoses,10)];

	dgelss(3*nPoses, 10, 1, 
		   JD, 3*nPoses, P, 3*nPoses, SVDs, 1.0e-5, &rank, work, lwork, &info);

//	fprintf(stderr,"Solved; info is %d and effective rank %d\n",info,rank);
//	fprintf(stderr,"Total error before step: %f\n",getTotalError());
//	for (i=0; i<10; i++) {
//		fprintf(stderr,"%f ",P[i]);
//	}

	//update parameters
	mData->addToIntercept(16,P[0]);
	mData->addToIntercept(17,P[1]);
	mData->addToIntercept(18,P[2]);
	mData->addToIntercept(19,P[3]);

	mData->addToSlope(16,0,P[4]);
	mData->addToSlope(16,3,P[5]);
	mData->addToSlope(17,0,P[6]);
	mData->addToSlope(17,3,P[7]);
	mData->addToSlope(18,1,P[8]);
	mData->addToSlope(19,2,P[9]);

	fprintf(stderr,"Total error AFTER step: %f\n",getTotalError());
	currentPose = cPoses.begin();

	delete [] SVDs;
	delete [] work;
	delete [] JD;
	delete [] P;
	delete [] fg;

	return true;
}

bool
GloveInterface::computeMeanPose()
{
	//computed the mean of the saved poses and writes it to a hard-coded file name.
	double *jv = new double[mRobot->getNumDOF()];

	for (int d=0; d<mRobot->getNumDOF(); d++) {
		jv[d]=0;
	}

	for (currentPose = cPoses.begin(); currentPose != cPoses.end(); currentPose++) {
		for (int d=0; d<mRobot->getNumDOF(); d++) {
			jv[d] += (*currentPose)->getJointValue(d);
		}
	}

	for (int d=0; d<mRobot->getNumDOF(); d++) {
		jv[d] = jv[d] / cPoses.size();
	}	
	
	cPoses.push_back( new CalibrationPose(N_SENSOR_VALUES) );
	currentPose = cPoses.end();
	currentPose--;
	(*currentPose)->setAllJointValues(jv);
	int map[] =  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,-1,-1,-1,-1};
	(*currentPose)->setAllMaps(map);
	showCurrentPose();

	FILE *fp = fopen("mean_pose.txt","w");
	fprintf(stderr,"Mean pose written to mean_pose.txt \n");
	(*currentPose)->writeToFile(fp);
	fclose(fp);

	delete [] jv;
	return true;
}

void GloveInterface::saveCalibration(const char* filename)
{
	FILE *fp = fopen(filename,"w");
	if (!fp) {
		fprintf(stderr,"Unable to open calibration file!\n");
		return;
	}

	int d,s;
	fprintf(fp,"%d %d\n",mRobot->getNumDOF(), N_SENSOR_VALUES);
	for (d=0; d<mRobot->getNumDOF(); d++) {
		for (s=0; s<N_SENSOR_VALUES; s++)
			fprintf(fp, "%f ",mData->getSlope(d,s));
		fprintf(fp,"\n");
	}
	for (d=0; d<mRobot->getNumDOF(); d++)
		fprintf(fp,"%f ",mData->getIntercept(d));
	fprintf(fp,"\n");
	fclose(fp);
	fprintf(stderr,"Calibration saved\n");
}

bool GloveInterface::loadCalibration(const char* filename)
{
	FILE *fp = fopen(filename,"r");
	if (!fp) {
		fprintf(stderr,"Unable to open calibration file!\n");
		return false;
	}

	int d,s;
	int numDOF, numSens;
	float val;

	if (fscanf(fp,"%d %d ",&numDOF,&numSens)  <= 0) { 
	  DBGA("GloveInterface::loadCalibration - Failed to dof num or sensor num");	 
	  return false;
	}
	
	if ( numDOF != mRobot->getNumDOF() || numSens != N_SENSOR_VALUES ) {
	  fprintf(stderr,"WARNING: calibration file contains wrong number of DOFs/sensors\n");
	  return false;
	}
	
	mData->reset();
	for (d=0; d<numDOF; d++) {
	  for (s=0; s<numSens; s++) {
	    if(fscanf(fp,"%f",&val) <= 0) {
	      DBGA("GloveInterface::loadCalibration - Failed to dof or sensor value");	 
	      return false; 
	    }
	  }
	  mData->setSlope(d,s,val);
	}
	
	for (d=0; d<mRobot->getNumDOF(); d++) {
	  if(fscanf(fp,"%f",&val) ){
	     DBGA("GloveInterface::loadCalibration - Failed to dof num or sensor num");	 
	     return false;
	  }
	  mData->setIntercept(d,val);
	}
	
	fclose(fp);
	fprintf(stderr,"Calibration loaded from file\n");
	return true;
}

/* saves the calibration poses to a file. For now, just sensor values and maps */
void GloveInterface::saveCalibrationPoses(const char *filename)
{
	writePoseListToFile(&cPoses, filename);
	currentPose = cPoses.begin();
}

void GloveInterface::loadCalibrationPoses(const char *filename)
{
	loadPoseListFromFile(&cPoses, filename);
	currentPose = cPoses.begin();
}

double GloveInterface::getPoseError(vec3* error, position* thumbLocation)
{
	double shellDistance = 8;
	double cFactor = (*currentPose)->recordedDistance;

	vec3 expectedVector, measuredVector, e;

	showCurrentPose();
	Body *thumbTip = mRobot->getChain(4)->getLink(2);
	Body *indexTip = mRobot->getChain(0)->getLink(2);
	position p1, p2;
	double rawDistance;
	rawDistance = mRobot->getWorld()->getDist(thumbTip, indexTip, p1, p2);

	if (thumbLocation != NULL) {
		*thumbLocation = p1;
	}

	p1 = p1 * thumbTip->getTran() * mRobot->getChain(4)->getTran().inverse();
	p2 = p2 * indexTip->getTran() * mRobot->getChain(4)->getTran().inverse();
	measuredVector = p2 - p1;

	expectedVector = normalise(measuredVector);
	expectedVector = ( cFactor + shellDistance) * expectedVector;

	//this isn't really the error, it's the motion that will COMPENSATE for the error
	//so it's actually the negated error (which would be expected - measured)
	e = measuredVector - expectedVector;

	if (error != NULL) {
		*error = e;
	}

//	fprintf(stderr,"Compensated: %f\n",rawDistance - shellDistance);
//	fprintf(stderr,"Expected: %f; abs error: %f\n",cFactor, e.len());
//	fprintf(stderr,"X: %f  Y: %f  Z: %f\n",e.x(), e.y(), e.z());

	return e.len();
}

double GloveInterface::getTotalError()
{
	double totalError = 0;
	for ( currentPose=cPoses.begin(); currentPose!=cPoses.end(); currentPose++) {
		totalError += getPoseError();
	}
	totalError /= cPoses.size();
	return totalError;
}

void GloveInterface::getPoseJacobian(double *J)
{
	double t1 = mRobot->getDOF(16)->getVal();
	double t2 = mRobot->getDOF(17)->getVal() + 0.8639;
	double t3 = mRobot->getDOF(18)->getVal() + 0.0873;
	double t4 = mRobot->getDOF(19)->getVal() + 1.4835;

	jacobian(t1, t2, t3, t4, 0, 0, 1000, J);
}

void GloveInterface::assembleJMatrix(double *D, int ldd)
{
	double t1 = mRobot->getDOF(16)->getVal();
	double t2 = mRobot->getDOF(17)->getVal() + 0.8639;
	double t3 = mRobot->getDOF(18)->getVal() + 0.0873;
	double t4 = mRobot->getDOF(19)->getVal() + 1.4835;

	position errorPos;
	getPoseError(NULL,&errorPos);

	double J[12];
//	fprintf(stderr,"Pose err: %.2f %.2f %.2f\n",errorPos.x(), errorPos.y(), errorPos.z());
	jacobian(t1, t2, t3, t4, errorPos.x(), errorPos.y(), errorPos.z(), J);

	double s0 = (*currentPose)->getSensorValue(0);
	double s1 = (*currentPose)->getSensorValue(3);
	double s2 = (*currentPose)->getSensorValue(1);
	double s3 = (*currentPose)->getSensorValue(2);

	double dTdG[4*6];
	compute_dTdG(s0, s1, s2, s3, dTdG);

	double JD[3*6];
	dgemm("N", "N", 3, 6, 4, 
		  1, J, 3, 
		  dTdG, 4, 
		  0, JD, 3);

	int i,j;
	for (i=0; i<3; i++) {
		for (j=0; j<4; j++) {
			D[ i + j*ldd] = J[ i + 3*j];
		}
	}
	for (i=0; i<3; i++) {
		for (j=0; j<6; j++) {
			D[ i + (4+j)*ldd ] = JD[ i + 3*j ];
		}
	}
}

void GloveInterface::assemblePMatrix( double *P )
{
	vec3 error;
	getPoseError(&error);
	
	P[0] = error.x();
	P[1] = error.y();
	P[2] = error.z();
}

void GloveInterface::clearPoses()
{
	cPoses.clear();
	currentPose = cPoses.begin();
}

void GloveInterface::startGlove()
{
	DBGA("Disabled... Init glove from World menu");
}
