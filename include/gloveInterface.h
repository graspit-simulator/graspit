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
// $Id: gloveInterface.h,v 1.10 2009/04/21 14:53:07 cmatei Exp $
//
//######################################################################

#ifndef _GLOVEINTERFACE_H_
#define _GLOVEINTERFACE_H_

class vec3;
class position;
class CyberGlove;
class Robot;

#include <QString>
#include <list>
#include <vector>

#include "matvec3D.h"

//! A data structure for conveniently storing hand data used for calibration
/*!	A calibration pose matches a set of joint values to a set of raw sensor 
	readings (individual CyberGlove joint readings) for a given pose. It is 
	ready to be used if both joint values and raw sensor values have been set.
*/
class CalibrationPose {
private:
	//! The number of raw sensor values from the glove. Usually hard-coded to 24
	int mSize;
	//! A set of joint values that the given sensor values should correspond to
	double *jointValues;
	//! A set of raw sensor values that the given joint values should correspond to
	int *sensorValues;
	//! Tells us to which robot DOF number each CyberGlove sensor is related to. 
	/*! A map of -1 means this sensor is not calibrated by this pose (like for 
		example the thumb, which this calibration poses ignore)*/
	int *sensorMap;
	//! A transform that is associated with this pose, for doing calibrations wrt objects or using flocks
	transf mTransf;
	//! Initializes an empty pose with no recorded information
	void init(int size);
public:
	CalibrationPose(int size);
	CalibrationPose(int size, double *joints, int *map);
	~CalibrationPose();

	void setJointValue(int j, double jv);
	void setAllJointValues(double *jv);
	void setSensorValue(int i, int sv);
	void setAllSensorValues(int *sv);
	void setMap(int i, int mv);
	void setAllMaps(int *m);
	void setTransf(transf t){mTransf = t;}

	double getJointValue(int i){return jointValues[i];}
	double *getAllJointValues(){return jointValues;}
	int getSensorValue(int i){return sensorValues[i];}
	int *getAllSensorValues(){return sensorValues;}
	int getMap(int i){return sensorMap[i];}
	int *getAllMaps(){return sensorMap;}
	transf getTransf(){return mTransf;}

	//! Tells us if this pose is ready to be used (i.e. all the required data has been set)
	bool isSet();
	int getSize(){return mSize;}
	bool jointsSet, sensorsSet, mapSet, poseSet;

	//! Used for thumb calibration to record the expected distance between the index and the thumb
	double recordedDistance;

	//! Some misc. info you might need later. Actually used to store object file name for grasp poses
	QString miscInfo;

	void writeToFile(FILE *fp);
	void readFromFile(FILE *fp);
};

//! Holds the information for linear conversion of raw glove senor readings to dof values
/*! The CData (stands for conversion data) class holds the information
	for converting raw glove readings to dof values. The conversions are
	all linear, so for each sensor, we store a slope and an intercept.
	It does not know how to match sensors with individual dof's. It also
	contains a couple of convenience functions to help calibration.

	In general, any glove sensor can affect any robot dof, there is not a 
	1-to-1 mapping. The slopes store the effect that each glove sensor
	has on each robot dof. It is therefore a sparse nd x ns matrix, where
	nd is the number of dofs and ns is the number of sensors. The intercepts
	are specific to each dof.

	In general, the value of the dof #d is obtained by:
	dof_d = intercepts_d + sum_over_all_sensors_s[slope(s,d) * raw_sensor(s)]
*/
class CData {
private:
	double *slopes,*intercepts;
	int nDOF, nSensors;
public:
	CData(int nd, int ns);
	~CData();
	double getSlope(int d, int s);
	double getIntercept(int d);
	void setSlope(int d, int s, double val);
	void setIntercept(int d, double val);
	void addToSlope(int d, int s, double val);
	void addToIntercept(int d, double val);
	void reset();
};

//! Interface between a GraspIt robot and the CyberGlove
/*! This class interfaces a Robot with a CyberGlove. The Raw Cyber Glove 
	(in the Sensors library) just talks to the serial port and gets raw 
	glove sensor values between 1 and 255. This interface converts them to
	joint angle values, can perform calibration to compute the parameters 
	for this conversion, and matches glove sensor numbers to robot DOF 
	numbers.

	This class can also calibrate the CyberGlove (compute the conversion
	parameters from raw sensor data to dof values). Unfortunately, the 
	calibrations are rather poorly engineered from a software standpoint,
	they should really have a dedicated interface and inheritance
	hierarchies.

	Up to some degree, all calibrations behave the same way: the user must
	supply a set of postures that are expected to match a pre-defined
	set of joint angles. Alternatively, the calibration postures can
	also be read from a file. After the record step, the calibration step
	uses these postures to compute the parameters. This step depends on
	the calibration type. See the enum \a calibrationType for details.

	The most difficult to calibrate is the thumb, where a couple of 
	glove sensors are affected by more then one dof. See the paper by
	Griffin, Findley, Turner and Cutkosky, "Calibration and Mapping of 
	a Human Hand for Dexterous Telemanipulation" for some details that
	have inspired the calibration done here.
*/
class GloveInterface {
private:
	//! The instance of the raw glove where raw sensor values come from
	CyberGlove *rawGlove;
	//! The robot that uses this interface.
	Robot *mRobot;
	//! The data for performing linear conversion from raw sensor data to dof values
	CData *mData;

	//! A list of poses currently used for calibration
	std::list<CalibrationPose*> cPoses;
	//! The current pose selected (for recording data or for display)
	std::list<CalibrationPose*>::iterator currentPose;
	//! The type of calibration currently being performed
	int cType;
	//! Whether this interface is calibrated (and ready to use) or not
	bool mCalibrated;

	double *savedDOFVals;

	bool computeMeanPose();
	bool performSimpleCalibration();
	bool performThumbCalibration();
	bool performComplexCalibration();
	bool complexCalibrationStep();

	//! Main interface function, gets a dof value from a list of raw sensor readings
	float getDOFValue(int d, int *rawValues);
public:
	GloveInterface(Robot *robot);
	~GloveInterface();
	Robot* getRobot(){return mRobot;}
	void setGlove(CyberGlove *glove);

	//! Initializes the glove, in case we want to use it during the calibration
	/*! We can also use pre-recorded poses if we don't want to. */
	void startGlove();
	//! Asks the raw CyberGlove to refresh its readings from the sensors via the serial port
	int instantRead();

	//! Tells wether a particular robot DOF is controlled via the CyberGlove or not
	bool isDOFControlled(int d);
	//! Uses the latest sensor values from the CyberGlove and computes the values of a particular DOF
	float getDOFValue(int d);

	//! Returns the number of raw sensors in the CyberGlove
	int getNumSensors();
	//! Computes parametes for linear conversion for a particular combination of raw sensor and dof number
	void setParameters(int s, int d, float sMin, float sMax, float dMin, float dMax);
	//! Sets the parametes for linear conversion for a particular combination of raw sensor and dof number
	void setParameters(int s, int d, float slope, float intercept);

	/* All the other function here are used for calibration of various types*/		

	//! The types of calibrations available with this interface
	/*! Here is what each does:

		FIST: asks the user for only two poses: with the hand flat and with the fist
		closed. Then, uses the two resulting values for each finger flexion dof
		to compute the slope and intercept. Does NOT calibrate the thumb at all,
		or the abduction / adduction dofs.

		SIMPLE_THUMB:

		COMPLEX_THUMB: asks the user to record as many poses as pssible where the
		distance between the index finger and the thumb is known. This is done
		either by touching the index and the thumb, or by holding between them
		an object of know size. Then attempts to use this data to calibrate
		the thumb sensors.

		ABD_ADD: asks for two poses, at the two ends of abd / add to compute
		the linear parameters for those dofs.

		MEAN_POSE: not really a calibration; the user records as many poses
		as she want, then this computes the mean of those poses.
	*/
	enum calibrationTypes{FIST, SIMPLE_THUMB, COMPLEX_THUMB, ABD_ADD, MEAN_POSE};

	void nextPose(int direction);
	void showCurrentPose();
	void recordPoseFromGlove(int d=0);

	bool poseSet();
	bool readyToCalibrate();
	bool calibrated(){return mCalibrated;}

	void saveCalibrationPoses(const char* filename);
	void loadCalibrationPoses(const char* filename);
	void clearPoses();
	int getNumPoses(){return cPoses.size();}

	bool loadCalibration(const char* filename);
	void saveCalibration(const char* filename);

	void initCalibration(int type);
	bool performCalibration();

	double getPoseError(vec3* error=NULL, position* thumbLocation=NULL);
	double getTotalError();
	void getPoseJacobian(double *J);
	void assembleJMatrix(double *D, int lda);
	void assemblePMatrix( double *P );

	void saveRobotPose();
	void revertRobotPose();
};
#endif
