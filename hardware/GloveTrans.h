#ifndef _glovetrans_h_
#define _glovetrans_h_

#include "CyberGlove.h"

class GloveTrans {
private:
	CyberGlove *rawGlove;
	float *slopes, *intercepts;
	float *jointValues;
public:
	GloveTrans();
	~GloveTrans();
	void setGlove(CyberGlove *glove){rawGlove = glove;}

	int instantRead();
	float getJointValue(int i);
	int testGlove(){return rawGlove->testGlove();}
	int getNumSensors(){return N_SENSOR_VALUES;}
	void setParameters(int s, float sMin, float sMax, float dMin, float dMax);
};

#endif