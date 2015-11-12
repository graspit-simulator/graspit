#include "GloveTrans.h"
#include <stdio.h>

GloveTrans::GloveTrans()
{
	slopes = new float[N_SENSOR_VALUES];
	intercepts = new float[N_SENSOR_VALUES];
	jointValues = new float[N_SENSOR_VALUES];
	for (int i=0; i<N_SENSOR_VALUES; i++) {
		slopes[i] = 1;
		intercepts[i] = 0;
	}
	rawGlove = NULL;
}

GloveTrans::~GloveTrans()
{
	if (rawGlove)
		delete rawGlove;
	delete [] slopes;
	delete [] intercepts;
	delete [] jointValues;
}

int GloveTrans::instantRead()
{
	if (!rawGlove)
		return 0;
	if (!rawGlove->instantRead() )
		return 0;
	for (int i=0; i<N_SENSOR_VALUES; i++) {
		jointValues[i] = rawGlove->getSensorValue(i) * slopes[i] + intercepts[i];
	}
	return 1;
}

float GloveTrans::getJointValue(int i)
{
	if (!rawGlove){
		fprintf(stderr,"No glove present!\n");
		return 0;
	}
	if (i<0 || i>=N_SENSOR_VALUES) {
		fprintf(stderr,"Wrong sensor ID!\n");
		return 0;
	}

	/*
	if (i==3) {
		float h = ((rawGlove->getSensorValue(0) - 100.0) * 30.0) / (80.0 * 100.0);
		fprintf(stderr,"h  is %f\n",h);
		float r = rawGlove->getSensorValue(i) * (1+h) * slopes[i] + intercepts[i];
		return r;
	}
	*/
	//fprintf(stderr,"Raw value: %d\n",rawGlove->getSensorValue(i));
	return rawGlove->getSensorValue(i) * slopes[i] + intercepts[i];
}

void GloveTrans::setParameters(int s, float sMin, float sMax, float dMin, float dMax)
{
	fprintf(stderr,"sMin %f sMax %f dMin %f dMax %f\n",sMin,sMax,dMin,dMax);
	float r = ( dMax - dMin ) / ( sMax - sMin );
	slopes[s] = r;
	intercepts[s] = dMin - sMin * r;
	fprintf(stderr,"Slope %f and intercept %f \n",slopes[s],intercepts[s]);
}