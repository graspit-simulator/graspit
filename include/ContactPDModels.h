#ifndef CPDMODEL_H
#define CPDMODEL_H
#include <vector>
/*
Defines a pressure model for distributing force in non-point contact models.
*/
class position;
class ContactPressureDistributionModels{
	enum modelType{WINKLER,HERTZIAN};
	modelType mtype;
public:
	ContactPressureDistributionModels();
	double maxFrictionOverTotalLoad(double * params);
	double pressureDistribution(double * params, double x, double y);
	void distributionSamples(double a, double b, int majRows, int minRows, std::vector<position> &pVec );
	void sampleForces(double * params, double a, double b, int majRows, int minRows, const std::vector<position> &pVec, std::vector<double> &forceVec );	
};
#endif