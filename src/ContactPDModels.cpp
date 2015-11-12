#include "ContactPDModels.h"
#include <math.h>
#include "debug.h"
#include "matvec3D.h"
//FIXME:Not all math.h contain a defition for pi, and the use
//of it elsewhere in graspit may not be portable!  This problem should
//be examined more thoroughly and resolved in a consistent manner
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ContactPressureDistributionModels::ContactPressureDistributionModels(){
	mtype = HERTZIAN;
}

double ContactPressureDistributionModels::maxFrictionOverTotalLoad(double * params){
	switch(mtype) {
		case WINKLER:{
			double a = params[0],b = params[1];
			return 8/15*sqrt(a*b);
			}
		case HERTZIAN:{
			double a = params[0],b = params[1];
			return 3*M_PI/16*sqrt(a*b);
			}
	}
	DBGA("ContactPressureDistributionModels::Unknown contact pressure distribution type");
	return -1;
}

double ContactPressureDistributionModels::pressureDistribution(double * params, double x, double y){
switch(mtype){
		case WINKLER:{
			double a = params[0],b = params[1], K = params[2], h = params[3], delta = params[4];
			return K*delta/h*(1-pow(x,2)/pow(a,2) - pow(y,2)/pow(b,2));
			}
		case HERTZIAN:{
			double a = params[0],b = params[1],nForce = params[2];
			//without considering the direction of the major and minor axis
			//return 3/(2 * M_PI * a*b) * pow(1-(x*x+y*y)/(a*b),0.5);

			/* considering the major and minor axis of the contact elipse
			   x,y are coordinates in the common coordinate system, see
			   Contact Mechanics Page 85.
			   The distribution can be found in Page 95: p(x,y) = p_0 * (1 - (x/a)^2 - (y/b)^2)^0.5
			   The total load for an ellipse contact P = (2/3) p_0 * pi * a * b
			   The following formula assumes the total load on the whole ellipse area is ONE newton
			   since P = 1, we have p_0 = 3/2*a*b*pi
			*/
			//std::cout << "a: " << a << " b: " << b << " nForce: " << nForce << std::endl;

			return nForce * 3/(2 * M_PI * a * b) * pow(1-pow(x,2)/pow(a,2)-pow(y,2)/pow(b,2), 0.5);
			}
	}
DBGA("ContactPressureDistributionModels::Unknown contact pressure distribution type");
return -1;
}

/* FIXME: Lame implementation - there are faster ways
 * pVec stores the sampling positions in the common contact ellipse
 */
void ContactPressureDistributionModels::distributionSamples(double a, double b, int majRows, int minRows, std::vector<position> &pVec )
{
	//double sum = 0;
	//double params[2]={a,b};
	for (int inda = 0; inda <= majRows; inda ++){
		for (int indb = 0; indb <= minRows; indb ++){
			double majCoord = -a+ 2*a*(inda+.5)/majRows;
			double minCoord = -b+ 2*b*(indb+.5)/minRows;

			if((pow(majCoord/a,2) + pow(minCoord/b,2)) <= 1)
			{
				pVec.push_back(position(majCoord, minCoord, 0));
				//sum +=pressureDistribution(params, majCoord, minCoord) * 2*a * 2*b / (majRows * minRows);
			}

		}
	}
	//std::cout << "sum is : " << sum << std::endl;
}

/*FIXME: logical constness is wanted for pVec
*/
void ContactPressureDistributionModels::sampleForces(double * params, double a, double b, int majRows, int minRows, const std::vector<position> &pVec, std::vector<double> &forceVec ){
	//note the length of the ellipse along the major axis is 2a, not a
	//so when we are computing the area, we need to use 2a and 2b, not a or b
	double sampleArea = 2*a/majRows * 2*b/minRows;
	for(unsigned int pind = 0; pind < pVec.size(); pind++){
		forceVec.push_back(sampleArea * pressureDistribution(params, pVec[pind].x(),pVec[pind].y()));
	}
}
