#include "discretize.h"
#include "contact.h"

void discretize::discretizeEllipse(SoftContact * sc, std::vector<vec3> & position, std::vector<double> & force)
{
	double a = params[0], b = params[1];
	vec3 center(params[2], params[3], params[4]), normal(params[5], params[6], params[7]);
	for(int i = 0; i < intervalA; ++i)
		for(int j = 0; j < intervalB; ++j)
		{

		}
}