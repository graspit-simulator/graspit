#include "SemanticGrasp.h"

void SemanticGrasp::setRollingAdjustment(double angleInDegree)
{
	mRollingAdjustment = 
		transf(Quaternion(angleInDegree * M_PI/180.0,vec3(0,0,1)),vec3(0,0,0));
}

//GraspExperienceEntry getAsGraspExperienceEntry()
//{
//	GraspExperienceEntry gee;
//	gee.setContactList(mContactList);
//	gee.setSpread(getCurrentHandSpread());
//	return gee;
//}