/*
 * graspExperienceBase.h
 *
 *  Created on: Jun 11, 2011
 *      Author: dang
 */

#ifndef GRASPEXPERIENCEBASE_H_
#define GRASPEXPERIENCEBASE_H_

#include <string>
#include <vector>

#include "graspExperienceEntry.h"

class GraspExperienceBase
{
private:
	//is the database loaded
	bool mIsLoaded;

	std::vector<GraspExperienceEntry> mGraspEntries;
	std::vector<transf> mHandPoses;
	std::vector<double> mSpread;

public:
	GraspExperienceBase() : mIsLoaded(false){}
	bool loadGraspExperienceBase(std::string path, int version = -1, const char * filter = NULL);
	bool loadHandPoses(std::string path, const char * filter = NULL);
	int getNumGrasps(){return mGraspEntries.size();}
	std::vector<GraspExperienceEntry> getKNNBasedOnTactile(const GraspExperienceEntry& e, int k = 20);
	//look around the location where graspexpeirence defines and search for the grasp with the epsilon quality
	GraspExperienceEntry getNNBasedOnPose(const GraspExperienceEntry& e, double radius, double epsilon);
	transf getPose(int i){ return mGraspEntries[i].getPose(); } //return mHandPoses[i]; }
	transf getPose(std::string id);
	//with multiple gee's, we interpolate a pose based on their distance,
	//cutoff specifies the threshold of distance for stopping considering the interpolation of the gee
	//default 1.2 means that we will look for all those NN's that are within a distance that is 1.2 times
	//the distance of the first NN
	transf getWeightedPose(std::vector<GraspExperienceEntry> geeList, double cutoff = 1.2);
	GraspExperienceEntry getEntry(int i) {return mGraspEntries[i];}
	GraspExperienceEntry getEntry(std::string id);
	void setGraspEntryList(std::vector<GraspExperienceEntry> list){ mGraspEntries = list; }
};


#endif /* GRASPEXPERIENCEBASE_H_ */
