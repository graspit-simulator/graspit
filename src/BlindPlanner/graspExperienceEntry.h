/*
 * graspExperienceEntry.h
 *
 *  Created on: Jun 11, 2011
 *      Author: dang
 */

#ifndef GRASPEXPERIENCEENTRY_H_
#define GRASPEXPERIENCEENTRY_H_

#include <vector>

#include "matvec3D.h"
#include "blindPlannerUtil.h"

//the threshold for contacts to be considered as a potential match
#define NN_DIST 10

class GraspExperienceEntry
{
private:
	//char mGraspId[100];
	GraspInfo mGraspInfo;
	std::vector<ContactInfo> mContactList;
	//the spread angle
	double mSpread;

	//auxiliary data storage
	//just a temporary data storage for temporary usage, updated for each
	double mDistance;
	//the relative pose used when needed
	transf mPose;

	//helper functions
	double L2Distance(const ContactInfo& cl1, const ContactInfo& cl2) const;
public:
	GraspExperienceEntry(){	}
	GraspExperienceEntry(const GraspInfo& gi, const std::vector<ContactInfo>& ci){ mGraspInfo = gi; mContactList = ci; }
	/*initialize an entry with one line of string from the file possibly
	 * the format of str is: graspID NUMContacts Contact1 Contact2 ...
	 * Contact* is x y z qw qx qy qz
	 */
	GraspExperienceEntry(const char* str, int version = 0);
	//get the similarity value based on tactile contacts
	double getTactileSimilarityMeasure(const GraspExperienceEntry& g, bool considerSpread = true) const;
	std::vector<ContactInfo> getContactList() const { return mContactList; }
	void setContactList(const std::vector<ContactInfo>& cl){ mContactList = cl; }
	//info string consists of the name of the object and the id number of the grasp
	std::string getInfoString(){ return mGraspInfo.getString(); }
	GraspInfo getGraspInfo(){ return mGraspInfo; }
	//mDistance is a temporary data storage only used for ranking when needed, it is modified when used
	void setDistance(double d){ mDistance = d; }
	double getDistance(){ return mDistance; }
	void setPose(transf p){mPose = p;}
	transf getPose() const {return mPose;}
	double getEpsilonQuality() const { return mGraspInfo.epsilonQuality; }
	double getVolumeQuality() const { return mGraspInfo.volumeQuality; }
	void setSpread(double s){ mSpread = s; }
	double getSpread() const { return mSpread; }


	
	//compute the distance of this grasp from another grasp g
	//this considers all the existent contacts without a threshold
	double minimumSumOfL2Distance(const GraspExperienceEntry& g) const;

	//this only considers contact pairs within some threshold defined by NN_DIST
	double estimateSumOfL2Distance(const GraspExperienceEntry& g) const;

	//helper functions
	void printMe(bool verbose = true) const;

	static GraspExperienceEntry getCurrentGraspAsGraspExperienceEntry();

};

#endif /* GRASPEXPERIENCEENTRY_H_ */
