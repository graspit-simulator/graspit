/*
 * graspExperienceEntry.cpp
 *
 *  Created on: Jun 11, 2011
 *      Author: dang
 */

#include "graspExperienceEntry.h"

GraspExperienceEntry::GraspExperienceEntry(const char* str, int version)
{
	int num;
	int offset;
	std::string tmp;
	char tmpStr[30];
	sscanf(str, "%s %lf %lf %d%n", tmpStr, &mGraspInfo.epsilonQuality, &mGraspInfo.volumeQuality, &num, &offset);
	tmp = std::string(tmpStr);
	size_t loc = tmp.find_last_of('_');
	mGraspInfo.objectName = tmp.substr(0, loc);
	//std::cout << tmp.c_str() + loc + 1 << std::endl;
	sscanf(tmp.c_str() + loc + 1, "%d", &mGraspInfo.graspIndex);

	//load the contacts
	for(int i = 0; i < num; ++i)
	{
		double x, y, z, qx, qy, qz, qw, force = -1, spread = -1;
		str += offset;
		if(version == 0)
		  {
		    //std::cout << "version 0" << std::endl;
			sscanf(str, "%lf %lf %lf %lf %lf %lf %lf%n", &x, &y, &z, &qw, &qx, &qy, &qz, &offset);
		  }
		else if(version == 1)
		  {
		    //std::cout << "version 1" << std::endl;
			sscanf(str, "%lf %lf %lf %lf %lf %lf %lf %lf%n", &force, &x, &y, &z, &qw, &qx, &qy, &qz, &offset);
		  }
		ContactInfo ci;
		ci.version = version;
		ci.location = vec3(x,y,z);
		ci.orientation = Quaternion(qw, qx, qy, qz);
		ci.force = force;
		mContactList.push_back(ci);
	}
	//printMe();
}

double GraspExperienceEntry::minimumSumOfL2Distance(const GraspExperienceEntry& g) const
{
	std::vector<ContactInfo> cl1, cl2;
	cl1 = g.getContactList();
	cl2 = mContactList;
	double sum = 0.0, dist = -1.0;
	//compare the contacts of the two grasps and accumulate their most similar contacts
	//non similar contacts are ignored

	//g.printMe();
	//this->printMe();
	//getchar();

	//step one: cl1 w.r.t cl2
	for(size_t i = 0; i < cl1.size(); ++i)
	{
		dist = -1.0;
		for(size_t j = 0; j < cl2.size(); ++j)
		{
			double tmp = L2Distance(cl1[i], cl2[j]);
			if(tmp < dist || dist < 0)
				dist = tmp;
		}
		sum += dist;
	}

	//step two: cl2 w.r.t cl1
	for(size_t i = 0; i < cl2.size(); ++i)
	{
		dist = -1.0;
		for(size_t j = 0; j < cl1.size(); ++j)
		{
			double tmp = L2Distance(cl2[i], cl1[j]);
			if(tmp < dist || dist < 0)
				dist = tmp;
		}
		sum += dist;
	}

	//if sum < 0, meaning one contact list is empty
	sum /= (cl1.size() + cl2.size());
	if(sum < 0)
		return 999999.99;//random number
	return sum;
}

double GraspExperienceEntry::estimateSumOfL2Distance(const GraspExperienceEntry& g) const
{
	std::vector<ContactInfo> cl1, cl2;
	cl1 = g.getContactList();
	cl2 = mContactList;
	double sum = 0.0, dist = -1.0;
	//compare the contacts of the two grasps and accumulate their most similar contacts
	//non similar contacts are ignored
	int numTotalMatch1, numTotalMatch2;

	//step one: cl1 w.r.t cl2
	numTotalMatch1 = 0;
	numTotalMatch2 = 0;
	//compute the distance of mathces
	for(size_t i = 0; i < cl1.size(); ++i)
	{
		dist = -1.0;
		//looking for the nearest match
		for(size_t j = 0; j < cl2.size(); ++j)
		{
			double tmp = L2Distance(cl1[i], cl2[j]);
			if(tmp < dist || dist < 0)
				dist = tmp;
		}
		//if the match is too far, we ignore this match
		if(dist < NN_DIST)
			numTotalMatch1 ++;
		else
			dist = 0;
		sum += dist;
	}

	if(numTotalMatch1 < 1)
		sum = -1;

	//step two: cl2 w.r.t cl1
	//compute the distance of mathces
	for(size_t i = 0; i < cl2.size(); ++i)
	{
		dist = -1.0;
		//looking for the nearest match
		for(size_t j = 0; j < cl1.size(); ++j)
		{
			double tmp = L2Distance(cl2[i], cl1[j]);
			if(tmp < dist || dist < 0)
				dist = tmp;
		}
		//if the match is too far, we ignore this match
		if(dist < NN_DIST)
			numTotalMatch2 ++;
		else
			dist = 0;
		sum += dist;
	}

	if(numTotalMatch2 < 1)
		sum = -1;

	if(sum < 0)//no matching contacts
		return 999999.99;//random number

	//we only want to calculate distance for matched contacts
	double estimate = sum / (numTotalMatch1 + numTotalMatch2) * (cl1.size() + cl2.size());
	//std::cout << "match1: " << numTotalMatch1 << " , match2: " << numTotalMatch2 << std::endl;
	return estimate;
}

double GraspExperienceEntry::L2Distance(const ContactInfo& cl1, const ContactInfo& cl2) const
{
	vec3 dist = cl1.location - cl2.location;
	return dist.len();
}

double GraspExperienceEntry::getTactileSimilarityMeasure(const GraspExperienceEntry& g, bool considerSpread) const
{
	//g.printMe();
	//this->printMe();
	//getchar();
	
	//if not consider the spread angle
	if(!considerSpread)
		return minimumSumOfL2Distance(g);

	//if we need to consider the spread angle
	double contact_dist = minimumSumOfL2Distance(g);
	double spread_dist = fabs(g.getSpread() - mSpread);
	return contact_dist + 100 * spread_dist; //0.57 degree is equivallent to 1 mm

}

void GraspExperienceEntry::printMe(bool verbose) const
{
	std::cout << "Grasp: " << mGraspInfo.getString() << " begins" << std::endl;
	std::cout << "distance: " << mDistance << std::endl;
	std::cout << "epsilon: " << mGraspInfo.epsilonQuality << ", volume: " << mGraspInfo.volumeQuality << std::endl;
	std::cout << "spread angle: " << mSpread << std::endl;
	if(verbose)
	  {
	    for(size_t i = 0; i < mContactList.size(); ++i)
	      {
		mContactList[i].printMe();
	      }
	  }
	std::cout << "Grasp: " << mGraspInfo.getString() << " ends" << std::endl;
}

GraspExperienceEntry GraspExperienceEntry::getCurrentGraspAsGraspExperienceEntry()
{
	//synthesize the current grasp
	GraspExperienceEntry gee;
	std::vector<ContactInfo> cl = getCurrentGraspContactInfoList();
	gee.setContactList(cl);
	gee.setSpread(getCurrentHandSpread());
	return gee;
}