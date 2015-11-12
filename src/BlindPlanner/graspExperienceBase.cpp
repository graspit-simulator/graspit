/*
* graspExperienceBase.cpp
*
*  Created on: Jun 11, 2011
*      Author: dang
*/

#include "graspExperienceBase.h"

#include <iostream>
#include <fstream>
#include <algorithm>

bool compareFunctor(SimpleAssociate sa1, SimpleAssociate sa2)
{
	return sa1.distance < sa2.distance;
}

bool GraspExperienceBase::loadGraspExperienceBase(std::string path, int version, const char * filter)
{
	std::ifstream file(path.c_str());
	if(!file.is_open())
	{
		std::cout << "File not found: " << path << std::endl;
		return false;
	}

	std::string line;
	line.resize(5000);
	//file opened

	if(version < 0)
	{
		getline(file, line);
		sscanf(line.c_str(),"%d", &version);
	}
	else
	{
		version = 1;
	}

	mGraspEntries.clear();
	for(int i = 0; 1; ++i)
	{
		getline(file, line);
		if(line.empty())
			break;

		if(line[0] == '#')
		{
			--i;
			continue;
		}
		//std::cout << "line is: " << line << std::endl;
		if(filter && line.find(filter) == std::string::npos)
		{
			--i;
			continue;
		}
		mGraspEntries.push_back(GraspExperienceEntry(line.c_str(), version));
	}

	file.close();

	std::cout << "loaded: " << getNumGrasps() << " entries" << std::endl;
	return true;
}

bool GraspExperienceBase::loadHandPoses(std::string path, const char * filter)
{
	std::ifstream file(path.c_str());
	if(!file.is_open())
	{
		std::cout << "File not found: " << path << std::endl;
		return false;
	}

	//int numPoses;
	std::string line;
	//file opened
	//getline(file, line);
	//sscanf(line.c_str(), "%d", &numPoses);
	//std::cout << "in total there are " << numPoses << " poses" << std::endl;

	mHandPoses.clear();
	mSpread.clear();

	char id[1024];
	double tx, ty, tz, rx, ry, rz, rw;
	int j = 0;
	int offset;
	double spread;
	for(int i = 0; 1; ++i)
	{
		getline(file, line);

		if(line.empty())
			break;

		if(line[0] == '#')
		{
			--i;
			continue;
		}
		//std::cout << "line is: " << line << std::endl;
		
		if(filter && line.find(filter) == std::string::npos)
		{
			--i;
			continue;
		}
		sscanf(line.c_str(), "%s %lf %lf %lf %lf %lf %lf %lf%n", id, &tx, &ty, &tz, &rw, &rx, &ry, &rz, &offset);
		mHandPoses.push_back(transf(Quaternion(rw, rx, ry, rz), vec3(tx, ty, tz)));
		//printf(" %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", tx, ty, tz, rw, rx, ry, rz);

		if(sscanf(line.c_str()+offset,"%lf", &spread))
		{
			//std::cout << "with spread angle" << std::endl;
		}
		else
		{
			spread = -1;
		}


		if(j < mGraspEntries.size() && !strcmp(mGraspEntries[j].getGraspInfo().getString().c_str(), id)) //whether the current hand pose has a valid tactile experience
		{
			mGraspEntries[j].setPose(mHandPoses[i]);
			mGraspEntries[j].setSpread(spread);
			++j;
		}

	}

	file.close();

	if( j != mGraspEntries.size() )
	{
		std::cout << "The sizes for hand poses and grasp exp entries do not match, hand poses not populated" << j << ", " << mGraspEntries.size() << std::endl;
		return false;
	}
	else
	{
		std::cout << "successfully stored " << j << " poses" << std::endl;
	}

	return true;
}

std::vector<GraspExperienceEntry> GraspExperienceBase::getKNNBasedOnTactile(const GraspExperienceEntry& e, int k)
{
	std::vector<GraspExperienceEntry> nn;
	std::vector<SimpleAssociate> distList;

	distList.resize(mGraspEntries.size());
	for(size_t i = 0; i < mGraspEntries.size(); ++i)
	{
		distList[i] = SimpleAssociate();
		distList[i].index = i;
		distList[i].distance = mGraspEntries[i].getTactileSimilarityMeasure(e);
		mGraspEntries[i].setDistance(distList[i].distance);
	}

	sort(distList.begin(), distList.end(), compareFunctor);
	for(size_t i = 0; i < mGraspEntries.size() && i < k; ++i)
	{
		nn.push_back(mGraspEntries[distList[i].index]);
	}

	return nn;
}

GraspExperienceEntry GraspExperienceBase::getNNBasedOnPose(const GraspExperienceEntry &e, double radius, double epsilon)
{
	//given the current grasp experience entry, e, look around it and find out which neighbor has a good epsilon quality
	std::vector<SimpleAssociate> distList;
	distList.resize(mGraspEntries.size());

	std::cout << "getNN" << std::endl;
	e.printMe();

	for(size_t i = 0; i < mGraspEntries.size(); ++i)
	{
		distList[i] = SimpleAssociate();
		distList[i].index = i;

		vec3 d = mGraspEntries[i].getPose().translation() - e.getPose().translation();
		Quaternion q = mGraspEntries[i].getPose().rotation().inverse() * e.getPose().rotation();
		double angle;
		vec3 axis;
		q.ToAngleAxis(angle, axis);
		if( angle > M_PI )
			angle = 2*M_PI - angle;

		//std::cout << d.len() << ", " << angle << std::endl;
		//if(angle < 0.2)
		//  std::cout << d.len() << ", " << angle << std::endl;

		distList[i].distance = d.len() + 100 * angle * angle; // 1000: 1.8 degree is equivallent to 1 mm
		// 100: 5.7 degree is equivallent to 1 mm
	}

	sort(distList.begin(), distList.end(), compareFunctor);
	mGraspEntries[distList[0].index].printMe();

	if(mGraspEntries[distList[0].index].getEpsilonQuality() > e.getEpsilonQuality() ||
		mGraspEntries[distList[0].index].getEpsilonQuality() < e.getEpsilonQuality())
		printf("true\n");
	else
		printf("false\n");

	if(mGraspEntries[distList[0].index].getEpsilonQuality() == e.getEpsilonQuality())
		printf("true2\n");
	else
		printf("false2\n");
	for(double ep = epsilon; ep > 0.0; ep -= 0.1)
	{
		//start from j = 1 instead of 0, because j = 0 is e itself (minimum distance with itself)
		for(size_t j = 1; j < mGraspEntries.size(); ++j)
		{
			if(mGraspEntries[distList[j].index].getEpsilonQuality() > ep &&
				mGraspEntries[distList[j].index].getEpsilonQuality() > e.getEpsilonQuality() &&
				distList[j].distance < radius)
			{
				vec3 d = mGraspEntries[distList[j].index].getPose().translation() - e.getPose().translation();
				Quaternion q = mGraspEntries[distList[j].index].getPose().rotation().inverse() * e.getPose().rotation().inverse();
				double angle;
				vec3 axis;
				q.ToAngleAxis(angle, axis);
				if( angle > M_PI )
					angle = 2*M_PI - angle;
				std::cout << d.len() << ", " << angle << std::endl;
				return mGraspEntries[distList[j].index];
			}
		}
	}

	std::cout << "No appropriate neighbor found" << std::endl;
	return mGraspEntries[distList[0].index];
}

transf GraspExperienceBase::getPose(std::string id)
{
	for(size_t i = 0; i < mGraspEntries.size(); ++i)
	{
		if( !strcmp(mGraspEntries[i].getInfoString().c_str(), id.c_str()) )
		{
			return mGraspEntries[i].getPose();
		}
	}
	std::cout << "Could not locate the entry by id: " << id << std::endl;
	return transf::IDENTITY;
}

GraspExperienceEntry GraspExperienceBase::getEntry(std::string id)
{
	for(size_t i = 0; i < mGraspEntries.size(); ++i)
	{
		if( !strcmp(mGraspEntries[i].getInfoString().c_str(), id.c_str()) )
		{
			return mGraspEntries[i];
		}
	}
	std::cout << "Could not locate the entry by id: , return a nonsense one" << id << std::endl;
	return mGraspEntries[0];
}

transf GraspExperienceBase::getWeightedPose(std::vector<GraspExperienceEntry> geeList, double cutoff)
{
	Quaternion q(1,0,0,0);
	vec3 t(0,0,0);
	//consider all the NN's that are within a threshold of(cutoff * distance of the first NN)
	double threshold = geeList[0].getDistance() * cutoff;
	double weightSum = 0;
	for(int i = 0; i < geeList.size(); ++i)
	{
		if(geeList[i].getDistance() > threshold)
			break;

		double weight = 1.0 / geeList[i].getDistance();
		weightSum += weight;
		t = t + weight * geeList[i].getPose().translation();
		q = q + weight * geeList[i].getPose().rotation();
	}
	//normalization
	q.normalise();
	t = t / weightSum;
	return transf(q, t);
}
