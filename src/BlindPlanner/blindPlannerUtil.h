/*
 * blindPlannerUtil.h
 *
 *  Created on: Jun 21, 2011
 *      Author: dang
 */

#ifndef BLINDPLANNERUTIL_H_
#define BLINDPLANNERUTIL_H_
#include <vector>
#include <string>
#include "matvec3D.h"
class World;

//whether we want to filter out the grasps that do not belong to the current graspable body while loading the experience database
//#define FILTER
//what distance we want to look at when we are deciding which object we choose the best located grasp
#define COMPLEX_DISTANCE

struct SimpleAssociate
{
  size_t index;
  double distance;
};

struct ContactInfo
{
	int version;
	vec3 location;
	Quaternion orientation;
	double force;

	void printMe() const{std::cout << "version: " << version << " force: " << force << " {" << location.x() << ", " << location.y() << ", " << location.z() << "} " <<
					"[" << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << "]" << std::endl;
	}
};

struct GraspInfo
{
	std::string objectName;
	int graspIndex;
	double epsilonQuality, volumeQuality;

	void printMe() const{ std::cout << "GraspInfo: " << objectName << "-" << graspIndex << std::endl; }
	std::string getString() const{ char together[100]; sprintf(together, "%s_%d", objectName.c_str(), graspIndex); return together; }
};

struct AdjustmentInfo
{
	//storing the id of the grasp neighbors
	std::vector<std::string> neighborIdPerNN;
	std::vector<transf> adjustmentPerNN;
	std::vector<double> spreadAdjustmentPerNN;
	std::vector<double> distancePerNN;
	int targetNN;
	void reset() { neighborIdPerNN.clear(); adjustmentPerNN.clear(); distancePerNN.clear(); }
};

std::vector<ContactInfo> getCurrentGraspContactInfoList(World * w = NULL);
std::vector<ContactInfo> getContactInfoList(std::vector<double> tactileReadings, World * w = NULL);
void getContactInfoListPerPad(std::vector<double> tactileReadings,
							  std::vector<ContactInfo>& palm,
							  std::vector<ContactInfo>& f1,
							  std::vector<ContactInfo>& f2,
							  std::vector<ContactInfo>& f3,
							  World * w = NULL);
std::vector<double> getTactileSensorReadings(World * w = NULL);
double getCurrentHandSpread();
void resetHandDOFBreakAwayFlags();
void resetHandPoseToZero();
bool selfCollision();
int numPadsWithTactileContacts(World * w = NULL);
void writeOutAsTactileExperience(std::string id, std::string surfix); //id specifies the current grasp id
//list the file names in the directory
//filter defines a sub string that the file name should contain
void getFileNames(std::vector<std::string> &out, const std::string &directory, const std::string &filter);
std::vector<ContactInfo> getContactWithLargestNormalForce(std::vector<ContactInfo> clist);
std::string getGraspCommand(transf finalPose, double* joints, bool asAdjustment = false, std::string cmd_prefix = "", double s = 1.0);
#endif /* BLINDPLANNERUTIL_H_ */
