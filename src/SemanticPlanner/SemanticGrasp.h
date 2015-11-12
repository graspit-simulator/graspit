#ifndef SEMANTIC_GRASP_H_
#define SEMANTIC_GRASP_H_
#include <string>
#include <vector>
#include "matvec3D.h"
#include "BlindPlanner/graspExperienceEntry.h"

class SemanticMap;

class SemanticGrasp
{
private:
	std::string mSemantics, mHandName;

	// to locate the code with the two parameters
	int mApproachingDirectionID;
	SemanticMap * mSemanticMap;
	//mGraspParamsFilePath refers to a file that stores the parameters for the EGPlanner in the refinement step
	//mGraspTactileExperienceFilePath refers to a file that stores the tactile experience data including controlled dof and tactile feedback
	std::string mGraspParamsFilePath, mGraspTactileExperienceFilePath;
	//int mDOFType;

	// to compensate for the camera's pose w.r.t. the palm
	transf mRollingAdjustment;
	std::vector<double> mDofs, mTactile;
	//std::vector<ContactInfo> mContactList;
	GraspExperienceEntry mGee;

public:
	SemanticGrasp() : mSemanticMap(NULL)
	{

	}
	void setSemantics(std::string s) { mSemantics = s; }
	void setHandName(std::string n) { mHandName = n; }
	void setApproachingDirection(int id){ mApproachingDirectionID = id; }
	void setRollingAdjustment(double angleInDegree);
	void setDOFs(std::vector<double> dofs) { mDofs = dofs; }
	void setTactile(std::vector<double> t) { mTactile = t; }
	void setMap(SemanticMap *map) { mSemanticMap = map; }
	void setGraspParamsFilePath(std::string path) { mGraspParamsFilePath = path; }
	void setGraspTactileExperienceFilePath(std::string path) { mGraspTactileExperienceFilePath = path; }
	//void setContactList(std::vector<ContactInfo> cl) { mContactList = cl; }
	void setGraspExperienceEntry(GraspExperienceEntry gee) { mGee = gee; }
	//void setDOFType(int t) { mDOFType = t; }

	std::vector<double> getDOFs() { return mDofs; }
	int getApproachingDirectionID() { return mApproachingDirectionID; }
	transf getRollingAdjustment() { return mRollingAdjustment; }
	std::string getGraspParamsFilePath(){ return mGraspParamsFilePath; }
	std::string getGraspTactileExperienceFilePath(){ return mGraspTactileExperienceFilePath; }
	//std::vector<ContactInfo> getContactList() { return mContactList; }
	//int getDOFType() { return mDOFType; }
	GraspExperienceEntry getGraspExperienceEntry() { return mGee; }

};

#endif //SEMANTIC_GRASP_H_
