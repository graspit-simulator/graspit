/*
 * handAdjust.h
 *
 *  Created on: Jun 20, 2011
 *      Author: dang
 */

#ifndef HANDADJUST_H_
#define HANDADJUST_H_
#include <vector>
#include <string>

#include "matvec3D.h"
#include "graspExperienceEntry.h"
#include "graspExperienceBase.h"
#include "blindPlannerUtil.h"
#include "DBase/graspit_db_grasp.h"
#include "DBase/graspit_db_model.h"

namespace db_planner
{
class DatabaseManager;
}
class GraspableBody;
class Grasp;

class HandAdjust{
private:
	std::vector<GraspExperienceEntry> mGraspNeighbors;
	//the actual grasp
	GraspExperienceEntry mActualGrasp;
	GraspExperienceBase *mGeb;
	db_planner::DatabaseManager *mDBMgr;
	std::vector<db_planner::Model*> mModelList;
	std::vector<db_planner::Grasp*> mGraspList;
	bool mIsInitialized;
	bool mIsReady;
	//save the world
	GraspableBody* mBody;
	transf mHandPose;
	std::vector<double> mHandJoint;
	//current test
	GraspableBody* mTestbody;

	//this records the information when the adjustment is being serched
	AdjustmentInfo mAdjustmentInfo;

	GraspitDBGrasp* getGrasp(std::string modelName, int index);
	GraspitDBModel* getModel(std::string objectName);

	// i is the neighbor index
	void loadNNModelFromCGDB(int i);

	// load the grasp from a world file
	// i is the index of the file in the world file list
	void loadNNFromWorldFile(int i);

	//returns whether it is grasped
	bool perturbAndGraspObject(transf t, double spread);
	bool graspObjectNew(transf t, double spread);

	//void init();

public:
	HandAdjust() : mGeb(NULL), mDBMgr(NULL), mIsInitialized(false), mIsReady(false) { }
	void init(GraspExperienceBase* geb);
	//perturb the stable grasps to better locate the current grasp and return the offset
	//if writeTo is given, we will only write out the procedure without examine the distance
	//if inSampleMode is set, we will sample the spread angle around each NN ignoring the query grasp
	transf getHandAdjustmentONLINE(double* spread = NULL, char *writeTo = NULL, bool inSampleMode = false);
	//look around within a sphere and find the neighbor whose epsilon quality is above a threshold
	transf getHandAdjustment2(GraspExperienceEntry nn, double radius = 20, double epsilon = 0.1);
	//assume we already have some perturbations computed before hand, this function will search for the directory for
	//valid experiences and search for good perturbation within the directory under graspit (e.g. 
	transf getHandAdjustmentUsePrecomputedPerturbation(GraspExperienceEntry nn, double* spread);
	void clearWorld(bool save = true);
	void restoreWorld();
	GraspitDBGrasp* loadNNGraspFromCGDB(int i);
	transf loadNNFromPoseList(int i);
	void setTactileNNList(GraspExperienceEntry gee, std::vector<GraspExperienceEntry> gn){ mGraspNeighbors = gn; mActualGrasp = gee;}
	void connectToDB(QString dbURL, QString dbName, QString dbPort, QString dbLogin, QString dbPassword);
	void setDBMgr(db_planner::DatabaseManager *dBMgr){mDBMgr = dBMgr;}
	AdjustmentInfo getAdjustmentInfo() { return mAdjustmentInfo; }
	
	/*
	Centering at the center of the contacts, sample r, p, y and depth to perturb the hand
	*/
	static std::vector<transf> getPerturbationListBasedOnContacts(double latitude, double longitude, double rolling, double depth, double centerDistance = -1);
	/*
	Assuming the perturbation is simple in the way that there is only translational error in y,z direction and
	rotational error in x direction of the hand coordinate system
	ty(mm), tz(mm), rx(degrees) specifies the range of the error
	*/
	static std::vector<transf> getPertrubationListBasedOnHand(double rx, double ty, double tz);
	/*
	Assuming the hand approaches from the +x direction, facing -x direction
	Rotation is along z axis
	x, y, z are virtual axes, specifying the axes in world coordinate system
	roll is the extra dimension for the hand's rolling dimension
	*/
	static std::vector<transf> getPertrubationListBasedOnObject(double tx, double ty, double rz, vec3 x, vec3 y, vec3 z, bool considerRolling, double roll);
};

#endif /* HANDADJUST_H_ */
