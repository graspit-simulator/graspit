#ifndef _GRASPPLANNINGSERVICE_H_
#define _GRASPPLANNINGSERVICE_H_

#include <QString>
#include <QObject>
#include <vector>

#include "matvec3D.h"

class ClientSocket;
class Hand;
class GraspPlanningState;
class GraspableBody;
class Body;
namespace db_planner {
	class DatabaseManager;
	class Model;
	class Grasp;
}

class GraspPlanningService : public QObject
{
	Q_OBJECT

private:
	enum GraspConstraintType {NONE=0, FINGERTIP_ALIGNED_ONLY = 1};
	QString mHandName, mObjectName, mMethodType, mMessage;
	ClientSocket* mSocket;
	transf mObjectPose;
	//! This list of grasps is the grasps from CGDB
	std::vector<db_planner::Grasp*> mGraspList, filteredGraspList;
	//! The list of models available in the dbase, as retrieved by the DBMgr
	std::vector<db_planner::Model*> mModelList;
	bool mIsParamSet;

	//! The mgr that all connections to the dbase have to go through
	db_planner::DatabaseManager *mDBMgr;
	Hand* mHand;
	
	//commented out as a hack
	//void init();
	//store database information
	QString mdbaseURL, mdbaseUserName, mdbasePassword, mdbaseName;
	int mdbasePort;
	
	QString synthesizeMsg(std::vector<db_planner::Grasp*> grasps, std::vector< std::vector<size_t> > coverageList = std::vector< std::vector<size_t> >());
	QString extractNumbers(std::vector<double> numArray);

	vec3 mApproachVector;
	double mApproachAngle;
	void clearGraspList();
	db_planner::Model * dbmodel;
	//un-define the default constructor
	GraspPlanningService(){};
	unsigned int mGraspConstraints;
	std::vector< std::vector<size_t> >  mGraspSetCoverageList;
        int filterGraspByCurrentObjectTablePose(Body* table, GraspableBody* model);

public:
	/* mObjectName specifies the object name in CGDB's scaled_model_name, mHandName specifies the hand name in CGDB, mObjectPose specifies the
       object's pose in some coordinate system, this service will transform the grasp to that coordinate system.
	*/

 GraspPlanningService(QString dbURL, QString dbName, QString dbPort, QString dbLogin, QString dbPassword, unsigned int graspConstraint = 0): mIsParamSet(false), dbmodel(NULL), mdbaseURL(dbURL), mdbaseUserName(dbLogin), mdbasePassword(dbPassword), mdbaseName(dbName), mdbasePort(dbPort.toInt()), mHand(NULL), mDBMgr(NULL), mGraspConstraints(graspConstraint)
	{
	  //init();
	}
	void setParams(QString hand, QString object, QString method_type, ClientSocket* clientSocket, transf t, const vec3 & approach_vector, double approach_tolerance_angle);
	~GraspPlanningService(){}
	void init();
	void retrieve();
	void plan_from_tasks();
	void transform();
	void check(const transf &);
	void rank();
	QString report();

	/* based on the retrieved grasp list, plan a limited list of grasps that covers
	   the all possible uncertain poses input thus far, default values for the
	   uncertainties are defined below
	*/
	void planWithUncertainties(double xUncertainty=10.0, double yUncertainty=10.0, double zUncertainty=10.0, double rxUncertainty=30.0, double ryUncertainty=30.0, double rzUncertainty=30.0);

	void planWithUncertainties(const std::vector<transf>& poseUncertainty);
	/* input is a vector of for coverage statistics, each vector element stores the set of items covered by the corresponding element, output is a list of indices which is the minimumCover solution based on greedy algorithm, the range is a positive integer, assuming the items are starting at 0 and ending at range
	 */
	bool minimumCover(std::vector< std::vector<int> > coverList, int range, std::vector<size_t> & solution);

};
#endif //_GRASPPLANNINGSERVICE_H_
