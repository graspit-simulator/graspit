//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: graspit_db_planner.h,v 1.5 2009/05/07 19:53:41 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBPlanner class
 */

#ifndef _GRASPIT_DB_PLANNER_H_
#define _GRASPIT_DB_PLANNER_H_
#include <ostream>
#include <vector>
#include <QObject>
#include <QString>
#include "matvec3D.h"
#include "DBPlanner/aligner.h"

class Hand;
class GraspableBody;
class GraspitDBGrasp;
class GraspitDBModel;
class QualEpsilon;
class QualVolume;
class GraspPlanningState;

namespace db_planner {
	class DatabaseManager;
	class Model;
	class Grasp;
}

class GraspitDBPlanner : public QObject{
	Q_OBJECT
public:
	enum DynamicCode{DYNAMIC_SUCCESS=0, DYNAMIC_NO_FC=1, DYNAMIC_OBJECT_EJECTED=2, DYNAMIC_ERROR=3, DYNAMIC_APPROACH_FAILED=4, NO_DYNAMICS=5};
	enum TestType{STATIC=0, DYNAMIC=1};
private:
	//! The hand that we are planning for
	Hand *mHand;
	//! The grasped object
	db_planner::Model *mObject;
	//! The mgr of the dbase we are using
	db_planner::DatabaseManager *mDBMgr;
	//! Used for aligning our object with its nbrs found in the dbase
	db_planner::Aligner<db_planner::Model> *mAligner;
	//! Stores the tested grasps
	std::vector<GraspPlanningState*> mTestedGrasps;
	//! Helper variable for the dynamic autograsp
	bool mDynamicsError;
	//! Epsilon quality measure
	QualEpsilon* mEpsQual;
	//! Volume quality measure
	QualVolume* mVolQual;

	//! This moves the hand out of collision
	bool moveHandOutOfCollision();
	//! Move the hand by the step specified by the vector
	void moveBy(vec3 v);
	//! Check if we need initialize the body for dynamic test
	bool checkDynBody();
	//! For CGDB dynamic body test
	void dynamicBodyInit();
	//! Static test
	bool testGraspStatic();
	//! Dynamic test
	bool testGraspDynamic(DynamicCode *code);

public slots:
	void dynamicsError(const char*);

public:
	// initialzie a planner
	GraspitDBPlanner(Hand *h,
					 db_planner::Model *b,
					 db_planner::DatabaseManager *m,
					 db_planner::Aligner<db_planner::Model> *a) :
					 mHand(h), mObject(b), mDBMgr(m), mAligner(a),
					 mEpsQual(NULL), mVolQual(NULL) {}
	~GraspitDBPlanner();

	//! This will test one grasp
	bool testCurrentGrasp(TestType t, DynamicCode* c = NULL);
	//! This will test all the grasps in 
	bool testGrasps(TestType t, std::vector<db_planner::Grasp*>,
					std::vector<db_planner::Grasp*>* testedGraspList);
	/*! This does cross correlation of the grasps in graspList by applying them
	on the models in modelList.  For each test of a grasp in graspList, the test score
	will be recorded.  In the end, all the grasps will NOT be ranked by the average of the
	records associated with the grasps.  After it, the grasps in graspList will have their
	test scores filled.
	*/
	void crossCorrelate(const std::vector<db_planner::Model*> testModels,
						std::vector<db_planner::Grasp*> graspList);

	//! Computes the epsilon quality(eq) and the volume quality(vq) for the current grasp
	void computeQuality(float& eq, float& vq);
};
#endif