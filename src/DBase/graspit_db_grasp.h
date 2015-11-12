//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: graspit_db_grasp.h,v 1.10 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %GraspitDBGrasp class
 */

#ifndef _GRASPIT_DB_GRASP_ENTRY_H_
#define _GRASPIT_DB_GRASP_ENTRY_H_
#include "DBPlanner/grasp.h"

#include <QString>
#include <vector>
#include <string>

#include "graspitGUI.h"
#include "world.h"
#include "ivmgr.h"

class GraspPlanningState;
class Hand;

using std::string;
/*! This is the class to define the data entry of grasps in CGDB
*/
class GraspitDBGrasp : public db_planner::Grasp{
	//! Search State stores the actual grasps
	GraspPlanningState *mPreGrasp, *mFinalGrasp;
	//! Stores the test scores in cross correlation
	std::vector<double> mTestScores;
	//! The hand associated with this grasp
	Hand* mHand;

public:
	GraspitDBGrasp(const GraspitDBGrasp& grasp2);
	GraspitDBGrasp(Hand* h) : mHand(h), mPreGrasp(NULL), mFinalGrasp(NULL) {}
	~GraspitDBGrasp();
	//! Returns the pre-grasp of this GraspitDBGrasp
	const GraspPlanningState* getPreGraspPlanningState()const { return mPreGrasp; }
	//! Returns the final-grasp of this GraspitDBGrasp
	GraspPlanningState* getFinalGraspPlanningState() { return mFinalGrasp; }
	//! Records the test score in the memory, not in CGDB, these scores will not be stored
	void addTestScore(double s) { mTestScores.push_back( s < 0 ? 0 : s); }
	//! Returns the score of the i-th test
	double getTestScore(int i) { return mTestScores[i]; }
	//! Returns the number of tests that have just been done
	int getNumTestScores() { return (int)mTestScores.size(); }
	//! Returns the average of the scores of the past tests
	double getTestAverageScore();
	//! Transform this grasp and stores it in the place indicated by transformed
	bool Transform(const float array[16]);
	//! Set the pregrasp planning state and modify the original pregrasp in db_planner::grasp as well	
	void setPreGraspPlanningState(GraspPlanningState* p);
	//! Set the finalgrasp planning state and modify the original finalgrasp and contacts in db_planner::grasp as well
	void setFinalGraspPlanningState(GraspPlanningState* p);
	/*! Set the joints and positions of the pre-grasp and final grasp, the base class should
	    have no idea of what GraspitDBGrasp looks like
	*/
	virtual bool SetGraspParameters(const vector<double>& pregrasp_joints, 
									const vector<double>& pregrasp_position, 
									const vector<double>& grasp_joints,
									const vector<double>& grasp_position);
	//! Returns the corresponding hand name in CGDB
	static QString getHandDBName(Hand* h);
	//! Returns the GraspIt path for loading a hand based on its db name
	static QString getHandGraspitPath(const QString & handDBName);
	static bool setHandMaterialFromDBName(Hand * h, const QString &hand_db_name);
	static Hand * loadHandFromDBName(const QString & hand_db_name);
};

/*! For now, the hand is passed to the constructor. In the future,
	we will probably allow the hand to be changed for this 
	allocator.
*/
class GraspitDBGraspAllocator : public db_planner::GraspAllocator
{
private:
	//! The hand that all grasps are initialized with
	Hand *mHand;
public:
	GraspitDBGraspAllocator(Hand *h) : mHand(h) {}
	db_planner::Grasp* Get() const {
		return new GraspitDBGrasp(mHand);
	}
	virtual bool setHand(Hand * h){mHand = h; return true;}
};

#endif
