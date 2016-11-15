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
// $Id: graspit_db_grasp.h,v 1.13 2010/01/13 23:03:37 cmatei Exp $
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
class GraspitDBModel;

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
        GraspitDBGrasp(Hand* h) : mPreGrasp(NULL), mFinalGrasp(NULL), mHand(h) {}
	~GraspitDBGrasp();
	//! Returns the pre-grasp of this GraspitDBGrasp
	GraspPlanningState* getPreGraspPlanningState() { return mPreGrasp; }
	//! Returns the final-grasp of this GraspitDBGrasp
	GraspPlanningState* getFinalGraspPlanningState() { return mFinalGrasp; }
	//! Returns the pre-grasp of this GraspitDBGrasp
	const GraspPlanningState* getPreGraspPlanningState() const { return mPreGrasp; }
	//! Returns the final-grasp of this GraspitDBGrasp
	const GraspPlanningState* getFinalGraspPlanningState() const { return mFinalGrasp; }

	//! Records the test score in the memory, not in CGDB, these scores will not be stored
	void addTestScore(double s) { mTestScores.push_back( s < 0 ? 0 : s); }
	//! Returns the score of the i-th test
	double getTestScore(int i) { return mTestScores[i]; }
	//! Returns the number of tests that have just been done
	int getNumTestScores() { return (int)mTestScores.size(); }
	//! Returns the average of the scores of the past tests
	double getTestAverageScore();
	//! Transform this grasp 
	bool Transform(const float array[16]);
	//! Set the pregrasp planning state and modify the original pregrasp in db_planner::grasp as well
	void setPreGraspPlanningState(GraspPlanningState* p);
	//! Set the finalgrasp planning state 
	/*! Modify the original finalgrasp and contacts in db_planner::grasp as well */
	void setFinalGraspPlanningState(GraspPlanningState* p);
	/*! Set the joints and positions of the pre-grasp and final grasp, the base class should
	    have no idea of what GraspitDBGrasp looks like
	*/
	virtual bool SetGraspParameters(const vector<double>& pregrasp_joints, 
									const vector<double>& pregrasp_position, 
									const vector<double>& grasp_joints,
									const vector<double>& grasp_position);
	//! Returns the hand associated with this grasp
	const Hand *getHand() const {return mHand;}

	//! Returns the corresponding hand name in CGDB
	static QString getHandDBName(Hand* h);
	//! Returns the GraspIt path for loading a hand based on its db name
	static QString getHandGraspitPath(QString handDBName);
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
};

#endif
