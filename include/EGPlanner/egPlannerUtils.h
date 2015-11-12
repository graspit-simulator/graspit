#ifndef EGPLANNER_UTILS_H
#define EGPLANNER_UTILS_H
#include "searchState.h"
#include "graspPlanningTask.h"
#include "loopPlanner.h"

class EGPlanner;
class Quaternion;
class Hand;
class Body;
class GraspitDBModel;
class vec3;

namespace egPlannerUtils{
	/*Assorted utility functions initially designed to make implementing planning on certain
	restricted subspaces more readable.  Names are long in the "self documenting functions"
	style, because what they do is straight forward, but the rationale for doing them
	can be a bit tricky.
	*/


	//Create a GraspPlanningState that fixes the y-axis to be parallel to the current
	//y-axis.  This is used to force the approach direction to be perpendicular to some
	// direction y, which could, for example, be the principle component of the object
	//geometry
	inline GraspPlanningState * gpsWithYParallelToCurrentY(Hand * h, const Body * const refBody);
	/* Not implemented yet, see cpp file for details
	inline  GraspPlanningState * gpsWithApproachPerpendicularToCurrentY(Hand * h, Body * refBody);
	*/
	//Get a transform that rotates vector A on to vector B.  Assumes normalised vectors
	inline Quaternion transV1OnToV2(const vec3 &v1, const vec3 &v2);
	//Align the Hand to vector v in place and move it out of collision
	inline bool alignHandYToV(Hand * h, const vec3 &v);
	//initialize planner to obect with search space with Y parallel to v and reorients hand appropriately
	//takes a planner, changes its parameters without changing it to an illegal state
	//returns a ready but unstarted planner
	template <class Planner>
	bool setPlannerStateSpaceYParallelToV(Planner * planner,  GraspableBody * const refBody, vec3 &v);
	//factory that creates the appropriate egplanner
	class AlignedYParallelToVEgPlannerFactory{
		private:
			db_planner::DatabaseManager & mDBMgr;
		public:
			AlignedYParallelToVEgPlannerFactory(db_planner::DatabaseManager * dbm):mDBMgr(*dbm){};
			EGPlanner * newPlanner(Hand * mHand, GraspitDBModel * m);
		
	};
	
	class SimulatedAnnealingPlannerFactory{
		private:
			db_planner::DatabaseManager & mDBMgr;
		public:
			SimulatedAnnealingPlannerFactory(db_planner::DatabaseManager * dbm):mDBMgr(*dbm){};
			EGPlanner * newPlanner(Hand * mHand, GraspitDBModel * m);
		
	};

	class GuidedPlannerFactory{
		private:
			db_planner::DatabaseManager & mDBMgr;
		public:
			GuidedPlannerFactory(db_planner::DatabaseManager * dbm):mDBMgr(*dbm){};
			EGPlanner * newPlanner(Hand * mHand, GraspitDBModel * m);
		
	};
	//factory that creates an alligned planner that is allowed to vary
	template <class Factory, int mConfidence>
	class LooselyAlignedPlannerFactory{
		private:
			Factory mFac;
		public:
			LooselyAlignedPlannerFactory(db_planner::DatabaseManager * dbm):mFac(dbm){};
			EGPlanner *newPlanner(Hand * hand, GraspitDBModel * m){
				EGPlanner * pln = mFac.newPlanner(hand, m);
				pln->getTargetState()->getPosition()->setAllConfidences(mConfidence*.01); //mConfidence is given in percentages
				return pln;	
			};
	};
		

	//Functor class for saving grasps
	class SaveGuidedPlannerTaskToDatabase{
	private:
		db_planner::DatabaseManager & mDBMgr;
		int mTaskNum;
	public:
		SaveGuidedPlannerTaskToDatabase(db_planner::DatabaseManager * dbm, const int taskNum):mDBMgr(*dbm),mTaskNum(taskNum){};
		bool saveGraspList(EGPlanner * finishedPlanner);
	};


	//Functor class for saving grasps
	class SaveSimulatedAnnealingTaskToDatabase{
	private:
		db_planner::DatabaseManager & mDBMgr;
		int mTaskNum;
	public:
		SaveSimulatedAnnealingTaskToDatabase(db_planner::DatabaseManager * dbm, const int taskNum):mDBMgr(*dbm),mTaskNum(taskNum){};
		bool saveGraspList(EGPlanner * finishedPlanner);
	};
}

//templated function implementations go in here
#include "egPlannerUtilsImpl.h"
#endif
