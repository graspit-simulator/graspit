#include "grasp.h"
#include "graspit_db_model.h"
#include "db_manager.h"

template <class Planner>
bool egPlannerUtils::setPlannerStateSpaceYParallelToV(Planner * planner, GraspableBody * const refBody, vec3 &v){
	alignHandYToV(planner->getHand(), v);
	GraspPlanningState * gps = gpsWithYParallelToCurrentY(planner->getHand(), refBody);	
	gps->getHand()->getGrasp()->setObjectNoUpdate(refBody);
	gps->getHand()->getGrasp()->setGravity(false);
	gps->setObject(refBody);
	static_cast<SimAnnPlanner*>(planner)->setModelState(gps);
	return true;
}

