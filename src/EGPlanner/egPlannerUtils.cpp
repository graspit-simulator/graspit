#include "egPlannerUtils.h"
#include "egPlanner.h"
#include "searchState.h"
#include "searchEnergy.h"
#include "robot.h"
#include "body.h"
#include "matvec3D.h"
#include "grasp.h"
#include "graspit_db_grasp.h"
#include "debug.h"
#include "guidedPlanner.h"


#ifdef Q_WS_X11
  #include <unistd.h>
#endif


GraspPlanningState *
egPlannerUtils::gpsWithYParallelToCurrentY(Hand * h, const Body * const refBody){
 GraspPlanningState * handObjectState = new GraspPlanningState(h); 
 handObjectState->setPositionType(SPACE_COMPLETE);
 handObjectState->getPosition()->setTran(h->getTran()*refBody->getTran().inverse());
 //fix x and z, leave y and w free.
 handObjectState->getVariable(6)->setFixed(false);
 handObjectState->getVariable(8)->setFixed(true);
  handObjectState->getVariable(5)->setFixed(false);
 handObjectState->getVariable(7)->setFixed(true);
//fix finger spread
handObjectState->getVariable(0)->setValue(-1.13);
handObjectState->getVariable(0)->setFixed(true);
 return handObjectState;
}

/*Fixme - I don't think this can be represented with any of the current state spaces
GraspPlanningState *
egPlannerUtils::gpsWithApproachPerpendicularToCurrentY(Hand * h, Body * refBody){
assert(0);
 return handObjectState;
}
*/


bool egPlannerUtils::alignHandYToV(Hand * h, const vec3 &v){
	transf t;	
	transf targetHandTrans = transf(transV1OnToV2(vec3(1,0,0) /*X axis FIXME*/,v),h->getTran().translation());
	h->setTran(targetHandTrans);
	//return (h->findInitialContact(0));
	return true;
}

Quaternion egPlannerUtils::transV1OnToV2(const vec3 &v1, const vec3 &v2){
	return Quaternion(acos(v1%v2), v1*v2/*rotate around common normal*/);
}


EGPlanner *  egPlannerUtils::AlignedYParallelToVEgPlannerFactory::newPlanner(
	Hand * mHand, GraspitDBModel * m){
			float pca[9];
			mDBMgr.GetPCA(*m, pca);
			vec3 v(pca[0],pca[3],pca[6]);
			EGPlanner * mPlanner = new GuidedPlanner(mHand);	
			mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
			mPlanner->setContactType(CONTACT_PRESET);
			setPlannerStateSpaceYParallelToV(mPlanner, m->getGraspableBody(),v);
			//fix finger spread
			/*mPlanner->getTargetState()->getPosture()->getVariable(0)->setValue(-1.13);
			mPlanner->getTargetState()->getPosture()->getVariable(0)->setFixed(true);
			static_cast<SimAnnPlanner *>(mPlanner)->setModelState(mPlanner->getTargetState());
			mPlanner->getTargetState()->reset();*/
			//mPlanner->resetPlanner();
			mPlanner->setMaxSteps(-1);
			mPlanner->setRepeat(true);
			return mPlanner;
}




GraspitDBGrasp* synthesize(GraspPlanningState* pre, GraspPlanningState* fin)
{
	//synthesize a new graspit_db_grasp
	// store it into CGDB
	GraspitDBGrasp* gp;

	gp = new GraspitDBGrasp(pre->getHand());
	gp->SetHandName(GraspitDBGrasp::getHandDBName(pre->getHand()).toStdString());
	db_planner::Model* m = pre->getHand()->getGrasp()->getObject()->getDBModel();
	gp->SetSourceModel(*m);
	//the pre-grasp's position is not in eigengrasp space, so we must save it in DOF space
	//these are only used for the representation of pregrasp in final grasp's format
	//convert it's tranform to the Quaternion__Translation format
	//make sure you pass it sticky=true, otherwise information is lost in the conversion
	pre->setPositionType(SPACE_COMPLETE,true);
	//we will want to save exact DOF positions, not eigengrasp values
	//again, make sure sticky=true
	pre->setPostureType(POSE_DOF,true);
	gp->setPreGraspPlanningState(new GraspPlanningState(pre));

	//start analyzing and generate the final grasp
	SearchEnergy* se = new SearchEnergy();
	se->setType(ENERGY_STRICT_AUTOGRASP);
	bool legal;
	double energy;
	DBGA("A");
	se->analyzeState(legal,energy,fin,false);
	DBGA("B");
	//save the qualities -- in case fin is not the right kind of state, make it the right kind
	fin->setPositionType(SPACE_COMPLETE,true);
	DBGA("C");
	fin->setPostureType(POSE_DOF,true);
	DBGA("D");
	fin->saveCurrentHandState();
	DBGA("E");
	fin->setEpsilonQuality(se->getEpsQual());
	DBGA("F");
	fin->setVolume(se->getVolQual());
	DBGA("G");

	
	//Contacts is not copied in copy constructor
	GraspPlanningState * fin_tmp = new GraspPlanningState(fin);
	//the contacts
	std::vector<double> tempArray;
	tempArray.clear();
	for(int i = 0; i < fin->getHand()->getGrasp()->getNumContacts(); ++ i){
		Contact * c = fin->getHand()->getGrasp()->getContact(i);
		fin_tmp->getContacts()->push_back(c->getPosition());
	}
	DBGA("H");

	gp->setFinalGraspPlanningState(fin_tmp);
	DBGA("I");

	delete se;
	return gp;
}


bool egPlannerUtils::SaveGuidedPlannerTaskToDatabase::saveGraspList( EGPlanner * finishedPlanner){
	//std::cout << "num children " << finishedPlanner->getNumChildren() << std::endl;
	//while(finishedPlanner->getNumChildren())
	{
	//	DBGA("Waiting for children to stop, children left: " << finishedPlanner->getNumChildren());
	//	sleep(23);
	}
	//DBGA("Exit");
	bool retVal = true;	
	// for storing it into CGDB
	std::vector<db_planner::Grasp*> gpList;

	for(int i = 0; i < finishedPlanner->getListSize()-1; i+=2){
	  DBGA("Synthesizing grasp: " << i);
	  gpList.push_back(synthesize(const_cast<GraspPlanningState*>(finishedPlanner->getGrasp(i)), const_cast<GraspPlanningState*>(finishedPlanner->getGrasp(i+1))));
	  gpList.back()->SetSource("STABILITY_LEARNING");
	}
	DBGA("Saving grasps - grasp number: " << gpList.size());
	if(!mDBMgr.SaveGrasps(gpList)){
		DBGA("Failed to Save Grasps");
		retVal = false;
	}
	for(int i = 0; i < finishedPlanner->getListSize()/2; ++i){
		delete gpList[i];
	}
	gpList.clear();
	std::cout << "saved" << std::endl;
	return retVal;
}


EGPlanner * egPlannerUtils::SimulatedAnnealingPlannerFactory::newPlanner(Hand * mHand, GraspitDBModel * m){
  EGPlanner * mPlanner = new SimAnnPlanner(mHand);	
  GraspPlanningState * ns = new GraspPlanningState(mHand);
  ns->setPositionType(SPACE_COMPLETE);
  ns->setObject(mHand->getWorld()->getGB(0));
  ns->setRefTran(mHand->getWorld()->getGB(0)->getTran());
  ns->reset();
  ns->saveCurrentHandState();
  static_cast<SimAnnPlanner *>(mPlanner)->setModelState(ns);
  mPlanner->setEnergyType(ENERGY_CONTACT);
  mPlanner->setContactType(CONTACT_PRESET);
  mPlanner->setMaxSteps(130000);
  mPlanner->setRepeat(false);  
  mPlanner->invalidateReset();
  return mPlanner;
}

EGPlanner * egPlannerUtils::GuidedPlannerFactory::newPlanner(Hand * mHand, GraspitDBModel * m){
  EGPlanner * mPlanner = new GuidedPlanner(mHand);	
  GraspPlanningState * ns = new GraspPlanningState(mHand);
  mHand->setTran(translate_transf(vec3(0,0,100)));
  ns->setPositionType(SPACE_COMPLETE);
  ns->setObject(mHand->getWorld()->getGB(0));
  ns->setRefTran(mHand->getWorld()->getGB(0)->getTran());
  ns->reset();
  ns->saveCurrentHandState();
  static_cast<SimAnnPlanner *>(mPlanner)->setModelState(ns);
  mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
  mPlanner->setContactType(CONTACT_PRESET);
  mPlanner->setMaxSteps(1000000);
  mPlanner->setRepeat(false);  
  mPlanner->invalidateReset();
  return mPlanner;
} 


bool egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase::saveGraspList(EGPlanner * finishedPlanner){

	std::cout << "num children " << finishedPlanner->getNumChildren() << std::endl;
	while(finishedPlanner->getNumChildren())
	{
		DBGA("Waiting for children to stop");
	}
  DBGA("Saving" << finishedPlanner->getListSize() << "Grasps "  );
  bool retVal = true;	
	// for storing it into CGDB
	std::vector<db_planner::Grasp*> gpList;
	for(int i = 0; i < finishedPlanner->getListSize(); i+=1){
	  GraspPlanningState final = new GraspPlanningState(finishedPlanner->getGrasp(i));
	  DBGA("Synthesizing grasp: " << i);
	    gpList.push_back(synthesize(const_cast<GraspPlanningState*>(finishedPlanner->getGrasp(i)), &final));
	  gpList.back()->SetSource("EIGENGRASPS_TASK_37");
	  gpList.back()->SetEnergy(finishedPlanner->getGrasp(i)->getEnergy());
	}
	DBGA("Grasp list generated");
	
	if(!mDBMgr.SaveGrasps(gpList)){
		DBGA("Failed to Save Grasps");
		retVal = false;
	}
	for(int i = 0; i < gpList.size(); ++i){
		delete gpList[i];
	}
	gpList.clear();
	return retVal;
}
