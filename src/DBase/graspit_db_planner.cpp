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
// $Id: graspit_db_planner.cpp,v 1.9 2009/05/08 23:41:43 hao Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBPlanner class
 */

#include "graspit_db_planner.h"

#include <algorithm>


#include "debug.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "quality.h"
#include "searchState.h"
#include "collisionInterface.h"
#include "DBPlanner/sql_database_manager.h"
#include "graspit_db_grasp.h"
#include "graspit_db_model.h"
#include "DBPlanner/database.h"

GraspitDBPlanner::~GraspitDBPlanner(){
	for(int i = 0; i < (int)mTestedGrasps.size(); ++i){
		delete mTestedGrasps[i];
	}
}

// move the hand by v, no rotation is applied
void GraspitDBPlanner::moveBy(vec3 v)
{
	transf newTran;
	// calculate the new position of the hand after the transform of v
	newTran = translate_transf(v * mHand->getApproachTran()) * mHand->getTran();
	mHand->setTran(newTran);
}

// move the hand out of collision, move it back until no collision is detected
bool GraspitDBPlanner::moveHandOutOfCollision()
{
	CollisionReport colReport;
	int numCols;
	double offset = -10.0;
	std::vector<Body*> justPalm;
	justPalm.push_back(mHand->getPalm());

	//go back only while PALM is in collision
	do{
		numCols = mHand->getWorld()->getCollisionReport(&colReport, &justPalm);
		if (numCols) moveBy(vec3(0.0,0.0,offset));
	} while (numCols);

	//open fingers that are in collision
	numCols = mHand->getWorld()->getCollisionReport(&colReport);
	for (int c=0; c<mHand->getNumChains(); c++) {
		if (!mHand->snapChainToContacts(c, colReport)) {
			//std::cout <<"failed to get a chain our of collision" << std::endl;
			//failed to get a chain out of collision, do not return
			//return false;
		}
	}
	//go back only while anything is in collision
	int safeGuard = 0;
	while (++safeGuard < 1000){
		numCols = mHand->getWorld()->getCollisionReport(&colReport);
		if (numCols){
			moveBy(vec3(0.0,0.0,offset));
		}
		else{
			break;
		}
	}
	//always approach to contact no matter how far
	double step = 10.0;
	for( int i = 0; i < 10; i ++) {
		if(mHand->approachToContact(step)){
			break;
		}
	}
	return true;
}

void GraspitDBPlanner::dynamicsError(const char*)
{
	mDynamicsError = true;
}

// routine for dynamic test of grasp
bool GraspitDBPlanner::testGraspDynamic(DynamicCode *code)
{
	if(checkDynBody())
		dynamicBodyInit();
	//go back *a lot* while there are any collisions 
	int numCols;
	CollisionReport colReport;
	do{
		moveBy(vec3(0.0,0.0,-300));
		numCols = mHand->getWorld()->getCollisionReport(&colReport);
	} while (numCols);
	//open the fingers
	mHand->autoGrasp(false, -1.0);
	//go back in until first contact, in small increments
	int count = 0;
	while(!mHand->approachToContact(50) && ++count<10);
	if (count >= 10) {
		*code = DYNAMIC_APPROACH_FAILED;
		return false;
	}
	//if the fingers are not in contact, do a static autograsp until the first contact
	bool fingerContact = false;
	for (int c=0; c<mHand->getNumChains(); c++) {
		for (int l=0; l<mHand->getChain(c)->getNumLinks(); l++) {
			if (mHand->getChain(c)->getLink(l)->getNumContacts()) {
				fingerContact = true;
				break;
			}
		}
		if (fingerContact) break;
	}
	//if no finger contact, do a static autograsp that *stops at first contact*
	//if fingers are in contact, retreat a tiny bit so we don't trap fingers btw
	//contact and joint limits
	if (!fingerContact) {
		mHand->autoGrasp(false, 1.0, true);
	} 
	//one more test for collisions to make *absolutely sure we are OK
	numCols = mHand->getWorld()->getCollisionReport(&colReport);
	if (numCols) {
		*code = DYNAMIC_APPROACH_FAILED;
		return false;
	}
	QObject::connect(mHand->getWorld(), SIGNAL(dynamicsError(const char*)),
					 this, SLOT(dynamicsError(const char*)));
	//this is more of a hack to cause the hand to autograsp in dynamics way
	//the world's callback for dynamics should never get called as everything 
	//gets done from inside here
	mHand->getWorld()->resetDynamics();
	//I think this happens in each dynamics step anyway
	mHand->getWorld()->resetDynamicWrenches();
	mHand->getWorld()->turnOnDynamics();
	//this should set the desired values of the dof's
	mHand->autoGrasp(false);

	//loop until dynamics is done
	mDynamicsError = false;
	int steps=0; int stepFailsafe = 800;
	while (1) {
		mHand->getWorld()->stepDynamics();
		if (mDynamicsError) break;
		if (++steps > stepFailsafe) break;
		if (!(steps%50)) {DBGA("Dynamic step " << steps);}
		//we can't check if autograsp is done, so we just run for a pre-sepcified number of loops
	}
	QObject::disconnect(mHand->getWorld(), SIGNAL(dynamicsError(const char*)),
				    	this, SLOT(dynamicsError(const char*)));
	//turn of dynamics; world dynamics on shouldn't have done anything anyway
	mHand->getWorld()->turnOffDynamics();
	if (mDynamicsError) {
		DBGA("Dynamics error!");
		*code = DYNAMIC_ERROR;
		return false;
	} 
	DBGA("Dynamic autograsp complete in " << steps << " steps");
	*code = DYNAMIC_SUCCESS;
	return true;
}

// routine for static grasp test
bool GraspitDBPlanner::testGraspStatic(){
	// first move the hand just out of collision
	if (!moveHandOutOfCollision()) {
		return false;
	}
	// then apply autograsp to close those fingers
	mHand->autoGrasp(true);
	return true;
}

// test current grasp
bool GraspitDBPlanner::testCurrentGrasp(TestType t, DynamicCode* c){
	// test current grasp in coresponding test type - static/dynamic
	if(t == STATIC)
		return testGraspStatic();
	else{
		if(!c)
			return false;
		return testGraspDynamic(c);
	}
}

bool GraspitDBPlanner::checkDynBody()
{
	return true;
}

void GraspitDBPlanner::dynamicBodyInit(){
}

// compute the epsilon and volume quality of current grasp
void GraspitDBPlanner::computeQuality(float &eq, float &vq)
{
	CollisionReport colReport;
	// first test whether the hand is in collision now
	int numCols = mHand->getWorld()->getCollisionReport(&colReport);
	// if it is in collision, then there should be no reason to calculate the quality
	if(numCols>0){
		eq = -1.0;
		vq = -1.0;
		return;
	}
	// if there is no collision, then begin computation
	if(mEpsQual){
		delete mEpsQual;
	}
	mEpsQual = new QualEpsilon( mHand->getGrasp(), ("Examine_dlg_qm"),"L1 Norm");
	if(mVolQual){
		delete mVolQual;
	}
	mVolQual = new QualVolume( mHand->getGrasp(), ("Examine_dlg_qm"),"L1 Norm");
	mHand->getWorld()->findAllContacts();
	mHand->getWorld()->updateGrasps();

	eq = mEpsQual->evaluate();
	vq = mVolQual->evaluate();
}
/*! Every grasp in graspList will be tested and stored in testedGraspList
    even if their test scores, epsilon quality and volume quality, are
	not positive, which means they are not stable grasps.
*/
bool GraspitDBPlanner::testGrasps(TestType t,
								  std::vector<db_planner::Grasp*> graspList,
								  std::vector<db_planner::Grasp*>* testedGraspList){
	mTestedGrasps.clear();
	if(testedGraspList)
		testedGraspList->clear();

	// test each of the grasps in graspList
	for(int i = 0; i < ((int)graspList.size()); i ++)
	{
		GraspitDBGrasp* tempGrasp = new GraspitDBGrasp( *static_cast<GraspitDBGrasp*>(graspList[i]) );
		// load this grasp into the world
		static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->setTran(transf::IDENTITY);

		GraspPlanningState *tempState = new GraspPlanningState(tempGrasp->getPreGraspPlanningState());
		tempState->setRefTran(transf::IDENTITY);
		tempGrasp->setPreGraspPlanningState(tempState);

		float elmts[16];
		if(mAligner->Align(tempGrasp->SourceModel(), *mObject, elmts))
			tempGrasp->Transform(elmts);

		mHand->getGrasp()->setObject(static_cast<GraspitDBModel*>(mObject)->getGraspableBody());
		// apply it to the hand
		tempGrasp->getPreGraspPlanningState()->execute();
		bool testResult;
		DynamicCode dynCode;
		// test this grasp
		if (t == STATIC) {
			testResult = testGraspStatic();
			dynCode = NO_DYNAMICS;
		} else if (t == DYNAMIC) {
			testResult = testGraspDynamic(&dynCode);
			//DBGA("Result: " << testResult << "; dynamics code: " << dynCode);
		}
		float eq = -1.0, vq;
		// decide whether to record
		if (testResult) {
			computeQuality(eq, vq);
			if (!static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->getNumContacts()) {
				dynCode = DYNAMIC_OBJECT_EJECTED;
			} else if (eq <= 0.0) {
				dynCode = DYNAMIC_NO_FC;
			}
		}else return false;

		if(eq < 0) continue;
		//we now save all grasps in mTestedGrasps
		GraspPlanningState *newGrasp = new GraspPlanningState(mHand);
		newGrasp->setPositionType(SPACE_COMPLETE,false);
		newGrasp->setPostureType(POSE_DOF,false);
		//it is possible the object moved after dynamics, so we need to re-set the reference posture
		newGrasp->setRefTran(static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->getTran());
		newGrasp->saveCurrentHandState();
		newGrasp->setEpsilonQuality(eq);
		newGrasp->setVolume(vq);
		newGrasp->setIndex(i);
		//this is a hack; just gives us a field to save the dynamics code
		newGrasp->setDistance((double)dynCode);
		mTestedGrasps.push_back(newGrasp);
		//std::cout << "grasp tested, quality: " << eq << std::endl;
		if(!testedGraspList)
			continue;
		// record this to synthesize the output
		db_planner::Grasp *recordGrasp = static_cast<db_planner::Grasp*>( new GraspitDBGrasp(*tempGrasp) );
		tempState = new GraspPlanningState(static_cast<GraspitDBGrasp*>(recordGrasp)->getPreGraspPlanningState());
		tempState->copyFrom(newGrasp);
		static_cast<GraspitDBGrasp*>(recordGrasp)->setPreGraspPlanningState(tempState);
		testedGraspList->push_back(recordGrasp);

		delete tempGrasp;
	}
	if(mTestedGrasps.size() != 0)
		return true;
	return false;
}

/*! The grasps in graspList will be tested and their DBGrasp::mTestScores
    will be modified during the cross correlation.  The grasps will not be
	sorted based on any scheme.  It is the caller's responsibility to sort
	the results based on its application.
*/
void GraspitDBPlanner::crossCorrelate(const std::vector<db_planner::Model*> modelList,
							   std::vector<db_planner::Grasp*> graspList)
{
	db_planner::Model* objectArchive = mObject;
	DBGA("num of modelList: " << modelList.size() << " num of graspList: " << graspList.size());
	std::vector<db_planner::Grasp*> temp_graspList;
	transf transform;
	db_planner::Grasp * grasp;

	//temporarily remove the current object from the world	
	mHand->getWorld()->destroyElement(static_cast<GraspitDBModel*>(mObject)->getGraspableBody(), false);

	for(int i = 0; i < (int)modelList.size(); i ++){
		db_planner::Model* m = modelList[i];
		// load in the test model
		if (!static_cast<GraspitDBModel*>(m)->geometryLoaded()) {
			//this loads the actual geometry in the scene graph of the object
			static_cast<GraspitDBModel*>(m)->load(mHand->getWorld());
		}
		mObject = m;
		//this adds the object to the collision detection system
		static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->addToIvc();
		//todo: where do dynamic information come from?
		//static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->initDynamics();
		//this adds the object to the graspit world
		mHand->getWorld()->addBody(static_cast<GraspitDBModel*>(mObject)->getGraspableBody());
		//prepare the temporary grasp test list
		temp_graspList.clear();
		for(int k = 0; k < (int)graspList.size(); k ++){
			QString neighborModelName = QString(graspList[k]->SourceModel().ModelName().c_str());
			// copy original grasp for temporary use
			grasp = static_cast<db_planner::Grasp*> (new GraspitDBGrasp(*(static_cast<GraspitDBGrasp*>(graspList[k]))));
			temp_graspList.push_back(grasp);
		}
		testGrasps(STATIC, temp_graspList, NULL);
		//record the statistics
		for(int j = 0; j < (int)mTestedGrasps.size(); j ++){
			static_cast<GraspitDBGrasp*>(graspList[mTestedGrasps[j]->getIndex()])->addTestScore(mTestedGrasps[j]->getEpsilonQuality());
		}
		for(int k = 0; k <(int)graspList.size(); k ++){
			if(static_cast<GraspitDBGrasp*>(graspList[k])->getNumTestScores()==i) static_cast<GraspitDBGrasp*>(graspList[k])->addTestScore(0.0);
		}
		//release the memory used by the temporary test grasp list
		for(size_t i = 0; i < temp_graspList.size(); ++i){
			delete temp_graspList[i];
		}
		temp_graspList.clear();
		//remove this object from the scene and the collision detection system, but not delete it
		mHand->getWorld()->destroyElement(static_cast<GraspitDBModel*>(mObject)->getGraspableBody(), false);
	}
	//recover the world to the state right before cross correlation process
	mObject = objectArchive;
	//this adds the object to the collision detection system
	static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->addToIvc();
	//todo: where to dynamic information come from?
	//static_cast<GraspitDBModel*>(mObject)->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world
	mHand->getWorld()->addBody(static_cast<GraspitDBModel*>(mObject)->getGraspableBody());
}