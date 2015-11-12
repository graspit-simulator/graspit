#include "graspControllers.h"
#include "SensorInterface.h"
#include "body.h"
#include "puma560.h"
#include "world.h"
#include "matvec3D.h"
#include <QTextStream>
#include <QString>

World *GraspControllerInterface::myWorld;
bool GraspControllerInterface::controllerOn;
GraspState GraspControllerInterface::gs;
//QTextStream cout(stdout);
TapTapController::TapTapController(World *w, bool doSetState){
gs = NOTHING;
stateVector.push_back(APPROACHING);
stateVector.push_back(BACKING_OFF);
stateVector.push_back(TRENDING_LEFT);
stateVector.push_back(TRENDING_RIGHT);
stateVector.push_back(CENTER);
stateVector.push_back(CLOSING);
stateVector.push_back(ERR);
stateVector.push_back(ERR);
myWorld = w;
//For now, do a cheap start:
hand = myWorld->getHand(0);
setRobot(0);
target = myWorld->getGB(1);
//std::cout << "Robot name: " << puma->getName().toAscii() << " Hand name: " << hand->getName().toAscii() << " Target name: " << target->getName().toAscii() << endl;
setEndPose();
if (doSetState)
	this->setState(APPROACHING);
}

#define SENSORNUM 9
void TapTapController::groupForces(double *gf){
	for (int si = 0; si < SENSORNUM; ++si){
		SensorInterface * sint;
		SensorOutput * so;	
		sint = myWorld->getSensor(si);
		so = sint->Poll();
		gf[sint->getGroupNumber() - 1] += pow(pow(so->sensorReading[0],2) + pow(so->sensorReading[1],2) + pow(so->sensorReading[2],2), 0.5);
	}
	return;
}

transf finalLinkPose(Robot * robot)
{
Link * l = robot->getChain(0)->getLastLink();
return (l->getTran());
//std::cout<< "Link Name: " << l->getName().toAscii() << std::endl; 
}

void TapTapController::controlCommand(){
	double sr[3] = {0,0,0};
	groupForces(sr);
	switch(gs){
		case APPROACHING:{
			// set DOF controller to maintain wrist pose facing towards object
			//if contact between hand and object go to backing off
			if (sr[0] > .01)
				setState(BACKING_OFF);
			break;
		}
		case BACKING_OFF:{
			// set DOF controller to back off while facing towards object.			
			double dist = hand->getApproachDistance(target,50.0);
			if(dist > 25.0)
				setState(TRENDING_LEFT);
			break;
		}
		case TRENDING_LEFT:{
			if (sr[1] > .01){
				leftPose = finalLinkPose(puma);			
				setState(TRENDING_RIGHT);
			}
			break;					
		}
		case TRENDING_RIGHT:{
			if (sr[2] > .01){
				rightPose = finalLinkPose(puma);			
				setState(CENTER);
			if (sr[1] > 0.01)
				break;
			}
			break;					
							}
		case CENTER:{
				transf currPose = finalLinkPose(puma);
				vec3 errVec = (currPose.translation() - centerPose.translation());
				if(errVec.len() < 10)
					setState(CLOSING);
				break;
			}
		case CLOSING:{
	//		myWorld->turnOffDynamics();
			}
		default:{}
	}
}

GraspState TapTapController::getState(){
return gs;
}

bool TapTapController::setState(GraspState g){
	/*Check that transition is allowed.
	 *This model assumes one transition per node.
	 */
	if(stateVector[gs] != g ){
		gs = ERR;	
		return false;
	}
		else gs = g; // Allow the switch to occur.


	switch (gs){
	case APPROACHING:
		{
		std::vector<transf> traj;
		//If no target, robot, or hand has been set, fail
		if(target == NULL || puma == NULL || hand == NULL)
			{
			gs = ERR;
					
			return false;
			}
		//Otherwise, set target, do inverse kinematics, and set desired DOF 
		puma->generateCartesianTrajectory(*const_cast<transf*>(&startPose), *const_cast<transf*>(&endPose),traj, 0, 0,3);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case BACKING_OFF:
		{
		std::vector<transf> traj;
		//Generate a position that is away from the normal of the palm.
		
		transf backOffPose;
		const transf currPose = finalLinkPose(puma);
		backOffPose.set(currPose.rotation(), currPose.translation() - 250 * appVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&backOffPose),traj, 0, 0,.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case TRENDING_LEFT:{
		std::vector<transf> traj;
		vec3 leftVec, gravity(0.0,0.0,1.0);
		const transf currPose = finalLinkPose(puma);
		leftVec = appVec * gravity;
		leftPose.set(currPose.rotation(), currPose.translation() + 100*leftVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&leftPose),traj, 0, 0,.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	
	case TRENDING_RIGHT:{
		std::vector<transf> traj;
		vec3 rightVec, gravity(0.0,1.0,-1.0);
		const transf currPose = finalLinkPose(puma);
		rightVec = appVec * gravity;
		rightPose.set(currPose.rotation(), currPose.translation() + 200*rightVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&rightPose),traj, 0, 0,1.0);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case CENTER:{
		std::vector<transf> traj;
		const transf currPose = finalLinkPose(puma);
		centerPose.set(leftPose.rotation(), leftPose.translation() + (rightPose.translation() - leftPose.translation())/2);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&centerPose),traj, 0, 0,0.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case CLOSING:{
		//std::cout << "Entered closing" << endl;
		if (!hand->autoGrasp(true))
			//std::cout << "Autograsp failed!" << endl;
		break;
		}
	}
	  
	return true;

}

bool TapTapController::setTarget(){
	target = myWorld->getSelectedBody(0);
	if(target == NULL)
		return false;
	return true;
}

Body *TapTapController::getTarget(){
return target;
}

bool TapTapController::setHand(){
	hand = myWorld->getCurrentHand();
	if(hand == NULL)
		return false;
	return true;
}


Robot *TapTapController::getRobot(){
return puma;
}

bool TapTapController::setRobot(int n){
	puma = myWorld->getRobot(n);
	startPose = finalLinkPose(puma);
	if(puma == NULL)
		return false;
	return true;
}

bool TapTapController::setEndPose(){
	if (!(puma && target && hand))
		return false;
	vec3 diffVector = target->getTran().translation() - startPose.translation();
	diffVector.z()=0;
	diffVector /= diffVector.len();
	appVec = diffVector;
	endPose.set(startPose.rotation(), startPose.translation() + diffVector*350.0);
	return true;
}

TapTapDOFController::TapTapDOFController(World * w) : TapTapController(w, 0){
	dofActivation = new double[puma->getNumDOF()];
	setState(APPROACHING);
	speed = 1.0;
}

TapTapDOFController::~TapTapDOFController(){
	delete dofActivation;
}

bool TapTapDOFController:: applyArmForces(){
	//puma->setDOFActivation(dofActivation);
	puma->setDesiredDOFVals(dofActivation);
	return true;
}

void TapTapDOFController::controlCommand(){
	TapTapController::controlCommand();
	transf currPose = finalLinkPose(puma);
	//fixme getTimeStep needs to be actual time step not default time step
	vec3 tmpVelocityVector = desiredVelocityVector;
	tmpVelocityVector *= (speed*myWorld->getTimeStep());
	transf desiredPose(currPose.rotation(), currPose.translation() + tmpVelocityVector);
	puma->invKinematics(desiredPose, dofActivation, 0);
	applyArmForces();
}



bool TapTapDOFController::setState(GraspState g){
	/*Check that transition is allowed.
	 *This model assumes one transition per node.
	 */
	if(stateVector[gs] != g ){
		gs = ERR;	
		return false;
	}
		else gs = g; // Allow the switch to occur.


	switch (gs){
	case APPROACHING:
		{
		//If no target, robot, or hand has been set, fail
		if(target == NULL || puma == NULL || hand == NULL)
			{
			gs = ERR;
					
			return false;
			}
		//Otherwise, set target, do inverse kinematics, and set desired DOF 
		desiredVelocityVector = endPose.translation() - startPose.translation();
		desiredVelocityVector /= desiredVelocityVector.len();
		break;
		}
	case BACKING_OFF:
		{
		std::vector<transf> traj;
		//Generate a position that is away from the normal of the palm.
		
		transf backOffPose;
		const transf currPose = finalLinkPose(puma);
		backOffPose.set(currPose.rotation(), currPose.translation() - 250 * appVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&backOffPose),traj, 0, 0,.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case TRENDING_LEFT:{
		std::vector<transf> traj;
		vec3 leftVec, gravity(0.0,0.0,1.0);
		const transf currPose = finalLinkPose(puma);
		leftVec = appVec * gravity;
		leftPose.set(currPose.rotation(), currPose.translation() + 100*leftVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&leftPose),traj, 0, 0,.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	
	case TRENDING_RIGHT:{
		std::vector<transf> traj;
		vec3 rightVec, gravity(0.0,1.0,-1.0);
		const transf currPose = finalLinkPose(puma);
		rightVec = appVec * gravity;
		rightPose.set(currPose.rotation(), currPose.translation() + 200*rightVec);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&rightPose),traj, 0, 0,1.0);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case CENTER:{
		std::vector<transf> traj;
		const transf currPose = finalLinkPose(puma);
		centerPose.set(leftPose.rotation(), leftPose.translation() + (rightPose.translation() - leftPose.translation())/2);
		puma->generateCartesianTrajectory(currPose, *const_cast<transf*>(&centerPose),traj, 0, 0,0.5);
		puma->setChainEndTrajectory(traj,0);
		break;
		}
	case CLOSING:{
		//std::cout << "Entered closing" << endl;
		if (!hand->autoGrasp(true))
			//std::cout << "Autograsp failed!" << endl;
		break;
		}
	}
	  
	return true;

}