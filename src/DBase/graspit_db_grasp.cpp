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
// $Id: graspit_db_grasp.cpp,v 1.11 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBGrasp class
 */

#include "graspit_db_grasp.h"
#include <QSqlQuery>
#include <QtSql>
#include <list>
#include <QDir>
#include "model_utils.h"

#include "searchState.h"
#include "matvec3D.h"
#include "body.h"
#include "robot.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "debug.h"

//! Create a HandObjectState from data
bool initializeHandObjectState(const std::vector<double>& joints, const std::vector<double>& position, 
						   GraspPlanningState *state)
{
	if(joints.size() == 0) return false;
	state->setPostureType(POSE_DOF);
	state->getPosture()->readFromArray(const_cast<std::vector<double>&>(joints));
	state->setPositionType(SPACE_COMPLETE);
	state->getPosition()->readFromArray(const_cast<std::vector<double>&>(position));
	return true;
}

GraspitDBGrasp::~GraspitDBGrasp(){
	delete mPreGrasp;
	delete mFinalGrasp;
}

//! copy another grasp
GraspitDBGrasp::GraspitDBGrasp(const GraspitDBGrasp& grasp2) : db_planner::Grasp(grasp2){
	mPreGrasp = new GraspPlanningState(grasp2.mPreGrasp->getHand());
	mPreGrasp->copyFrom(grasp2.mPreGrasp);
	mFinalGrasp = new GraspPlanningState(grasp2.mFinalGrasp->getHand());
	mFinalGrasp->copyFrom(grasp2.mFinalGrasp);
	mTestScores = grasp2.mTestScores;
}

//! give the required data, fill in the necessary information of a grasp
bool GraspitDBGrasp::SetGraspParameters(const std::vector<double>& prejoint,
										const std::vector<double>& prepos,
										const std::vector<double>& finjoint,
										const std::vector<double>& finpos){
  //if(mHand is null, check if we there is a current world hand and if that world hand is the appropriate kind:
  if(!mHand){
    Hand * currentHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    if(!currentHand || getHandDBName(currentHand).compare(QString(getHandName().c_str())) ){
      //if not, attempt to load the hand with the appropriate name, and then set it to the current world hand
      mHand = loadHandFromDBName(QString(getHandName().c_str()));
      if (!mHand)
	return false;
    }      
    else
      mHand = currentHand;//if the current hand is the appropriate hand, set mHand to it -- WARNING this has some really funky implications for multithreading
    
  }

	mPreGrasp = new GraspPlanningState(mHand);
	initializeHandObjectState(prejoint, prepos, mPreGrasp);

	mFinalGrasp = new GraspPlanningState(mHand);
	initializeHandObjectState(finjoint, finpos, mFinalGrasp);
	return true;
}

//! transform this grasp by the transformation defined by array
bool GraspitDBGrasp::Transform(const float array[16]){

	transf originalPosition, alignedPosition, transform;

	// synthesize the transformation matrix
	mat3 m;
	m[0] = array[0];
	m[1] = array[1];
	m[2] = array[2];
	m[3] = array[4];
	m[4] = array[5];
	m[5] = array[6];
	m[6] = array[8];
	m[7] = array[9];
	m[8] = array[10];
	vec3 v;
	v[0] = array[3];
	v[1] = array[7];
	v[2] = array[11];
	transform.set(m,v);
	PositionState* ps = mPreGrasp->getPosition();
	ps->setTran(ps->getCoreTran()*transform);
	return true;
}

//! get the average test scores stored in this grasp
double GraspitDBGrasp::getTestAverageScore(){
	double sum = 0;
	for(int i = 0; i < (int)mTestScores.size(); i ++){
		sum += mTestScores[i];
	}
	if((int)mTestScores.size()>0){
		return sum/(double)mTestScores.size();
	}
	else{
		return 0;
	}
}

void GraspitDBGrasp::setPreGraspPlanningState(GraspPlanningState* p){
	delete mPreGrasp;
	mPreGrasp = p;

	const PositionState *positionS = p->readPosition();
	const PostureState *postureS = p->readPosture();

	std::vector<double> position, joints;
	for(int i = 0; i < positionS->getNumVariables(); ++i){
		position.push_back(positionS->getVariable(i)->getValue());
	}
	for(int i = 0; i < postureS->getNumVariables(); ++i){
		joints.push_back(postureS->getVariable(i)->getValue());
	}
	this->SetPregraspJoints(joints);
	this->SetPregraspPosition(position);
}

void GraspitDBGrasp::setFinalGraspPlanningState(GraspPlanningState* p){
	delete mFinalGrasp;
	mFinalGrasp = p;

	const PositionState *positionS = p->readPosition();
	const PostureState *postureS = p->readPosture();

	std::vector<double> pos, joints, contacts;
	for(int i = 0; i < positionS->getNumVariables(); ++i){
		pos.push_back(positionS->getVariable(i)->getValue());
	}
	for(int i = 0; i < postureS->getNumVariables(); ++i){
		joints.push_back(postureS->getVariable(i)->getValue());
	}
	std::list<position>* tmpContacts;
	tmpContacts = p->getContacts();
	for(std::list<position>::iterator it = tmpContacts->begin(); it!= tmpContacts->end(); ++it){
		contacts.push_back((*it).x());
		contacts.push_back((*it).y());
		contacts.push_back((*it).z());
	}
	this->SetFinalgraspJoints(joints);
	this->SetFinalgraspPosition(pos);
	this->SetContacts(contacts);
	//set the qualities
	this->SetEpsilonQuality(p->getEpsilonQuality());
	this->SetVolumeQuality(p->getVolume());
}

//! get the hand name that is in the CGDB
QString GraspitDBGrasp::getHandDBName(Hand* h)
{
	QString handName = h->getName();
	int material = h->getFinger(0)->getLink(0)->getMaterial();
	QString hand_db_name;
	if(handName == QString("Barrett")){
		if(material == h->getWorld()->getMaterialIdx("rubber")){
			hand_db_name = QString("BARRETT_RUBBER");
		} else if(material == graspItGUI->getIVmgr()->getWorld()->getMaterialIdx("plastic")){
			hand_db_name = QString("BARRETT_PLASTIC");
		} else if(material == h->getWorld()->getMaterialIdx("wood")){
			hand_db_name = QString("BARRETT_WOOD");
		}
	}else if(handName == QString("NewBarrett")){
		if(material == h->getWorld()->getMaterialIdx("rubber")){
			hand_db_name = QString("NEWBARRETT_RUBBER");
		} else if(material == graspItGUI->getIVmgr()->getWorld()->getMaterialIdx("plastic")){
			hand_db_name = QString("NEWBARRETT_PLASTIC");
		} else if(material == h->getWorld()->getMaterialIdx("wood")){
			hand_db_name = QString("NEWBARRETT_WOOD");
		}
	}else if(handName == QString("HumanHand20DOF")){
		hand_db_name = QString("HUMAN");
	}else if(handName == QString("pr2_gripper")){
		hand_db_name = QString("WILLOW_GRIPPER");
	}else if(handName == QString("McGrip") || handName == QString("McGrip_optim")){
		hand_db_name = QString("MC_GRIP");
	}else if (handName == QString("cobra_gripper")){
		hand_db_name = QString("COBRA_GRIPPER");
	}else if (handName == QString("tri_gripper_7dof")){
		hand_db_name = QString("TRI_GRIPPER_7DOF");
	}else {
		std::cout << "Wrong hand name detected: " << handName.latin1() << 
			".  Acceptable GRASPIT_hand_names are: Barrett, HumanHand20DOF, Pr2Gripper" << std::endl;
		hand_db_name = QString::null;
	}
	DBGP("Hand Name: " <<hand_db_name.toStdString());
	return hand_db_name;
}

bool GraspitDBGrasp::setHandMaterialFromDBName(Hand * h, const QString &hand_db_name){
	if(hand_db_name.contains("BARRETT")){
		//get the material index
		QStringList bstringlist = hand_db_name.split('_');
		//there must be a material after the '_' for this to be a valid name
		if (bstringlist.size() < 2) 
			return false;
		materialT mat = readMaterial(bstringlist[1].toLower().toAscii());
		if (mat == invalid)
			return false;
		RobotTools::setHandMaterial(h, mat);
		DBGA("Currently setting palm material to wood for all barret hands");
		h->getPalm()->setMaterial(readMaterial("wood"));
			return true;
	}
	//for all others, currently do nothing.
	return true;
}

Hand * GraspitDBGrasp::loadHandFromDBName(const QString &hand_db_name){
	QString handPath = getHandGraspitPath(hand_db_name);
	if(handPath ==QString::null)
		return NULL;
	handPath = QString(getenv("GRASPIT")) + handPath;
	if(!QDir().exists(handPath) && !modelUtils::repairHandGeometry()){
	  DBGA("Hand path not found");
	  return NULL;
	}
	DBGA("GraspitDBGrasp: loading hand from " << handPath.latin1());	      
	Hand *h = static_cast<Hand*>(graspItGUI->getIVmgr()->getWorld()->importRobot(handPath));
	if ( !h ) {
		DBGA("Failed to load hand");
	}
	else
		setHandMaterialFromDBName(h, hand_db_name);
	return h;
}

QString GraspitDBGrasp::getHandGraspitPath(const QString & handDBName)
{
	QString path;
	if (handDBName=="BARRETT_RUBBER") {
		path = "/models/robots/Barrett/Barrett.xml";
	} else if (handDBName=="BARRETT_PLASTIC") {
		path = "/models/robots/Barrett/Barrett.xml";
	} else if (handDBName=="BARRETT_WOOD") {
		path = "/models/robots/Barrett/Barrett.xml";
	} else if (handDBName=="NEWBARRETT_RUBBER") {
		path = "/models/robots/NewBarrett/NewBarrett.xml";
	} else if (handDBName=="BARRETT_HUMAN") {
		path = "/models/robots/HumanHand/HumanHand20DOF.xml";
	} else if (handDBName=="WILLOW_GRIPPER") {
		path = "/models/robots/pr2_gripper/pr2_gripper.xml";
	} else if (handDBName=="MC_GRIP") {
		path = "/models/robots/McHand/McGrip.xml";
	} else if (handDBName=="NEWBARRETT_RUBBER") {
		path = "/models/robots/NewBarrett/NewBarrett.xml";
	} else {
		DBGA("Cannot convert database hand name " << handDBName.latin1() << 
		     " to GraspIt path");
		path = QString::null;
	}
	return path;

}
