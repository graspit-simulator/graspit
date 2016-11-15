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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: robot.cpp,v 1.89 2010/07/09 02:33:44 cmatei Exp $
//
//######################################################################

/*! \file 
\brief Implements the robot class hierarchy
*/
#include <iomanip>
#include <QFile>
#include <QTextStream>

//needed just for the image of the Flock of Birds sensor and the approach direction
#include "SoArrow.h"
#include <Inventor/nodes/SoCube.h>

#include "bBox.h"
#include "mytools.h"
#include "matvecIO.h"
#include "robot.h"
#include "joint.h"
#include "dynJoint.h"
#include "world.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "dynamics.h"
#include "gloveInterface.h"
#include "eigenGrasp.h"
#include "matrix.h"
#include "tinyxml.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(MOVE_DOF);

const double Robot::AUTO_GRASP_TIME_STEP = 0.01;

/*! Removes the base and mountpiece from the world, and deletes the
kinematic chains and DOFs.  If this robot is connected to a parent
robot, it detaches itself.
*/
Robot::~Robot()
{
	for (int i=0;i<numChains;i++) {
		if (chainVec[i]) delete chainVec[i];
	}
	for (int i=0;i<numDOF;i++) {
		if (dofVec[i]) delete dofVec[i];
	}
	if (base) myWorld->destroyElement(base);  
	if (mountPiece) myWorld->destroyElement(mountPiece);
	if (parent) parent->detachRobot(this);
	if (mGloveInterface) delete mGloveInterface;
	if (mEigenGrasps) delete mEigenGrasps;

	std::cout << "Deleted robot: " << name() <<std::endl;
}

/*! Loads the robot information from an XML node, which is asumed
	to be the root of a hierarchy containing all the relevant
	information. The \a rootPath is considered the reference path
	for other files which are referenced in the XML. One exception
	are the link files which are assumed to be placed in 
	rootPath/iv
*/
int
Robot::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	QString robotName = root->Attribute("type");
	if(robotName.isNull()){
		QTWARNING("Robot Name undefined");
		return FAILURE;
	}
	const TiXmlElement* element = findXmlElement(root,"palm");
	QString valueStr;
	QString ivdir = rootPath + "iv/";
	// read and load the base; automatically placed at origin
	DBGA("Creating base...\n");  
	if(element){
		valueStr = element->GetText();	
		valueStr = valueStr.stripWhiteSpace();
		base = new Link(this,-1,-1,myWorld,(QString(name())+"Base").latin1());
		if (!base  || base->load(ivdir+valueStr)==FAILURE) {
			if (base) delete base; 
			base = NULL;
			DBGA("Failed to load base");
			return FAILURE;
		}
		base->addToIvc();

		//init my IVRoot and add the base
		IVRoot = new SoSeparator;
		IVRoot->addChild(base->getIVRoot());
	}
	else{
		QTWARNING("Base not found");
		return FAILURE;
	}
	std::list<const TiXmlElement*> elementList = findAllXmlElements(root, "dof");
	numDOF = countXmlElements(root, "dof");
	if (numDOF < 1) {
		DBGA("Wrong number of DOFs specified: " << numDOF);
		return FAILURE;
	}
	DBGA("Setting up " << numDOF << " degrees of freedom...");
	dofVec.resize(numDOF, NULL);

	//read each dof information
	std::list<const TiXmlElement*>::iterator p = elementList.begin();
	int f=0;
	while(p!=elementList.end()){
		valueStr = (*p)->Attribute("type");
		if(valueStr.isNull()){
			QTWARNING("DOF Type not found");
			return FAILURE;
		}
		QString dofType = valueStr.stripWhiteSpace();
		if(dofType == "r")
			dofVec[f] = new RigidDOF();
		else if(dofType == "b")
			dofVec[f] = new BreakAwayDOF();
		else if(dofType == "c")
			dofVec[f] = new CompliantDOF();
		else {
			DBGA("Unknown DOF Type requested: " << dofType.toStdString() << " for DOF " << f);
			return FAILURE;
		}
		dofVec[f]->dofNum = f;
		if (!dofVec[f]->readParametersFromXml(*p)) {
			DBGA("Failed to read DOF " << f );
			return FAILURE;
		}
		p++;
		f++;
	}

	//read number of chains and allocate them
	numChains = countXmlElements(root,"chain");
	if (numChains < 1) {
		DBGA("Wrong number of chains"); 
		return FAILURE;
	}
	DBGA("Creating " << numChains << " kinematic chains (fingers)...");
	chainVec.resize(numChains, NULL);

	//ask each chain to read itself from the file
	numJoints = 0;
	f = 0;
	elementList = findAllXmlElements(root, "chain");
	p = elementList.begin();
	while(p!=elementList.end()){
		fprintf(stderr, "Chain %d: ",f);
		chainVec[f] = new KinematicChain(this,f, numJoints);
		if (chainVec[f]->initChainFromXml((*p),ivdir)==FAILURE) {
			DBGA("Failed to read chain " << f);
			return FAILURE;
		}
		numJoints += chainVec[f]->getNumJoints();
		p++;	
		f++;
	}

	//set up DOFs before setting up EigenGrasps
	std::list<Joint *>jointList;
	for (int d=0; d<numDOF; d++) {
		jointList.clear();
		for (int f=0; f<numChains; f++) {
			for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
				if (chainVec[f]->getJoint(j)->getDOFNum() == d) {
					jointList.push_back(chainVec[f]->getJoint(j));
				}
			}
		}
		dofVec[d]->initDOF(this,jointList);
	}

	//reset the dynamics: fix the base and set desired dof's to current values
	getBase()->fix();
	for (int i=0; i<getNumDOF(); i++) {
		getDOF(i)->setDesiredPos( getDOF(i)->getVal() );
	}

	//optional information

	//load approach direction
	approachTran = transf::IDENTITY;
	element = findXmlElement(root,"approachDirection");
	if(element){
		const TiXmlElement* tmp = findXmlElement(element,"referenceLocation");
		if(!tmp){
			DBGA("Failed to read approach direction");	
			return FAILURE;
		}
		valueStr = tmp->GetText();
		valueStr = valueStr.simplifyWhiteSpace().stripWhiteSpace();
		QStringList l = QStringList::split(' ', valueStr);
		if(l.count()!=3){
			DBGA("Invalid approach direction input");
			return FAILURE;
		}
		bool ok1, ok2, ok3;
		float locX, locY, locZ;
		locX = l[0].toFloat(&ok1);
		locY = l[1].toFloat(&ok2);
		locZ = l[2].toFloat(&ok3);
		if(!ok1 || !ok2 || !ok3){
			DBGA("Invalid approach direction input");
			return FAILURE;
		}
		vec3 aPt = vec3(locX, locY, locZ);

		tmp = findXmlElement(element,"direction");
		if(!tmp){
			DBGA("Failed to read approach direction");	
			return FAILURE;
		}
		valueStr = tmp->GetText();
		valueStr = valueStr.simplifyWhiteSpace().stripWhiteSpace();
		l = QStringList::split(' ', valueStr);
		if(l.count()!=3){
			DBGA("Invalid approach direction input");
			return FAILURE;
		}
		locX = l[0].toFloat(&ok1);
		locY = l[1].toFloat(&ok2);
		locZ = l[2].toFloat(&ok3);
		if(!ok1 || !ok2 || !ok3){
			DBGA("Invalid approach direction input");
			return FAILURE;
		}
		vec3 zdir = vec3(locX, locY, locZ);
		zdir = normalise(zdir);

		vec3 xdir;
		if ( fabs(zdir % vec3(1,0,0)) > 0.9 ) xdir = vec3(0,1,0);
		else xdir = vec3(1,0,0);
		vec3 ydir = zdir * xdir;
		xdir = ydir * zdir;

		mat3 r(xdir, ydir, zdir);
		approachTran.set(r, aPt);
	}
	addApproachGeometry();

	//load EigenGrasps
	mEigenGrasps = new EigenGraspInterface(this);
	bool eigenLoaded = false;
	element = findXmlElement(root,"eigenGrasps");
	if(element) {
		valueStr = element->GetText();
		valueStr = valueStr.stripWhiteSpace();
		QString eigenFile = rootPath + valueStr;
		if (loadEigenData(eigenFile)==SUCCESS) {
			DBGA("Using eigengrasps from file: " << valueStr.latin1());
			eigenLoaded = true;
		}
	}
	if (!eigenLoaded) {
		mEigenGrasps->setTrivial();
		DBGA("Using identity eigengrasps");
	}

	//load pre-defined, "virtual" contact locations
	element = findXmlElement(root,"virtualContacts");
	if(element){
		valueStr = element->GetText();
		valueStr = valueStr.stripWhiteSpace();
		QString contactFile = rootPath + valueStr;
		if (loadContactData(contactFile)==SUCCESS) {
			DBGA("Loaded virtual contacts from file " << valueStr.latin1());
			showVirtualContacts(true);
		} else {
			DBGA("Failed to load virtual contacts from file " << valueStr.latin1());
		}
	}

	//load CyberGlove information
	mUseCyberGlove = false;
	element = findXmlElement(root,"cyberGlove");
	if(element){
		valueStr = element->GetText();
		valueStr = valueStr.stripWhiteSpace();
		mGloveInterface = new GloveInterface(this);
		QString calibFile = rootPath + valueStr;
		if (!mGloveInterface->loadCalibration(calibFile.latin1())) {
			DBGA("Failed to load glove calibration file " << calibFile.latin1());
			delete mGloveInterface; mGloveInterface = NULL;
			mUseCyberGlove = false;
		} else {
			DBGA("Cyberglove calibration loaded from " << calibFile.latin1());
			mUseCyberGlove = true;
		}
	}

	//load flock of birds information
	element = findXmlElement(root,"flockOfBirds");
	if(element){
		QString birdNumber = element->Attribute("number");
		if(birdNumber.isNull()){
			DBGA("There was a problem reading the Flock of Birds data in the configuration file");
			return FAILURE;
		}
		birdNumber = birdNumber.stripWhiteSpace();
		mBirdNumber = birdNumber.toInt();
		transf sensorTrans;
		const TiXmlElement* tmp = findXmlElement(element,"transform");
		if(!getTransform(tmp, sensorTrans)){
			DBGA("There was a problem reading the Flock of Birds data in the configuration file");
			return FAILURE;
		}		
		mFlockTran.identity();
		mFlockTran.setMount(sensorTrans);
		mUsesFlock = true;
		DBGA("Robot using Flock of Birds sensor " << mBirdNumber);
	}
	if (mUsesFlock) {
		addFlockSensorGeometry();
	}
	return SUCCESS;
}

/*! Loads the eigengrasp information from the file \a filename. */
int
Robot::loadEigenData(QString filename)
{
	if ( !mEigenGrasps->readFromFile(filename.latin1()) ) {
		DBGA("Unable to load eigenGrasp file " << filename.latin1());
		return FAILURE;
	}
	QString name = filename.section('/',-1,-1);
	name = name.section('.',0,0);
	mEigenGrasps->setName(name);
	return SUCCESS;
}

/*! Loads all the virtual contacts specified in the file \a filename
*/
int 
Robot::loadContactData(QString filename)
{
  FILE *fp = fopen(filename.latin1(), "r");
  if (!fp) {
    DBGA("Could not open contact file " << filename.latin1());
    return FAILURE;
  }
  
  char robotName[500];
  ; //yes, I know, can seg fault...
  if(fscanf(fp,"%s",robotName) <= 0)
    {
      DBGA("Robot::loadContactData - failed to read robot name");
      return 0;
    }
  
  int numContacts;
  if( fscanf(fp,"%d",&numContacts) <= 0){
    DBGA("Robot::loadContactData - failed to read number of contacts");
    return -1;
  }

  for (int i=0; i<numContacts; i++) {
    VirtualContact* newContact = new VirtualContact();
    newContact->readFromFile(fp);    
    int f = newContact->getFingerNum();
    int l = newContact->getLinkNum();
    if ( f >= 0) {
      if ( f>=getNumChains() || l>=getChain(f)->getNumLinks()) {
        DBGA("Virtual contact error, chain " << f << " link " << l << " does not exist");
        delete newContact;
        continue;
      }
      newContact->setBody( getChain(f)->getLink(l) );
      getChain(f)->getLink(l)->addVirtualContact( newContact );
    }
    else {
      newContact->setBody( getBase() );
      getBase()->addVirtualContact(newContact);
    }
    newContact->computeWrenches(false,false);
  }
  fclose(fp);
  return SUCCESS;
}

/*! This robot becomes a "clone" of another robot, specified in \a original.
This means that the new robot has its own kinematic structure, DOF's, etc
but its links share the geometry of the original robot. See the clone 
function in the Body class for details on how that works.

This is generally used for multi-threading, if you want to do spread the 
computations done on a robot to multiple cores. You then create multiple
clones of your robot and pass one to each thread. See the threading 
documentation in the collision detection classes for details.
*/
void
Robot::cloneFrom(Robot *original)
{
	myName = original->getName() + QString(" clone");

	//new robots have contact indicators disabled by default
	if (base) delete base;
	base = new Link(this,-1,-1,myWorld,(QString(name())+"Base").latin1());
	base->cloneFrom( original->getBase() );
	//base->initDynamics();
	IVRoot = new SoSeparator;
	IVRoot->addChild(base->getIVRoot());

	numDOF = original->getNumDOF();
	dofVec.resize(numDOF, NULL);
	for (int f=0; f<numDOF; f++) {
		switch(original->getDOF(f)->getType()) {
			case DOF::RIGID:
				dofVec[f] = new RigidDOF( (RigidDOF*)(original->getDOF(f)) );
				break;
			case DOF::BREAKAWAY:
				dofVec[f] = new BreakAwayDOF( (BreakAwayDOF*)(original->getDOF(f)) );
				break;
			case DOF::COMPLIANT:
				dofVec[f] = new CompliantDOF( (CompliantDOF*)(original->getDOF(f)) );
				break;
			default:
				DBGA("ERROR: Unknown DOF type in original!");
				assert(0);
		}
	}

	numChains = original->getNumChains();
	chainVec.resize(numChains, NULL);
	numJoints = 0;
	for (int f=0; f<numChains; f++) {
		chainVec[f] = new KinematicChain(this,f,numJoints);
		chainVec[f]->cloneFrom( original->getChain(f) );
		numJoints += chainVec[f]->getNumJoints();
	}
	assert (numJoints == original->getNumJoints() );

	std::list<Joint *>jointList;
	for (int d=0; d<numDOF; d++) {
		jointList.clear();
		for (int f=0; f<numChains; f++) {
			for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
				if (chainVec[f]->getJoint(j)->getDOFNum() == d) {
					jointList.push_back(chainVec[f]->getJoint(j));
				}
			}
		}
		dofVec[d]->initDOF(this,jointList);
	}

	base->setTran(transf::IDENTITY);
	//careful: clones robots have never been tested with the dynamics engine
	getBase()->fix();
	for (int i=0; i<getNumDOF(); i++) {
		getDOF(i)->setDesiredPos( getDOF(i)->getVal() );
	}

	setRenderGeometry( original->getRenderGeometry() );

	//check if original has an approach tran
	approachTran = original->getApproachTran();
	addApproachGeometry();

	//clone eigengrasp interface
	mEigenGrasps = new EigenGraspInterface(original->getEigenGrasps());

	//no glove interface
	mGloveInterface = NULL;
	this->mUseCyberGlove = false;
}

/*! Sets the transparency of all the links that make up this robot,
as well as the base
*/
void
Robot::setTransparency(float t)
{
	base->setTransparency(t);
	for (int i=0; i<getNumChains(); i++) {
		for (int l=0; l<getChain(i)->getNumLinks(); l++) {
			getChain(i)->getLink(l)->setTransparency(t);
		}
	}
}

/*! Sets the name \a newName for the robot, as well as derived names of the form
newName_chain#_link# for the links and newName_base for the base.
*/
void
Robot::setName(QString newName)
{
	WorldElement::setName(newName);
	for (int c=0; c<getNumChains(); c++) {
		for (int l=0; l<getChain(c)->getNumLinks(); l++){
			getChain(c)->getLink(l)->setName( newName + QString("_chain%1_link%2").arg(c).arg(l) );
		}
	}
	if (base) base->setName(newName + QString("_base"));
}

/*! Adds an arrow that shows the pre-defined approach direction for this
robot. The arrow is rooted at the origin of the approach direction and
points in the z direction of mApproachTran
*/
void
Robot::addApproachGeometry()
{	
	IVApproachRoot = new SoSeparator();
	
	SoTransform *t1 = new SoTransform();
	approachTran.toSoTransform(t1);
	IVApproachRoot->addChild(t1);

	SoArrow *arrow = new SoArrow;
	arrow->height = 14;
	arrow->cylRadius = (float)1.25;
	arrow->coneRadius = (float)2.6;
	arrow->coneHeight = (float)5.5;
	SoTransform *arrowTran = new SoTransform();
	arrowTran->rotation.setValue(SbVec3f(1,0,0),(float)(M_PI/2.0));
	IVApproachRoot->addChild(arrowTran);
	IVApproachRoot->addChild(arrow);

	getBase()->getIVRoot()->addChild(IVApproachRoot);	
}

/*! Adds a visual marker that shows where on the robot the Flock of Birds
sensor is mounted. Uses the mounting information stored in the
mFlockTran, which is usually pre-defined in the robot file
*/
void
Robot::addFlockSensorGeometry()
{
	IVFlockRoot = new SoSeparator();
	SoTransform *t = new SoTransform();
	mFlockTran.getMount().toSoTransform( t );
	IVFlockRoot->addChild(t);
	SoCube *cube = new SoCube();
	cube->width = 30;
	cube->height = 20;
	cube->depth = 4;
	IVFlockRoot->addChild(cube);
	SoCube *smallCube = new SoCube();
	smallCube->width = 10;
	smallCube->height = 20;
	smallCube->depth = 6;
	SoTransform *t2 = new SoTransform();
	t2->translation.setValue(10,0,-5);
	IVFlockRoot->addChild(t2);
	IVFlockRoot->addChild(smallCube);
	getBase()->getIVRoot()->addChild(IVFlockRoot);
}

/*! Sets the robot to use the trivial eigengrasps set, where each eigengrasp
corresponds to a single DOF with an amplitude of 1.0. In this case, there
is no difference between seting EG's and using DOF's directly.
*/
int
Robot::useIdentityEigenData()
{
	if (!mEigenGrasps->setTrivial()) {
		QTWARNING("Error setting Identity EigenGrasps");
		return FAILURE;
	}
	return SUCCESS;
}

/*! Tells the robot which CyberGlove the raw sensor data is coming from. The robot
does not process raw data directly, just passes it to the mGloveInterface which
hodsl calibration and DOF mapping between a raw Glove and this particular robot
*/
void Robot::setGlove(CyberGlove *glove)
{
	if (!mGloveInterface) {
		mUseCyberGlove = false;
		return;
	}
	mGloveInterface->setGlove(glove);
}

/*! Sets the values of the DOF's based on the information from the mGloveInterface,
which has presumably processed a new batch of raw information from a
real CyberGlove
*/
void
Robot::processCyberGlove()
{
	int i;
	double *desiredVals = new double[getNumDOF()];
	for (i=0; i<getNumDOF(); i++) {
		if ( mGloveInterface->isDOFControlled(i) ) {
			desiredVals[i] = mGloveInterface->getDOFValue(i);
			if ( desiredVals[i] < getDOF(i)->getMin() ) desiredVals[i] = getDOF(i)->getMin();
			if ( desiredVals[i] > getDOF(i)->getMax() ) desiredVals[i] = getDOF(i)->getMax();
		}
		else {
			desiredVals[i] = getDOF(i)->getVal();
		}
	}
	moveDOFToContacts(desiredVals, NULL, false);
	emitConfigChange();
	delete [] desiredVals;
}

/*!
Given the body filename of a mountpiece, this will load it into the
world and connect it to the base of this robot.
*/
Link *
Robot::importMountPiece(QString filename)
{
	QString mountName = QString(name())+"_mount";
	mountPiece = new Link(this,-1,-1,myWorld,mountName.latin1());
	if (mountPiece->load(filename)==FAILURE){
		delete mountPiece; mountPiece = NULL; return NULL;
	}
	mountPiece->addToIvc();
	IVRoot->addChild(mountPiece->getIVRoot());
	mountPiece->setTran(base->getTran());
	base->setDynJoint(new FixedDynJoint(mountPiece,base));
	return mountPiece;
}

/*! Adds all the bodies associated with this robot (links, base, mountpiece,
attached robots) to the given vector of bodies.
*/
void 
Robot::getBodyList(std::vector<Body*> *bodies)
{
	if (base) bodies->push_back(base);
	if (mountPiece) bodies->push_back(mountPiece);
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<chainVec[c]->getNumLinks(); l++) {
			bodies->push_back(chainVec[c]->getLink(l));
		}
	}
	for (int c=0; c<numChains; c++) {
		for (int i=0; i<chainVec[c]->getNumAttachedRobots(); i++) {
			chainVec[c]->getAttachedRobot(i)->getBodyList(bodies);
		}
	}
}

/*!
Returns a vector of all links associated with this robot including
all links in all chains, the base, and the mountpiece.
*/
void
Robot::getAllLinks(std::vector<DynamicBody *> &allLinkVec)
{
	int c,l,i;
	if (base) allLinkVec.push_back(base);
	if (mountPiece) allLinkVec.push_back(mountPiece);

	for (c=0;c<numChains;c++)
		for (l=0;l<chainVec[c]->getNumLinks();l++)
			allLinkVec.push_back(chainVec[c]->getLink(l));
	for (c=0;c<numChains;c++)
		for (i=0;i<chainVec[c]->getNumAttachedRobots();i++)
			chainVec[c]->getAttachedRobot(i)->getAllLinks(allLinkVec);
}

/*!
Returns a pointer to this robot and recursively, all child robots
connected to this one
*/
void
Robot::getAllAttachedRobots(std::vector<Robot *> &robotVec)
{
	robotVec.push_back(this);
	for (int c=0;c<numChains;c++) {
		for (int i=0;i<chainVec[c]->getNumAttachedRobots();i++) {
			chainVec[c]->getAttachedRobot(i)->getAllAttachedRobots(robotVec);
		}
	}
}


/*!
  Given a pointer to a robot, the number of the chain on this robot to
  connect the new robot to, and the offset transform between the chain end
  and the base of the new robot, this will attach the new robot to this
  robot's chain end.

  Also disables collision between the link of this robot that the other
  robot is attached to and the base of the other robot as well as the mount
  piece.
*/
void
Robot::attachRobot(Robot *r,int chainNum,const transf &offsetTr)
{
  r->parent = this;
  r->parentChainNum = chainNum;
  r->tranToParentEnd = offsetTr.inverse();
  chainVec[chainNum]->attachRobot(r,offsetTr);

  Link *lastLink = chainVec[chainNum]->getLink(chainVec[chainNum]->getNumLinks()-1);
  myWorld->toggleCollisions(false, lastLink, r->getBase());
  if (r->getMountPiece()) {
	  myWorld->toggleCollisions(false, lastLink, r->getMountPiece());
  }
}


/*!
The detaches the robot pointed to by \a r from this robot so that they may
move independently.
*/
void
Robot::detachRobot(Robot *r)
{
	DBGA("Detaching Robot " << r->getName().latin1() << " from " << getName().latin1());
	r->parent = NULL;
	chainVec[r->parentChainNum]->detachRobot(r);
}


/*!
Given an array of DOF values equal in length to the number of DOFs in
this robot, and the chain number for which the  kinematics should be
computed, this will return a vector of transforms corresponding to
the pose of each link in the chain.
*/
void
Robot::fwdKinematics(double* dofVals,std::vector<transf>& trVec,int chainNum)
{
	double *jointVals = new double[numJoints];
	getJointValues(jointVals);
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->accumulateMove(dofVals[d], jointVals, NULL);
	}
	chainVec[chainNum]->fwdKinematics(jointVals,trVec);
	delete [] jointVals;
}

/*! Given a transform for the end pose of a particular kinematic chain,
	this will compute the DOF values corresponding to that pose.
	It will use an iterative approximation technique. In this method,
	the jacobian is based on each dofs rather than the joint angles
*/
int
Robot::invKinematics(const transf& targetPos, double* dofVals, int chainNum)
{
	//upperBound constrains the change of the joint values in radian
	//epsilon is the upper bound for a joint delta value in radian to check the convergence
	double factor = 0, epsilon = 0.0001, upperBound = 0.2;
	const int safeGuardUpperBound = 200;
	Matrix deltaPR(6,1); // the difference of the position and the orientation
	deltaPR.setAllElements(1.0);
	Matrix deltaDOF(numDOF, 1); // the delta of dof value in this robot
	std::vector<double> currentDOFVals(numDOF);

	for(int i = 0; i < numDOF; ++i){
		//get the dof vals for inverse kinematics computation
		currentDOFVals[i] = dofVec[i]->getVal();
		//get the dof vals, for later comparison and decision
		dofVals[i] = dofVec[i]->getVal();
	}

	std::vector<transf> fwdK;
	fwdK.resize(chainVec[chainNum]->getNumLinks(), transf::IDENTITY);
	transf currentPos;

	// forwardKinematics only calculates the forward kinematics with respect to the base
	// of the robot, did not consider the transformation from world origin to the base
	
	fwdKinematics(&currentDOFVals[0], fwdK, chainNum);
	currentPos = fwdK[chainVec[chainNum]->getNumLinks() - 1];

	DBGP("from: " << currentPos.translation().x() << " " <<
		currentPos.translation().y() << " " <<
		currentPos.translation().z() << " " <<
		currentPos.rotation().w << " " <<
		currentPos.rotation().x << " " <<
		currentPos.rotation().y << " " <<
		currentPos.rotation().z);

	DBGP("to: " << targetPos.translation().x() << " " <<
		targetPos.translation().y() << " " <<
		targetPos.translation().z() << " " <<
		targetPos.rotation().w << " " <<
		targetPos.rotation().x << " " <<
		targetPos.rotation().y << " " <<
		targetPos.rotation().z);

	int safeguard = 0;
	while(++safeguard < safeGuardUpperBound){
		/* forwardKinematics calculates the forward kinematics with respect to the world
		*/
		fwdKinematics(&currentDOFVals[0], fwdK, chainNum);
		currentPos = fwdK[chainVec[chainNum]->getNumLinks() - 1];
		//compute the deltaPR, the difference of the position and orientation
		vec3 dtLocation, dtOrientation;
		dtLocation = targetPos.translation() - currentPos.translation();

		vec3 v1, v2;
		mat3 m1, m2;
		currentPos.rotation().ToRotationMatrix(m1);
		targetPos.rotation().ToRotationMatrix(m2);

		v1 = vec3(m1.element(0,0), m1.element(0,1), m1.element(0,2));
		v2 = vec3(m2.element(0,0), m2.element(0,1), m2.element(0,2));
		dtOrientation = v1 * v2;
		v1 = vec3(m1.element(1,0), m1.element(1,1), m1.element(1,2));
		v2 = vec3(m2.element(1,0), m2.element(1,1), m2.element(1,2));
		dtOrientation += v1 * v2;
		v1 = vec3(m1.element(2,0), m1.element(2,1), m1.element(2,2));
		v2 = vec3(m2.element(2,0), m2.element(2,1), m2.element(2,2));
		dtOrientation += v1 * v2;
		dtOrientation *= 0.5;

		deltaPR.elem(0,0) = dtLocation[0];
		deltaPR.elem(1,0) = dtLocation[1];
		deltaPR.elem(2,0) = dtLocation[2];
		deltaPR.elem(3,0) = dtOrientation[0];
		deltaPR.elem(4,0) = dtOrientation[1];
		deltaPR.elem(5,0) = dtOrientation[2];

		Matrix m = chainVec[chainNum]->actuatedJacobian(chainVec[chainNum]->linkJacobian(true));
		Matrix jointJacobian = m.getSubMatrix(m.rows() - 6, 0, 6, chainVec[chainNum]->getNumJoints());
		Matrix jointToDOFJacobian = getJacobianJointToDOF(chainNum);
		Matrix DOFJacobian = Matrix(jointJacobian.rows(), jointToDOFJacobian.cols());
		matrixMultiply(jointJacobian, jointToDOFJacobian, DOFJacobian);

/*		for(int i = 0; i < DOFJacobian.rows(); ++i){
			for(int j = 0; j < DOFJacobian.cols(); ++j){
				DBGA(DOFJacobian.elem(i,j) << " ");
			}
			std::cout << std::endl;
		}

		for(int i = 0; i < deltaPR.rows(); ++i){
			for(int j = 0; j < deltaPR.cols(); ++j){
				DBGA(deltaPR.elem(i,j) << " ");
			}
			DBGA("");
		}
*/
		//underDeterminedSolveMPInv(DOFJacobian, deltaPR, deltaDOF);
		underDeterminedSolveQR(DOFJacobian, deltaPR, deltaDOF);
/*		std::cout << "results: \n";
		for(int i = 0; i < deltaDOF.rows(); ++i){
			for(int j = 0; j < deltaDOF.cols(); ++j){
				std::cout << deltaDOF.elem(i,j) << " ";
			}
			std::cout << std::endl;
		}
*/
		//apply the change to the robot and continue
		for(int i = 0; i < numDOF; ++i){
			//modify the dof's
			currentDOFVals[i] += deltaDOF.elem(i,0);
			//rounded the dof's within 360 degree
			currentDOFVals[i] -= (int)(currentDOFVals[i]/(2*M_PI)) * 2 * M_PI;
		}

		//check the maximum of the joint change
		factor = -1.0;
		for(int i = 0; i < deltaDOF.cols(); ++i){
			factor = factor > fabs(deltaDOF.elem(i,0)) ? factor : fabs(deltaDOF.elem(i,0));
		}

		//check the convergence
		if(factor < epsilon){
			for(int k = 0; k < numDOF; ++k){
				//std::cout << "from: " << dofVals[k] << " to: " << currentJointVals[k];
				//check if the joint value jumps to another one which is too far away of the previous one
				if(fabs(dofVals[k] - currentDOFVals[k]) > upperBound){
					std::cout << "exceeds the upper bound at DOF: " << k << " , jumping to another pose\n";
					return FAILURE;
				}
				dofVals[k] = currentDOFVals[k];
			}
			break;
		}
	}
	//check that all the joints are within the legal range
	for(int i = 0; i < numChains; ++i){
		for(int j = 0; j < chainVec[i]->getNumJoints(); ++j){
			double jnt = dofVals[chainVec[i]->getJoint(j)->getDOFNum()] *
				chainVec[i]->getJoint(j)->getCouplingRatio();
			if(jnt > chainVec[i]->getJoint(j)->getMax() ||
				jnt < chainVec[i]->getJoint(j)->getMin()){
					std::cout << "inverse kinematics in invalid joint value: " << i << "th chain, " << j << "th joint\n";
					return FAILURE;
				}
		}
	}

	if(safeguard >= safeGuardUpperBound){
		std::cout << "safeguard hit\n";
		return FAILURE;
	}
	/*
	chainVec[chainNum]->fwdKinematics(&currentJointVals[0], fwdK);
	currentPos = fwdK[chainVec[chainNum]->getNumLinks() - 1];
	std::cout << "in: " << currentPos.translation().x() << " " <<
	currentPos.translation().y() << " " <<
	currentPos.translation().z() << " " <<
	currentPos.rotation().w << " " <<
	currentPos.rotation().x << " " <<
	currentPos.rotation().y << " " <<
	currentPos.rotation().z << " " << "\n";
	*/
	return SUCCESS;
}

/*!
This is an internal method called by contactsPreventMotion.  Given a
motion transform for an entire robot that is defined with respect to the
base frame of the robot, it recursively checks all of the
kinematic chains of the robot and any robots connected to them to
determine if any contacts will prevent the motion.
*/
bool
Robot::simpleContactsPreventMotion(const transf& motion) const
{
	transf linkMotion;
	transf baseToLink;

	for (int i=0;i<numChains;i++) {
		for (int j=0; j<chainVec[i]->getNumLinks();j++) {
			if (chainVec[i]->getLink(j)->getNumContacts()) {
				baseToLink = chainVec[i]->getLink(j)->getTran() * base->getTran().inverse();
				linkMotion = baseToLink * motion * baseToLink.inverse();
				if (chainVec[i]->getLink(j)->externalContactsPreventMotion(linkMotion))
					return true;
			}
		}
		// recursively check the motion for any robots attached to this chain
		for (int j=0;j<chainVec[i]->getNumAttachedRobots();j++) {
			baseToLink = chainVec[i]->getAttachedRobot(j)->getBase()->getTran() * base->getTran().inverse();
			linkMotion = baseToLink * motion * baseToLink.inverse();
			if (chainVec[i]->getAttachedRobot(j)->simpleContactsPreventMotion(linkMotion))
				return true;
		}
	}
	return false;
}


/*!
Examines all of the contacts on every link of this robot, and each
child robot connected to this one, to determine if the desired motion
can be performed.  It also performs the inverse kinematics of the
parent robot to see if any of those link motions will be prevented by
contacts on those links.  The motion is expressed with respect to the
base frame of the robot.  This is only used when dynamics are off.
*/
bool
Robot::contactsPreventMotion(const transf& motion) const
{
	int l;
	transf linkMotion;

	// check if we can move the base link
	if (base->externalContactsPreventMotion(motion)) return true;
	if (mountPiece && mountPiece->contactsPreventMotion(motion)) return true;

	// check if we can move all the kinematic chains and the robots connected to them
	if (simpleContactsPreventMotion(motion)) return true;

	// if this robot is connected to another before it, try to do inv kinematics
	// and check if contacts on any of those links will prevent this motion
	if (parent) {
		transf newTran = tranToParentEnd*motion*base->getTran();// * parent->getBase()->getTran().inverse();
		KinematicChain *chain = parent->getChain(parentChainNum);
		std::vector<transf> parentTrVec(chain->getNumLinks());    
		double *dofVals = new double[parent->getNumDOF()];

		if (parent->invKinematics(newTran,dofVals,parentChainNum)==FAILURE){
			delete [] dofVals;
			return true;
		}

		parent->fwdKinematics(dofVals,parentTrVec,parentChainNum);
		delete [] dofVals;

		for (l=0;l<chain->getNumLinks();l++) {      
			if (chain->getLink(l)->getNumContacts()) {
				linkMotion = parentTrVec[l] * chain->getLink(l)->getTran().inverse();
				if (chain->getLink(l)->contactsPreventMotion(linkMotion)) {
					return true;
				}
			}
		} 
	}
	return false;
}

/*! Breaks all the contacts formed on the links or the base of this robot */
void
Robot::breakContacts()
{
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			getChain(f)->getLink(l)->breakContacts();
		}
	}
	base->breakContacts();
	if (mountPiece) mountPiece->breakContacts();
}


/*! This can be used to disable / enable the automatic render requests from this robot. 
If this is disabled, changes to the robot pose or posture should not trigger an
automatic render request through Coin. The robot however is still part of the scene 
graph, so whenever a render request comes from someplace else, the robot is rendered
in its most current state. See also the option in the World class to completely
remove a robot from the scene graph.
*/
void
Robot::setRenderGeometry(bool s)
{
	mRenderGeometry = s;
	if (parent) parent->setRenderGeometry(s);
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<chainVec[c]->getNumAttachedRobots(); j++) {
			chainVec[c]->getAttachedRobot(j)->setRenderGeometry(s);
		}
	}
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			getChain(f)->getLink(l)->setRenderGeometry(s);
		}
	}
	base->setRenderGeometry(s);
	if (mountPiece) mountPiece->setRenderGeometry(s);
}

/*! This attempt to set the pose of the robot base to tr.  It does not check
	for collisions, but it will check the inverse kinematics of a parent
	robot (if it exists) to determine whether the move is valid.  If it is
	valid the DOF's of the parent are set, and this robot and all of its
	children are moved.
*/
int
Robot::setTran(const transf& tr)
{
  if (parent) {
    double *dofVals = new double[parent->getNumDOF()];
    transf parentBaseToEnd = tranToParentEnd*tr;
	if (parent->invKinematics(parentBaseToEnd,dofVals,parentChainNum)==FAILURE) {
	  delete [] dofVals;
      return FAILURE;
    }
    parent->forceDOFVals(dofVals);
	delete [] dofVals;
  } else {
    simpleSetTran(tr);  
  }
  return SUCCESS;
}

/*! Returns the total number of links of this robot, including the palm and
the mount piece (if any).
*/
int
Robot::getNumLinks() const
{
	int links=0;
	for (int k=0; k<getNumChains(); k++) {
		links += chainVec[k]->getNumLinks();
	}
	links++; //palm
	if (mountPiece) links++;
	return links;
}

/*! Returns the total number of contacts on this robot, including all links
	and the palm. If \a body is NULL it returns the total number of contacts
	on this robot regardless of who they are against.
*/
int
Robot::getNumContacts(Body *body)
{
	int c = getBase()->getNumContacts(body);
	for (int f=0; f<getNumChains(); f++) {
		c += getChain(f)->getNumContacts(body);
	}
	return c;
}

/*! Returns the list of contacts on this robot against the Body \a body, including 
	contacts on all robot links and the palm. If \a body is NULL it returns all the
	contacts on this robot regardless of who they are against.
*/
std::list<Contact*> 
Robot::getContacts(Body* body)
{
	std::list<Contact*> contacts;
	std::list<Contact*> chainContacts = getBase()->getContacts(body);
	contacts.insert(contacts.end(),chainContacts.begin(), chainContacts.end());
	for (int f=0; f<getNumChains(); f++) {
		chainContacts = getChain(f)->getContacts(body);
		contacts.insert(contacts.end(),chainContacts.begin(), chainContacts.end());
	}
	return contacts;
}

/*! Returns the total number of virtual contacts on this robot, including all 
links and the palm.
*/
int
Robot::getNumVirtualContacts()
{
	int c = getBase()->getNumVirtualContacts();
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			c += getChain(f)->getLink(l)->getNumVirtualContacts();
		}
	}
	return c;
}

/*! Shows or hides the virtual contact on this robot (which can be shown
as thin red cylinders). They are usually hidden so they don't trigger
a redraw when not wanted, although we now have a better mechanism 
for that using setRenderGeometry on the entire robot 
*/
void
Robot::showVirtualContacts(bool on)
{
	int s; bool b;
	if (on) s = 1;
	else s = 2;
	b = getBase()->frictionConesShown();
	getBase()->showFrictionCones(b, s);
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			b = getChain(f)->getLink(l)->frictionConesShown();
			getChain(f)->getLink(l)->showFrictionCones( b, s );
		}
	}
}

//----------------------------------save and load state-------------------------

/*! Saves the current state of the robot, which can be restored later. 
Overwrites any previously saved state. Maybe at some point we will 
unify this with the stack of dynamic states which can hold any number
of states for a body.
*/
void 
Robot::saveState()
{
	savedTran = getTran();
	savedDOF.clear();
	QTextStream stream(&savedDOF,QIODevice::ReadWrite);
	writeDOFVals(stream);
	savedState = true;
}

/*! Restores the previously saved state of this robot. If no state has been
saved since the last restore, is prints out a warning, but does not
die in pain.
*/
void
Robot::restoreState()
{
	if (!savedState) {
		DBGA("Warning: hand state not saved!");
		if (savedDOF.size()==0) {
			DBGA("Saved DOF data absent; can not restore");
			return;
		}
	}
	savedState = false;
	setTran( savedTran );
	QTextStream stream(&savedDOF,QIODevice::ReadOnly);
	readDOFVals(stream);
}

/*! Reads DOF values from a text stream (usually from a saved world file or an 
internally saved state) and sets them
*/
QTextStream&
Robot::readDOFVals(QTextStream &is)
{
	for (int d=0; d<numDOF; d++) {
		if (!dofVec[d]->readFromStream(is)) {
			DBGA("Failed to read DOF " << d << " from stream");
			return is;
		}
	}

	// This is a somewhat contorted way of doing things, but this is the only mechanism available
	// for actually moving the DOFs 
	double *jointVals = new double[numJoints];
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->getJointValues(jointVals);
	}
	setJointValuesAndUpdate(jointVals);
	delete [] jointVals;
	return is;
}

/*! Writes DOF values to a text stream (usually a world save file or the 
internally saved state)
*/
QTextStream&
Robot::writeDOFVals(QTextStream &os)
{
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->writeToStream(os);
		os << " ";
	}
	return os;
}

//---------------------------------- STATIC MOVEMENT FUNCTIONS--------

/*! Asks the chains to set the new joint values in \a jointVals. It is 
	expected that this vector is of size equal to the number of joints of 
	this robot. Does NOT ask the chains to also update the poses of the links.
*/
void Robot::setJointValues(const double* jointVals)
{
	for (int c=0; c<numChains; c++) {
		chainVec[c]->setJointValues(jointVals);
	}
}

/*! Asks the chains to set the new joint values in \a jointVals. It is 
	expected that this vector is of size equal to the number of joints 
	of this robot. Also asks the chains to also update the poses of the 
	links.
*/
void Robot::setJointValuesAndUpdate(const double* jointVals)
{
	for (int c=0; c<numChains; c++) {
		chainVec[c]->setJointValues(jointVals);
		chainVec[c]->updateLinkPoses();
	}
}


/*! This is one of the core function used for moving the DOF's of a robot in statics.
Its purpose is to solve a collision: given an initial set of joint values, which
should be collision-free, and a set of final joint values that result in a collision,
it will interpolate between the two to find the initial moment of contact (where 
the bodies are not inter-penetrating, but are separated by less then the contat
threshold.

In order to be efficient, it only checks the bodies that are given in the 
\a colReport. If other bodies are also colliding at any point during the interpolation,
this function will never know about it.

Returns 0 if the interpolation fails (usually because the starting configuration
is also in collision, or 1 if its succeeds. In case of success, \a
interpolationTime will hold the interpolation coefficient that resulted in the
original contact.
*/	
int
Robot::interpolateJoints(double *initialVals, double *finalVals, 
						 CollisionReport *colReport, double *interpolationTime)
{
	double *currentVals = new double[ getNumJoints() ];
	double minDist,t = 0.0,deltat = 1.0;
	while (deltat > 1.0e-20 && t >= 0) {

		DBGP("DOF joint interpolation cycle")
			deltat *= 0.5;
		for (int j=0; j<getNumJoints(); j++) {
			currentVals[j] = t*finalVals[j] + (1.0-t)*initialVals[j];
		}
		setJointValuesAndUpdate(currentVals);

		minDist = 1.0e10;
		for (int i=0; i<(int)colReport->size(); i++) {
			minDist = MIN(minDist,myWorld->getDist( (*colReport)[i].first, (*colReport)[i].second));
			if ( minDist <= 0 ) {
				t -= deltat;
				break;
			}
		}
		if (minDist > 0) {
			if (minDist < 0.5 * Contact::THRESHOLD) break;
			t += deltat;
		}
	}

	int retVal;
	if (deltat < 1.0e-20 || t < 0) {
#ifdef GRASPITDBG
		std::cerr << "t: " << t << "  d: " << deltat << std::endl;
		std::cerr << "I am " << getName().latin1() << std::endl;
		for (int i=0; i<(int)colReport->size(); i++) {
			std::cerr << (*colReport)[i].first->getName().latin1()<<" -- " 
				<< (*colReport)[i].second->getName().latin1() << std::endl;
		}
		std::cerr << "min dist: " << minDist << std::endl << std::endl;
#endif
		DBGA("Robot joint interpolation error");
		retVal = 0;
	} else {
		DBGP("Interpolation to t=" << t << " (deltat=" << deltat << ")");
		retVal = 1;
	}
	delete [] currentVals;
	*interpolationTime = t;
	return retVal;
}

/*! A utility function for the main static DOF movement functions. Given a link
that is stopped (presumably due to some contact) it will mark all the joints
that affect that link as stopped. This is done by setting the a bit in the 
entry that correspinds to a given joint in the vector \a stoppedJoints. 
\a stoppedJoints is assumed to be large enough for all the joints in the robot.
*/
void Robot::stopJointsFromLink(Link *link, double *desiredJointVals, int *stoppedJoints)
{
	if (link->getChainNum()<0) return; //it's the base
	KinematicChain *chain = chainVec[link->getChainNum()];
	for (int j=chain->getLastJoint( link->getLinkNum() ); j>=0; j--) {
		int jnum = chain->getJoint(j)->getNum();
		if (desiredJointVals[jnum] > chain->getJoint(j)->getVal()) {
			//stopped in the positive direction
			stoppedJoints[jnum] |= 1;
		} else if (desiredJointVals[jnum] < chain->getJoint(j)->getVal()){
			//stopped in the negative direction
			stoppedJoints[jnum] |= 2;
		}
	}
}

/*! One of the main functions for moving DOF's in statics. Given a set of \a
desiredDofVals, this will ask the DOF's what values should be given to the
joints. The vector \a stoppedJoints contains information about which joints
have been stopped due to contacts. On exit, \a jointVals will contain the 
needed joint values, and \a actualDofVals will contain the DOF values that
were actually set (which might be different from \a desireDofVals because
contacts might prevent the desired motion.

The main reason for this implementation is that different types of DOF's 
react different to contacts and implement coupling in their own differnt
way.

Returns \a true if at least on the joints of the robot is still moving, or
\false if contacts prevent all motion, limits have been reached and no more
movement is possible.
*/
bool
Robot::getJointValuesFromDOF(const double *desiredDofVals, double *actualDofVals, 
							 double *jointVals, int *stoppedJoints)
{
	bool done,moving;
	std::vector<transf> newLinkTran;
	DBGP("Getting joint movement from DOFs");
	do {
		moving = false; done = true;
		getJointValues(jointVals);
		//compute the aggregate move for all DOF's
		for (int d=0;d<numDOF;d++){
			//this check is now done by each DOF independently
			//if ( fabs(dofVals[d] - dofVec[d]->getVal()) < 1.0e-5) continue;
			if (dofVec[d]->accumulateMove(desiredDofVals[d], jointVals,stoppedJoints)) {
				moving = true;
				actualDofVals[d] = desiredDofVals[d];
				DBGP("DOF " << d << " is moving");
			} else {
				actualDofVals[d] = dofVec[d]->getVal();
			}
		}
		if (!moving) {
			DBGP("No DOF movement; done.");
			break;
		}
		//see if motion is allowed by existing contacts
		for (int c = 0; c < getNumChains(); c++) {
			KinematicChain *chain = getChain(c);
			newLinkTran.resize(chain->getNumLinks(), transf::IDENTITY);
			chain->infinitesimalMotion(jointVals, newLinkTran);
			for (int l=0; l<chain->getNumLinks(); l++){
				Link *link = chain->getLink(l);
				transf motion = newLinkTran[l];
				if ( link->contactsPreventMotion(motion) ) {
					DBGP("Chain " << link->getChainNum() << " link " << link->getLinkNum() << " is blocked.");
					//we stop all joints that are in the same chain before the stopped link
					stopJointsFromLink(link, jointVals, stoppedJoints);
					done = false;
					//once proximal links in a chain have been stopped, let's process distal links again
					//before making a decision on them, as their movement will be different
					break;
				}
			}
		}
		if (done) {
			DBGP("All movement OK; done.");
		}
	} while (!done);
	return moving;
}

/*! The core of the robot DOF movement in statics. Given a set of \a desiredVals for the
DOF's, it will set the DOF's to that value. If the result is free of contact, we are
done. If the result is in collision, it will interpolate to find the exact moment
of contact and set the DOF's at that value. \a stoppedJoints marks any joints that
can not move (presumably due to some contact discovered earlier). Returns the number
of collisions found (and resolved) in \a numCols.

After completing the move, it will also mark all the new contacts that have appeared
due to the move. It will also mark as stopped the joints of the links that are now in 
contact, by setting the appropriate bits in \a stoppedJoints.

In theory, this function should always leave the robot in a legal state, with no
inter-penetrations. 

Returns \a true if the move had been performed successfully. Returns \a false if either
no motion was performed because all dofs are already at the desired values, or contacts
prevent all motion. Also returns \a false if there is an error in the interpolation.

A final note of warning: this rather involved way of doing this was dictated by multiple
problems: always avoid leaving the robot in an illegal state; allow different dof's to
react to stopped joints in their own way; don't compute contacts more often then you
have to (it's expensive); make sure the dof's move together, and not one by one; handle
the case where the collision detection engine finds a collision, but fails to detect a 
contact after it's solved (happens very rarely, but it does); etc. In general, this whole
process is more complicated than it appears at first. We would love a general and robust
solution to this, but tread carefully here.
*/
bool
Robot::jumpDOFToContact(double *desiredVals, int *stoppedJoints, int *numCols)
{
	CollisionReport colReport, lateContacts;

	//get new joint values and see which dof's are moving
	double *newDofVals = new double[ getNumDOF() ];
	double *initialDofVals = new double [ getNumDOF() ];
	getDOFVals(initialDofVals);

	double *newJointVals = new double[ getNumJoints() ];
	double *initialJointVals = new double[ getNumJoints() ];
	getJointValues(initialJointVals);

	if (!getJointValuesFromDOF(desiredVals, newDofVals, newJointVals, stoppedJoints)) {
		DBGP("No movement of DOFs; forceDOFTo returning false");
		if (numCols) *numCols = 0;
		delete [] initialJointVals;	delete [] newJointVals;
		delete [] initialDofVals; delete [] newDofVals;
		return false;
	}

	//compute list of links that have actually moved
	//question: should the dof's compute this?
	std::vector<Body*> interestList;
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<chainVec[c]->getNumLinks(); l++) {
			int j = chainVec[c]->getLastJoint(l);
			if ( !stoppedJoints[ chainVec[c]->getJoint(j)->getNum() ] ) {
				interestList.push_back( chainVec[c]->getLink(l) );
			}
		}
	}

	setJointValuesAndUpdate(newJointVals);
	bool interpolationError = false;
	double interpolationTime;
	while (1) {
		myWorld->getCollisionReport(&colReport, &interestList);
		//if we are free of collision, we are done
		if (colReport.empty()) break;
		//if not, we need to interpolate
		//goal is to interpolate to current joint values
		getJointValues(newJointVals);
		if (!interpolateJoints(initialJointVals, newJointVals, &colReport, &interpolationTime)) {
			DBGA("Interpolation failed!");
			interpolationError = true;
			break;
		}
		//save the list of contacting bodies
		lateContacts.clear();
		for (int i=0;i<(int)colReport.size();i++) {
			lateContacts.push_back( colReport[i] );
			DBGP("Contact: " << colReport[i].first->getName().latin1() << "--" << 
				colReport[i].second->getName().latin1());
		}
		for (int d=0; d<numDOF; d++) {
			DBGP("Dof " << d << "initial " << initialDofVals[d] << " new " << newDofVals[d]);
			newDofVals[d] =     newDofVals[d] * interpolationTime + 
				initialDofVals[d] * ( 1.0 - interpolationTime);
			DBGP("Interpolated: " << newDofVals[d]);
		}
	}

	if (!interpolationError) {
		DBGP("ForceDOFTo done");
		myWorld->findContacts(lateContacts);
		for (int i=0; i<(int)lateContacts.size(); i++) {
			if (lateContacts[i].first->getOwner()==this) {
				stopJointsFromLink( (Link*)lateContacts[i].first, newJointVals, stoppedJoints);
			}
			if (lateContacts[i].second->getOwner()==this) {
				stopJointsFromLink( (Link*)lateContacts[i].second, newJointVals, stoppedJoints);
			}
		}
		updateDofVals(newDofVals);
		if (numCols) *numCols = (int)lateContacts.size();
	}

	delete [] initialDofVals; delete [] newDofVals;
	delete [] initialJointVals; delete [] newJointVals;

	if (!interpolationError) return true;
	else return false;
}


/*! This is the only interface for the user to perform robot DOF movement in statics.
The desired dof vals are specified in \a desiredVals, which should be of size
equal to the number of dofs of this robot.

The most important aspect is that the motion can be broken down in small steps, to
make sure that no collisions are missed along the way by "jumping through" an obstacle.
The other important aspect is that, when a contact is found and a DOF is stopped,
the other DOF's can continue to move.

\a desiredSteps holds the size of the desired steps for each DOF. We usually use 10 
degrees	for rotational DOFs and 50*Contact::THRESHOLD for translational DOF's. You can 
use WorldElement::ONE_STEP if no stepping is to be performed. If \a desiredSteps is 
NULL it has the same effect as setting all entried to ONE_STEP.

If \a stopAtContact is true, all movement ends as soon as the first contact is detected.
If not, movement proceeds until all DOF's have either reached the desired value, reached
their limit, or have been stopped due to contact.

Works by breaking down the motion in small time steps and uses the internal forceDOFTo 
function repeatedly for each step.Returns true if the joints were moved; if no movement 
was possible from the beginning, it returns false.

Note that the desired steps are treated as absolute values; this function will
automatically choose the right sign for the steps based on the direction of movement.
In the past, the sign of the desired step used to matter, so some functions inside
GraspIt still check for that themselves; that should no be necessary anymore.
*/

bool 
Robot::moveDOFToContacts(double *desiredVals, double *desiredSteps, bool stopAtContact, bool renderIt)
{
	//	PROF_RESET_ALL;
	//	PROF_START_TIMER(MOVE_DOF);
	PROF_TIMER_FUNC(MOVE_DOF);

	int i,d;
	CollisionReport result;

	double *stepSize= new double[numDOF];
	double *currVals = new double[numDOF];
	double *newVals = new double[numDOF];

	for (i=0;i<numDOF;i++) {
		if (!desiredSteps || desiredSteps[i] == WorldElement::ONE_STEP ) {
			stepSize[i] = desiredVals[i] - dofVec[i]->getVal();
		} else if (desiredSteps[i]!=0.0) {
		        //make sure the steps are pointing in the right direction
		        double factor = 1.0;
			if (desiredVals[i] < dofVec[i]->getVal()) factor = -1.0;
			stepSize[i] = factor * fabs(desiredSteps[i]);
		} else {
			stepSize[i] = 0.0;
			desiredVals[i] = dofVec[i]->getVal();
		}
		DBGP("from " << dofVec[i]->getVal() << " to " << desiredVals[i] << " in " << stepSize[i] << " steps");
	}

	//this vector will keep tracked of the joints that have been stopped
	//at previous time steps due to contacts.
	int *stoppedJoints = new int[numJoints];
	for (int j=0; j<numJoints; j++) {
		stoppedJoints[j] = 0;
	}

	//loop until termination conditions are met
	int numCols,itercount = 0;
	bool moved = false;
	do {
		itercount++;
		DBGP("Move to contact cycle");
		getDOFVals(currVals);
		for (d=0;d<numDOF;d++) {
			newVals[d] = currVals[d] + stepSize[d];
			DBGP("  Current value (a): " << currVals[d] << "; new value: " << newVals[d] << 
			     "; step size: " << stepSize[d]);
			if (stepSize[d] > 0 && newVals[d] > desiredVals[d])
				newVals[d] = desiredVals[d];
			else if (stepSize[d] < 0 && newVals[d] < desiredVals[d])
				newVals[d] = desiredVals[d];
			DBGP("  Current value (b): " << currVals[d] << "; new value: " << newVals[d]);
		}
		//perform one more step. jumpDOFToContact will inform us if no more
		//movement is done (or possible) and we can exit.
		if (!jumpDOFToContact(newVals, stoppedJoints, &numCols)){
			DBGP("ForceDOFTo reports no movement is possible");
			break;
		}
		moved = true;
		bool stopRequest = false;
		//inform whoever is listening a step has been performed
		emit moveDOFStepTaken(numCols, stopRequest);
		if (stopRequest) {
			DBGP("Receiver of movement signal requests early exit");
			break;
		}
		//check if we stop at first contact
		if (stopAtContact && numCols != 0) {
			DBGP("Stopping at first contact");
			break;
		}
		if (itercount > 500) {
			DBGA("MoveDOF failsafe hit");
			break;
		}
		if (renderIt && (itercount%25==0) && graspItGUI && graspItGUI->getIVmgr()->getWorld()==myWorld) {
			graspItGUI->getIVmgr()->getViewer()->render();
		}
	} while (1);

	//	PROF_STOP_TIMER(MOVE_DOF);
	//	PROF_PRINT_ALL;

	delete [] currVals;
	delete [] newVals;
	delete [] stepSize;
	delete [] stoppedJoints;
	return moved;
}

/*! This function checks if the path to the desired vals is legal. Breaks 
down motion into steps and does each step. As soon as any collision is 
detected, it returns false. Duplicates a lot of the functionality of 
moveDOFToContacts. This is not a very good implementation and should
probably be cleaned up in the future.
*/
bool
Robot::checkDOFPath(double *desiredVals, double desiredStep)
{
	int d;
	bool done, success = true;

	double *stepSize= new double[numDOF];
	double *currVals = new double[numDOF];
	double *newVals = new double[numDOF];

	for (d=0;d<numDOF;d++) {
		if (desiredStep == WorldElement::ONE_STEP )
			stepSize[d] = desiredVals[d] - dofVec[d]->getVal();
		else if ( desiredVals[d] >= dofVec[d]->getVal() )
			stepSize[d] = desiredStep;
		else stepSize[d] = - desiredStep;
	}

	do {
		DBGP("Move to contact cycle")
			getDOFVals(currVals);
		for (d=0;d<numDOF;d++) {
			newVals[d] = currVals[d] + stepSize[d];
			if (stepSize[d] > 0 && newVals[d] > desiredVals[d])
				newVals[d] = desiredVals[d];
			else if (stepSize[d] < 0 && newVals[d] < desiredVals[d])
				newVals[d] = desiredVals[d];
		}

		forceDOFVals(newVals);

		if ( !myWorld->noCollision() ) {
			success = false;
			break;
		}

		done = true;
		for (d=0;d<numDOF;d++) {
			if (newVals[d] != desiredVals[d]) done = false;
		}

	} while (!done);

	delete [] currVals;
	delete [] newVals;
	delete [] stepSize;
	return success;
}

//-----------------------------static joint torques computations------------------------

/*! Returns the static torques on all the joints of this robot. This is 
relevant for underactuated compliant hands only, should be zero in all 
other cases. See the CompliantDOF class for details.

If \useDynamicDofForce is set, the static joint torques are computed using
the dynamic force currently set by each DOF. If not, the minimum values for
balancing the system are computed. This is somewhat hacky as this is done
in statics, not dynamics, but we needed a way of computing static joint 
torques for some pre-specified level of DOF force.
*/
Matrix 
Robot::staticJointTorques(bool useDynamicDofForce)
{
	std::vector<double> jointTorques(getNumJoints(), 0.0);
	for (int d=0; d<numDOF; d++) {
		double dofForce = -1;
		if (useDynamicDofForce) {
			dofForce = dofVec[d]->getForce();
		}
		dofVec[d]->computeStaticJointTorques(&jointTorques[0], dofForce);
	}
	return Matrix(&jointTorques[0], getNumJoints(), 1, true);
}

//---------------------- miscellaneous functions ---------------------------

/*! Tells us how far along the pre-specified approach distance a certain object is.
Since the approach direction might never intersect the given body, it also
needs a cap telling is how far to look. This cap is \a maxDist. Therefore,
a return value of \a maxDist might mean that the object is never hit by moving
along the approach direction.
*/
double
Robot::getApproachDistance(Body *object, double maxDist)
{
	position p0 = position(0,0,0) * getApproachTran() * getTran();
	position p = p0;
	vec3 app = vec3(0,0,1) * getApproachTran() * getTran();
	bool done = false;
	double totalDist = 0;
	vec3 direction;
	int loops = 0;
	while (!done) {
		loops++;
		//compute shortest distance between current point and object;
		direction = myWorld->pointDistanceToBody(p, object);

		//current closest point on the object
		position pc = p; pc+=direction;
		//and its direction rel. to approach direction
		if ( normalise(pc-p0) % app > 0.86 ) {
			done = true;
		}
		//advance along approach direction
		double d = direction.len();
		totalDist += d;
		p += d * app;
		if (totalDist > maxDist) done = true;
		if (loops > 10) {
			done = true;
			totalDist = maxDist+1;
			DBGA("Force exit from gettAppDist");
		}
	}
	//if (totalDist > maxDist) {DBGA("Object far away in " << loops << " loop(s)");}
	//else {DBGA("Object at " << totalDist << " in " << loops << " loop(s)");}
	return totalDist;
}

/*!	This function snaps a kinematic chain out of collision with an object and into 
contact - if possible. It moves the chain "out" (in the opposite direction of 
autograsp) a little bit. If this is a valid	position, it interpolates between 
it and the original collision position to find the moment of contact.
*/
bool
Robot::snapChainToContacts(int chainNum, CollisionReport colReport)
{
	bool persistent = true;

	KinematicChain *chain = getChain(chainNum);
	chain->filterCollisionReport(colReport);
	if (colReport.empty()) {
		DBGP("Snap to chain "<<chainNum<<" done from the start - no collisions");
		return true;
	}

	std::vector<double> openVals(numDOF);
	std::vector<double> closedVals(numDOF);
	std::vector<double> openJointVals(numJoints);
	std::vector<double> closedJointVals(numJoints);
	CollisionReport openColReport;

	getDOFVals(&closedVals[0]);
	getDOFVals(&openVals[0]);
	getJointValues(&closedJointVals[0]);

	double SNAP = 0.1;
	while (1) {
		bool limitHit = true;
		int l = chain->getNumLinks() - 1;
		for (int j=chain->getLastJoint(l); j>=0; j--) {
			int d = chain->getJoint(j)->getDOFNum();
			double current = dofVec[d]->getVal();
			double target;
			if (dofVec[d]->getDefaultVelocity() < 0) {
				target = current + SNAP;
				if ( target > dofVec[d]->getMax() ) target = dofVec[d]->getMax();
				else limitHit = false;
			} else if (dofVec[d]->getDefaultVelocity() > 0) {
				target = current - SNAP;
				if ( target < dofVec[d]->getMin()) target = dofVec[d]->getMin();
				else limitHit = false;
			} else target = current;
			openVals[d] = target;
		}

		forceDOFVals(&openVals[0]);
		myWorld->getCollisionReport(&openColReport);
		chainVec[chainNum]->filterCollisionReport(openColReport);
		if ( !openColReport.empty() ) {
			if (persistent && !limitHit) {
				//try again
				forceDOFVals(&closedVals[0]);
				SNAP += 0.1;
			}
			else break;
		} else {
			break;
		}
	}
	if (!openColReport.empty()) {
		DBGP("Snap to chain "<<chainNum<<" done - open position in collision");
		forceDOFVals(&closedVals[0]);
		return false;
	}

	DBGP("Open values valid");
	getJointValues(&openJointVals[0]);
	double time;
	if (!interpolateJoints(&openJointVals[0], &closedJointVals[0], &colReport, &time)) {
		DBGP("Snap to chain "<<chainNum<<" interpolation failed");
		forceDOFVals(&closedVals[0]);
		return false;
	} else {
		//set dof vals according to interpolation time
		for (int d=0; d<numDOF; d++) {
			closedVals[d] = closedVals[d] * time +  openVals[d] * ( 1.0 - time);
		}
		updateDofVals(&closedVals[0]);
	}
	DBGP("Snap to chain "<<chainNum<<" success");
	myWorld->findContacts(colReport);
	return true;
}


//-----------------------DYNAMICS-------------------------------

/*! This is the main function for moving the robot DOF's in dynamics. All the
user can do is set the desired dof vals. This function will compute a smooth 
trajectory for each joint so that each has a smooth acceleration and velocity
profile. The intermediate values become the setpoints for each
joint controller.

After that, we rely on the simulation world to run each dynamic time step and
call the robot's dof controllers which will set dof dynamic forces based on
the desired dof values. The next dynamic time step will then move the links
based on the forces applied by the dof's.

In practice, the dof controllers are tricky and I've never been able to make
them very robust. They work reasonably well, but not in all cases.
*/
void
Robot::setDesiredDOFVals(double *dofVals)
{
	int i,j,d,numSteps;
	double timeNeeded;
	double t;
	double coeffs[5];    
	double tpow,q1,q0,qd0,qd1;
	double *traj;

	for (d=0;d<numDOF;d++) {
		if (dofVec[d]->getDefaultVelocity() == 0.0) continue;

		DBGP("DOF "<<d<<" trajectory");
		dofVec[d]->setDesiredPos(dofVals[d]);
		if (dofVec[d]->getVal() != dofVals[d]) {

			q0 = dofVec[d]->getVal();
			q1 = dofVals[d];
			qd0 = 0.0;//dofVec[d]->getActualVelocity();
			qd1 = 0.0;

			timeNeeded = fabs(dofVals[d]-dofVec[d]->getVal()) / fabs( dofVec[d]->getDesiredVelocity() );

			//make this a whole number of timesteps
			numSteps = (int)ceil(timeNeeded/myWorld->getTimeStep());
			timeNeeded = numSteps*myWorld->getTimeStep();

			DBGP("numSteps: "<< numSteps << " time needed: " << timeNeeded);
			traj = new double[numSteps];

			for (i=0;i<numSteps;i++) {
				t = (double)i/(double)(numSteps-1);
				coeffs[4] = 6.0*(q1 - q0) - 3.0*(qd1+qd0)*timeNeeded;
				coeffs[3] = -15.0*(q1 - q0) + (8.0*qd0 + 7.0*qd1)*timeNeeded;
				coeffs[2] = 10.0*(q1 - q0) - (6.0*qd0 + 4.0*qd1)*timeNeeded;
				coeffs[1] = 0.0;
				coeffs[0] = qd0*timeNeeded; 
				traj[i] = q0;

				tpow = 1.0;
				for (j=0;j<5;j++) {
					tpow *= t;
					traj[i] += tpow*coeffs[j];
				}
				DBGP(i << " " << t << " " << traj[i]);
			}
			dofVec[d]->setTrajectory(traj,numSteps);
			delete [] traj;
		}
	}
}

/*!
Given a kinematic chain number and a vector of chain end poses, this
computes the inverse kinematics for each pose, and computes a smooth
trajectory for each DOF to reach each pose.
*/
void
Robot::setChainEndTrajectory(std::vector<transf>& traj,int chainNum)
{
	int i,j,numPts = traj.size();
	double *dofVals = new double[numDOF];
	bool *dofInChain = new bool[numDOF];

	if (numPts < 1 || chainNum < 0 || chainNum > numChains-1) return;
	for (j=0;j<numDOF;j++)
		dofInChain[j] = false;

	for (j=0;j<chainVec[chainNum]->getNumJoints();j++)
		dofInChain[chainVec[chainNum]->getJoint(j)->getDOFNum()] = true;

	invKinematics(traj[0],dofVals,chainNum);
	for (j=0;j<numDOF;j++)
		if (dofInChain[j]) dofVec[j]->setTrajectory(&dofVals[j],1);

	for (i=1;i<numPts;i++) {
		invKinematics(traj[i],dofVals,chainNum);
		for (j=0;j<numDOF;j++)
			if (dofInChain[j]) dofVec[j]->addToTrajectory(&dofVals[j],1);
	}
	delete [] dofVals;
	delete [] dofInChain;
}


/*!
Given a start and end pose, this creates a number of intermediate poses
that linearly interpolate between the two in cartesian space.  The
start and end velocities of the end effector default to 0 but can be
changed.  If the time needed for the move is not set, the trajectory
will have an average velocity equal to the \a defaultTranVel and
\a defaultRotVel.  The number of poses generated is determined by the
amount of time needed for the move divided by the default dyanmic time
step length.
*/
void
Robot::generateCartesianTrajectory(const transf &startTr, const transf &endTr,
								   std::vector<transf> &traj,
								   double startVel, double endVel,
								   double timeNeeded)
{
	int i,j,numSteps;
	double t,tpow,dist,angle,coeffs[5];    
	vec3 newPos,axis;
	Quaternion newRot;

	(endTr.rotation() * startTr.rotation().inverse()).ToAngleAxis(angle,axis);

	if (timeNeeded <= 0.0) {
		if (defaultTranslVel == 0.0 || defaultRotVel == 0.0) return;
		timeNeeded =
			MAX((endTr.translation() - startTr.translation()).len()/defaultTranslVel,
			fabs(angle)/defaultRotVel);
	}

	//make this a whole number of timesteps
	numSteps = (int)ceil(timeNeeded/myWorld->getTimeStep());
	timeNeeded = numSteps*myWorld->getTimeStep();
	DBGP("Desired Tran Traj numSteps " << numSteps);  

	if (numSteps) {
		traj.clear();
		traj.reserve(numSteps);
	}

	for (i=0;i<numSteps;i++) {
		t = (double)i/(double)(numSteps-1);

		coeffs[4] = 6.0 - 3.0*(startVel+endVel)*timeNeeded;
		coeffs[3] = -15.0 + (8.0*startVel + 7.0*endVel)*timeNeeded;
		coeffs[2] = 10.0 - (6.0*startVel + 4.0*endVel)*timeNeeded;
		coeffs[1] = 0.0;
		coeffs[0] = startVel*timeNeeded; 

		dist = 0.0;    
		tpow = 1.0;
		for (j=0;j<5;j++) {
			tpow *= t;
			dist += tpow*coeffs[j];
		}
		newRot = Quaternion::Slerp(dist,startTr.rotation(),endTr.rotation());
		newPos = (1.0-dist)*startTr.translation() + dist*endTr.translation();
		traj.push_back(transf(newRot,newPos));
	}
}

/*!
This is only called when dynamics is on.  It updates the joint values
of the dynamic joints. It is important that this be called after each 
dynamic movement because other function then look at joint->getVal()
for many different reasons.
*/
void
Robot::updateJointValuesFromDynamics()
{
	double *jointVals = new double[ getNumJoints() ];
	//accumulate joint values as decided by the dynamics
	for (int f=0;f<numChains;f++) {
		for (int l=0;l<chainVec[f]->getNumLinks();l++) {
			if (chainVec[f]->getLink(l)->getDynJoint())
				chainVec[f]->getLink(l)->getDynJoint()->updateValues();
		}
		for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
			jointVals[ chainVec[f]->getJoint(j)->getNum() ] = chainVec[f]->getJoint(j)->getDynamicsVal();
		}
	}
	//set the joint themselves
	setJointValues(jointVals);

	//we no longer set the DOFs according to what the dynamic joints look like
	//we just do it once when the dynamics are turned off
	//however, the PD controller still needs to do it and relies on jont vals having
	//been set here
	delete [] jointVals;
}

/*!
Applies joint friction forces as a pair of equal and opposite forces
to links connected at each joint.  Frictional forces resist the
direction of motion of the joint and uses a viscous friction model
with the coefficients defined in the robot configuration file.

Also applies other passive forces, such as spring forces for compliant
joints. Spring coefficients are also read from config file.
*/
void
Robot::applyJointPassiveInternalWrenches()
{
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<getChain(c)->getNumJoints(); j++) {
			getChain(c)->getJoint(j)->applyPassiveInternalWrenches();
		}
	}
}

/*! Transparently goes to each DOF and asks them to build the limit 
constraints however they want. See the corresponding DOF functions 
and users manual for dynamics for details.
*/
void 
Robot::buildDOFLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
								double* H, double *g, int &hcn)
{
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->buildDynamicLimitConstraints(islandIndices, numBodies, H, g, hcn);
	}
}

/*! Transparently goes to each DOF and asks them to build the coupling 
constraints however they want. See the corresponding DOF functions 
and users manual for dynamics for details.
*/
void
Robot::buildDOFCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
								   double* Nu, double *eps, int &ncn)
{	
	for(int d=0; d<numDOF; d++) {
		dofVec[d]->buildDynamicCouplingConstraints(islandIndices, numBodies, Nu, eps, ncn);
	}
	return;
}

/*!
If the current simulation time is past the DOF setpoint update time
this updates the DOF set points.  Then the PD position controller for
each DOF computes the control force for the DOF given the length of the
current timestep.
*/
void
Robot::DOFController(double timeStep)
{
	DBGP(" in Robot controller");
	if (myWorld->getWorldTime() > dofUpdateTime) {
		DBGP("Updating setpoints");
		for (int d=0;d<numDOF;d++) {
			dofVec[d]->updateSetPoint();
		}
		dofUpdateTime += myWorld->getTimeStep();
	}
	for (int d=0;d<numDOF;d++) {
		dofVec[d]->callController(timeStep);
	}
}


/*! A specialized function that attempts to see if the autograsp has completed,
when executed dynamically. Does not do a great job at it, as this turned out
to be a much trickier problem than expected.
*/
bool 
Robot::dynamicAutograspComplete()
{
	for (int c=0; c<numChains; c++) {
		Link *l = chainVec[c]->getLink( chainVec[c]->getNumLinks()-1 );
		//if the last link has a contact, the autograsp *might* have ended
		if (l->getNumContacts()) continue;
		int jNum = chainVec[c]->getLastJoint(chainVec[c]->getNumLinks()-1);
		Joint *j = chainVec[c]->getJoint(jNum);
		double limit;
		//which joint limit are we going towards
		if (dofVec[j->getDOFNum()]->getDefaultVelocity() > 0) {
			if (j->getCouplingRatio() > 0) {
				limit = j->getMax();
			} else {
				limit = j->getMin();
			}
		} else if (dofVec[j->getDOFNum()]->getDefaultVelocity() < 0) {
			if (j->getCouplingRatio() < 0) {
				limit = j->getMax();
			} else {
				limit = j->getMin();
			}
		} else {
			assert(0);
		}
		//if this joint has hit its limit, it is possible that the autograsp is done
		if (fabs(j->getDynamicsVal() - limit) < 3.0e-2) continue;
		//if none or the above are true, the autograsp definitely is still going
		DBGP("Autograsp going on chain " << c << " joint " << jNum << ": val " 
			<< j->getDynamicsVal() << "; limit " << limit);
		return false;
	}
	return true;		
}

/*! Looks at dynamics LCP parameters to determine if any of the contacts are slipping
during dynamic simulation. Does not do a great job at it, as this turned out
to be a much trickier problem than expected.
*/
bool
Robot::contactSlip()
{
	double linearThreshold = 100.0;
	double angularThreshold = 2.0;
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<getChain(c)->getNumLinks(); l++) {
			Link *link = getChain(c)->getLink(l);
			if (!link->getNumContacts()) continue;
			const double *v = link->getVelocity();
			double magn = 0.0;
			for (int i=0; i<3; i++) {
				magn += v[i] * v[i];
			}
			if (magn > linearThreshold * linearThreshold) {
				DBGA("Chain " << c << " link " << l << " moving with linear vel. magnitude " << magn);
				return true;
			}
			magn = 0.0;
			for (int i=3; i<6; i++) {
				magn += v[i] * v[i];
			}
			if (magn > angularThreshold * angularThreshold) {
				DBGA("Chain " << c << " link " << l << " moving with angular vel. magnitude " << magn);
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//                           Hand Methods
//////////////////////////////////////////////////////////////////////////////

/*! Simply creates a new grasp object */
Hand::Hand(World *w,const char *name) : Robot(w,name)
{
	grasp = new Grasp(this);
}

/*! Deletes the associate grasp object */
Hand::~Hand()
{
	std::cout << "Deleting Hand: " << std::endl;
	delete grasp;
}

/*! Calls the Robot's clone fctn, but also sets this hand's grasp
to point to the same object as the original's
*/
void
Hand::cloneFrom(Hand *original)
{
	Robot::cloneFrom(original);
	grasp->setObjectNoUpdate( original->getGrasp()->getObject() );
	grasp->setGravity( original->getGrasp()->isGravitySet() );
}

/*!
Closes each DOF at a rate equal to \a speedFactor * its default velocity.
The closing continues until a joint limit is reached or a contact
prevents further motion.  The larger the \a speedFactor, the larger the
individual steps will be and increases the likelyhood that a collision
may be missed.  The \a renderIt flag controls whether the intermediate
steps will be rendered.

If \a stopAtContact is set and the execution is in static mode, movement
will stop as soon as any contact is made. Otherwise, movement will continue
untill all joints have been stopped and no movement is possible.

If you want the opposite motion, just pass a negative \a speedFactor.
*/
bool
Hand::autoGrasp(bool renderIt, double speedFactor, bool stopAtContact)
{
	int i;
	double *desiredVals = new double[numDOF];

	if (myWorld->dynamicsAreOn()) {
		for (i=0;i<numDOF;i++) {
			if (speedFactor * dofVec[i]->getDefaultVelocity() > 0)
				desiredVals[i] = dofVec[i]->getMax();
			else if (speedFactor * dofVec[i]->getDefaultVelocity() < 0)
				desiredVals[i] = dofVec[i]->getMin();		
			else desiredVals[i] = dofVec[i]->getVal();
			DBGP("Desired val "<<i<<" "<<desiredVals[i]);
			//for now
			dofVec[i]->setDesiredVelocity(speedFactor * dofVec[i]->getDefaultVelocity());
		}
		setDesiredDOFVals(desiredVals);
		delete [] desiredVals;
		return true;
	}

	double *stepSize= new double[numDOF];
	for (i=0;i<numDOF;i++) {
		if (speedFactor * dofVec[i]->getDefaultVelocity() >= 0) desiredVals[i] = dofVec[i]->getMax();
		else desiredVals[i] = dofVec[i]->getMin();
		stepSize[i] = dofVec[i]->getDefaultVelocity()*speedFactor*AUTO_GRASP_TIME_STEP;
	}

	bool moved = moveDOFToContacts(desiredVals, stepSize, stopAtContact, renderIt);
	delete [] desiredVals;
	delete [] stepSize;
	return moved;
}

/*
This is a shortcut (read "hack") that opens the hand (in the opposite direction 
of autoGrasp), in increments of speedFactor * each DOF range. Does it until no 
collision is detected. It does it in chunks, never does interpolation, if it can 
not find a collision-free pose it just returns false.
*/
bool
Hand::quickOpen(double speedFactor)
{
	double *desiredVals = new double[numDOF];
	getDOFVals(desiredVals);
	double *desiredSteps = new double[numDOF];
	for (int i=0; i<numDOF; i++) {
		double d = -1 * dofVec[i]->getDefaultVelocity();
		if ( d > 0 ) {
			desiredSteps[i] = d * speedFactor * (dofVec[i]->getMax() - desiredVals[i]);
		} else if ( d < 0 ) {
			desiredSteps[i] = d * speedFactor * (desiredVals[i] - dofVec[i]->getMin() );
		} else {
			desiredSteps[i] = 0;
		}
	}
	bool done = false, success = false;
	int loops = 0;
	while (!done) {
		loops++;
		//we move out at least once
		done = true;
		for (int i=0; i<numDOF; i++) {
			desiredVals[i] += desiredSteps[i];

			if ( desiredVals[i] > dofVec[i]->getMax() ) {desiredVals[i] = dofVec[i]->getMax();}
			else if ( desiredVals[i] < dofVec[i]->getMin() ){ desiredVals[i] = dofVec[i]->getMin();}
			else if (desiredSteps[i] != 0) {done = false;} //we can do at least another loop
			//DBGA("DOF " << i << " to " << desiredVals[i]);
		}
		forceDOFVals(desiredVals);
		if ( myWorld->noCollision() ) {
			done = true;
			success = true;
		}		
	}
	if (loops > 20) DBGA("Open finger loops: " << loops << " Hand: " << myName.latin1());
	delete [] desiredVals;
	delete [] desiredSteps;
	return success;
}

/*! Moves the palm (robot) in the direction specified by approachDirection. It 
stops if an object has been touched, or the maximum distance of \a moveDist
has been covered. Returns true if object has been hit. It expects an initial
state that is collision-free.
*/
bool
Hand::approachToContact(double moveDist, bool oneStep)
{
	transf newTran = translate_transf(vec3(0,0,moveDist) * getApproachTran()) * getTran();
	bool result;
	if (oneStep) {
		result = moveTo(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);
	} else {
		result = moveTo(newTran, 50*Contact::THRESHOLD, M_PI/36.0);
	}
	if (result) {
		DBGP("Approach no contact");
		return false;
	} else {
		DBGP("Approach results in contact");
		return true;
	}
}

/*! Moves the hand back along its approach direction until it is collision free, 
then forward until it finds the exact moment of contact. It moves forward at 
most \a moveDist. It can accept	an initial state that is in collision, as it 
will move back until collision is resolved.
*/
bool 
Hand::findInitialContact(double moveDist)
{
	CollisionReport colReport;
	while (myWorld->getCollisionReport(&colReport)) {
		transf newTran = translate_transf(vec3(0,0,-moveDist / 2.0) * 
			getApproachTran()) * getTran();
		setTran(newTran);
	}
	return approachToContact(moveDist, false);
}

Matrix
Robot::getJacobianJointToDOF(int chainNum){
	Matrix m = Matrix(chainVec[chainNum]->getNumJoints(), numDOF);
	//initialy they are irrelevant to each other
	m.setAllElements(0.0);
	for(int i = 0; i < m.rows(); ++i){
		//for each joint i
		m.elem(i,chainVec[chainNum]->getJoint(i)->getDOFNum()) = chainVec[chainNum]->getJoint(i)->getCouplingRatio();
	}
	return m;
}
