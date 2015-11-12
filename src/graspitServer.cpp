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
// Author(s): Andrew T. Miller 
//
// $Id: graspitServer.cpp,v 1.7 2009/03/25 22:10:04 cmatei Exp $
//
//######################################################################

/*! \file 
\brief Implements the application's TCP server.
*/

#include <QTextStream>
#include <iostream>
#include "graspitServer.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "contact.h"
#include "graspPlanningService.h"
#include "handControlAPI.h"
#include "qualityEstimator.h"
#include "BlindPlanner/graspExperienceBase.h"
#include "BlindPlanner/handAdjust.h"
#ifdef LOCAL_EXPLORE_ENABLED
#include "BlindPlanner/localExplore.h"
#endif
#include <sstream>
/*!
Stub destructor.
*/
ClientSocket::~ClientSocket()
{
#ifdef GRASPITDBG
	std::cout << "client socket destroyed"<<std::endl;
#endif
}

/*!
This reads the next portion of a command line after the command to collect
the body indices, and returns a vector of pointers to those bodies.  If
this portion starts with the word "ALL", then all the bodies in the world
are added to the body vector.  Otherwise it reads the number of body
indices, and reads each index in turn, adding the corresponding body
pointer to the vector.  If an index is read that does not exist, an error
message is sent back and this method returns FAILURE.
*/
int
ClientSocket::readBodyIndList(std::vector<Body *> &bodyVec)
{
	QTextStream os(this);
	int i,numBodies,bodNum;
	bool ok;
	World *world = graspItGUI->getIVmgr()->getWorld();
	std::cout << "ReadBodyIndList Line:"<<line.latin1() << std::endl;

	/* if the index list is empty, use every body and send
	back the count
	*/

	if (strPtr == lineStrList.end()) return FAILURE;

	if ((*strPtr).startsWith("ALL")) {
		strPtr++;
		for (i=0;i<world->getNumBodies();i++)
			bodyVec.push_back(world->getBody(i));
		std::cout << "Sending num bodies: "<<world->getNumBodies()<<std::endl;
		os << world->getNumBodies() << endl;
		return SUCCESS;
	}

	numBodies = (*strPtr).toInt(&ok);
	if (!ok) return FAILURE;
	strPtr++;

	for (i=0;i<numBodies;i++) {
		if (strPtr == lineStrList.end()) return FAILURE;
		bodNum = (*strPtr).toInt(&ok);
		if (!ok) return FAILURE;

		if (bodNum>=0 && bodNum<world->getNumBodies()) {
			bodyVec.push_back(world->getBody(bodNum));
			if (world->getBody(bodNum)==NULL) {
				os << "Error: Cannot find body " << bodNum <<"\n";
				return FAILURE;
			}
		}
		else {
			os << "Error: Cannot find body " << bodNum <<"\n";
			return FAILURE;
		}
		strPtr++;
	}
	return SUCCESS;
}


/*!
This reads the next portion of a command line after the command to collect
the robot indices, and returns a vector of pointers to those robots.  If
this portion starts with the word "ALL", then all the robots in the world
are added to the robot vector.  Otherwise it reads the number of robot
indices, and reads each index in turn, adding the corresponding robot
pointer to the vector.  If an index is read that does not exist, an error
message is sent back and this method returns FAILURE.
*/
int
ClientSocket::readRobotIndList(std::vector<Robot *> &robVec)
{
	QTextStream os(this);
	int i,robNum,numRobots;
	bool ok;
	World *world = graspItGUI->getIVmgr()->getWorld();
	std::cout << "ReadRobotIndList Line:"<<line.latin1() << std::endl;

	/* if the index list is empty, use every robot and send
	back the count
	*/
	if (strPtr == lineStrList.end()) return FAILURE;

	if ((*strPtr).startsWith("ALL")) {
		strPtr++;
		for (i=0;i<world->getNumRobots();i++)
			robVec.push_back(world->getRobot(i));
		std::cout << "Sending num robots: "<<world->getNumRobots()<<std::endl;
		os << world->getNumRobots() << endl;
		return SUCCESS;
	}

	numRobots = (*strPtr).toInt(&ok);
	if (!ok) return FAILURE;
	strPtr++;

	for (i=0;i<numRobots;i++) {
		if (strPtr == lineStrList.end()) return FAILURE;
		robNum = (*strPtr).toInt(&ok);
		if (!ok) return FAILURE;

		if (robNum>=0 && robNum<world->getNumRobots()) {
			robVec.push_back(world->getRobot(robNum));
			if (world->getRobot(robNum)==NULL) {
				os << "Error: Cannot find robot " << robNum <<"\n";
				return FAILURE;
			}
		}
		else {
			os << "Error: Cannot find robot " << robNum <<"\n";
			return FAILURE;
		}
		strPtr++;
	}
	return SUCCESS;
}

/*!
This is the main routine for parsing input on the clientSocket.
There should be one command for each line of input.  This reads one
line, and looks at the first word (up to the first space character) to
determine the command.   Then if there are body or robot indices to read,
it calls a support routine to read those and return a vector of bodies or
robots.  These are then passed to the appropriate routine to carry out the
action and write out any necessary results.
*/
void
ClientSocket::readClient()
{
	int i,numData,numBodies,numRobots;
	double time;
	std::vector<Body *> bodyVec;
	std::vector<Robot *> robVec;

	bool ok;

	while ( canReadLine() ) {
		line = readLine();
		line.truncate(line.length()-1); //strip newline character
		lineStrList =
			QStringList::split(' ',line);
		if (lineStrList.size() ==0)
			continue;
		strPtr = lineStrList.begin();

#ifdef GRASPITDBG
		std::cout <<"Command parser line: "<<line << std::endl;
#endif

		if (*strPtr == "getContacts") {
			strPtr++; if (strPtr == lineStrList.end()) continue;
			numData = (*strPtr).toInt(&ok); strPtr++;
			if (!ok) continue;

#ifdef GRASPITDBG
			std::cout << "Num data: "<<numData<<std::endl;
#endif

			if (readBodyIndList(bodyVec)) continue;
			numBodies = bodyVec.size();
			for (i=0;i<numBodies;i++)
				sendContacts(bodyVec[i],numData);
		}

		else if (*strPtr == "getAverageContacts") {
			strPtr++;
			if (readBodyIndList(bodyVec)) continue;
			numBodies = bodyVec.size();
			for (i=0;i<numBodies;i++)
				sendAverageContacts(bodyVec[i]);
		}

		else if (*strPtr == "getBodyName") {
			strPtr++;
			if (readBodyIndList(bodyVec)) continue;
			numBodies = bodyVec.size();
			for (i=0;i<numBodies;i++)
				sendBodyName(bodyVec[i]);
		}

		else if (*strPtr == "getRobotName") {
			strPtr++;
			if (readRobotIndList(robVec)) continue;
			numRobots = robVec.size();
			for (i=0;i<numRobots;i++)
				sendRobotName(robVec[i]);
		}

		else if (*strPtr == "getDOFVals") {
			strPtr++;
			if (readRobotIndList(robVec)) continue;
			numRobots = robVec.size();
			for (i=0;i<numRobots;i++)
				sendDOFVals(robVec[i]);
		}

		else if (*strPtr == "moveDOFs") {
			strPtr++;
			readDOFVals();
		}

		else if (*strPtr == "autoGrasp") {
			strPtr++;
			autoGraspCommand();
		}

		else if (*strPtr == "approachToContact") {
			strPtr++;
			approachToContactCommand();
		}

		else if (*strPtr == "render")
			graspItGUI->getIVmgr()->getViewer()->render();

		else if (*strPtr == "setDOFForces") {
			strPtr++;
			if (readRobotIndList(robVec)) continue;
			numRobots = robVec.size();
			for (i=0;i<numRobots;i++)
				if (readDOFForces(robVec[i])==FAILURE) continue;
		}

		else if ((*strPtr) == "moveDynamicBodies") {
			strPtr++;
			if (strPtr == lineStrList.end()) ok = FALSE;
			else {
				time = (*strPtr).toDouble(&ok); strPtr++;
			}
			if (!ok)
				moveDynamicBodies(-1);
			else
				moveDynamicBodies(time);
		}

		else if (*strPtr == "computeNewVelocities") {

#ifdef GRASPITDBG
			std::cout << "cnv" << std::endl;
#endif

			strPtr++; if (strPtr == lineStrList.end()) continue;
			time = (*strPtr).toDouble(&ok); strPtr++;
			if (!ok) continue;

#ifdef GRASPITDBG
			std::cout << time <<std::endl;
#endif
			computeNewVelocities(time);
		}
		else if((*strPtr) == "planGrasp") {
			std::cout << "Grasp planning command received: " << line.latin1() << std::endl;

			planGrasp();
			break;
		}
		else if((*strPtr) == "setBodyTransf") {
			strPtr ++;
			setBodyTransf();
			break;
		}
		else if((*strPtr) == "sendBodyTransf") {
			strPtr ++;
			sendBodyTransf();
			break;
		}
		else if((*strPtr) == "setRobotTransf") {
			strPtr ++;
			setRobotTransf();
			break;
		}
		else if((*strPtr) == "sendRobotTransf") {
			strPtr ++;
			sendRobotTransf();
			break;
		}
		else if((*strPtr) == "getRobotVertices") {
			//Usage - getRobotVertices #robots (or 'ALL') robot indeces
			std::vector<Robot *> rl;
			++strPtr;
			readRobotIndList(rl);
			getRobotVertices(rl);
			QTextStream os(this);
			os << "endoflist\n";
		}
		else if((*strPtr) == "getBodyVertices"){
			std::vector<Body *> bd;
			++strPtr;
			readBodyIndList(bd);
			getBodyVertices(bd);
			QTextStream os(this);
			os << "endoflist\n";
		}
		else if((*strPtr) == "estimateQuality"){
			//input is the joint angles and the tactile readings
			//joint values: f1,f2,f3,fs,b1,b2,b3
			std::cout << "Grasp quality estimate command received: " << line.latin1() << std::endl;
			estimateStability();
		}
		else if((*strPtr) == "queryExperience"){
			queryExperience();
		}
		else if((*strPtr) == "retrieveCommand"){
			retrieveCommand();
		}
#ifdef LOCAL_EXPLORE_ENABLED
		else if((*strPtr) == "startExplore"){
			startExplore();
		}
		else if((*strPtr) == "addPointCloud"){
			addPointCloud();
		}
		else if((*strPtr) == "endExplore"){
			endExplore();
		}
#endif
	}
}

/*!
Given a pointer to a body, this examines all the contacts on that body and
finds the average wrench acting on the body through those contacts and the
average contact location in body coordinates.  This is written to the
socket on 2 separate lines.
*/
void
ClientSocket::sendAverageContacts(Body* bod)
{
	QTextStream os(this);
	std::list<Contact *> contactList;
	std::list<Contact *>::iterator cp;
	int i,numContacts;
	double totalWrench[6]={0.0,0.0,0.0,0.0,0.0,0.0};
	double *wrench;
	vec3 totalLoc = vec3::ZERO;

	numContacts = bod->getNumContacts();
	contactList = bod->getContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		wrench = (*cp)->getDynamicContactWrench();
		for (i=0;i<6;i++) totalWrench[i] += wrench[i];
		totalLoc += (*cp)->getContactFrame().translation();
	}

	if (numContacts>1) {
		for (i=0;i<6;i++) totalWrench[i] /= numContacts;
		totalLoc = totalLoc / numContacts;
	}

	os << totalWrench[0]<<" "<<totalWrench[1]<<" "<<totalWrench[2]<<
		" "<<totalWrench[3]<<" "<<totalWrench[4]<<" "<<totalWrench[5]<<"\n";
	os << totalLoc[0] << " "<<totalLoc[1] << " " <<totalLoc[2]<<"\n";

}

/*!
Given a pointer to a body, this writes the name of the body and a newline
to the socket.
*/
void
ClientSocket::sendBodyName(Body* bod)
{
	QTextStream os(this);
	std::cout << "sending " << bod->getName().latin1() << "\n";
	os << bod->getName().latin1() << "\n";
}

/*!
Given a pointer to a robot, this writes the name of the robot and a newline
to the socket.
*/
void
ClientSocket::sendRobotName(Robot* rob)
{
	QTextStream os(this);
	std::cout << "sending " << rob->getName().latin1() << "\n";
	os << rob->getName().latin1() << "\n";
}

/*!
Given a pointer to a body, this first writes a line containing the number
of contacts on the body.  Then it writes \a numData lines for each contact.
The first line contains the 6 numbers of the contact wrench.  The next line
(if required) contains the 3 numbers specifying the contact location in
body coordinates, and the last line (if required) contains the scalar
constraint error for that contact.
*/
void
ClientSocket::sendContacts(Body *bod,int numData)
{
	QTextStream os(this);
	std::list<Contact *> contactList;
	std::list<Contact *>::iterator cp;
	vec3 loc;
	double err;
	double *wrench;

#ifdef GRASPITDBG
	std::cout << "sending numContacts: "<<bod->getNumContacts()<<std::endl;
#endif

	os << bod->getNumContacts()<<"\n";

	contactList = bod->getContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		wrench = (*cp)->getDynamicContactWrench();
		loc = (*cp)->getContactFrame().translation();
		err = (*cp)->getConstraintError();

		os << wrench[0]<<" "<<wrench[1]<<" "<<wrench[2]<<" "<<wrench[3]<<" "<<
			wrench[4]<<" "<<wrench[5]<<"\n";
		if (numData > 1) 
			os << loc[0] << " "<< loc[1] << " " << loc[2]<<"\n";
		if (numData > 2)
			os << err << "\n";
	}
}

/*!
Given a pointer to robot, this first writes a line to the socket containing
the number of %DOF's in the robot, then for each %DOF, it writes a line
containting the current value of that %DOF.
*/
void
ClientSocket::sendDOFVals(Robot *rob)
{
	QTextStream os(this);
	int i;

	os << rob->getNumDOF() << "\n";
	for (i=0;i<rob->getNumDOF();i++)
		os << rob->getDOF(i)->getVal() << "\n";
}

/*!
After the readDOFVals command was read by readClient, this expects to
read a valid robot index, then the correct number of %DOF's for this robot,
then a desired value for each %DOF, and finally a value for each DOF to step
by during the move.  It performs the %DOF moves, finds the new contacts,
updates the grasp, and then it sends one line for each %DOF containing
the actual value for the %DOF after the move.
*/
int
ClientSocket::readDOFVals()
{
	Robot *rob;
	double *val,*stepby;
	QTextStream os(this);
	int numDOF,i,robNum;
	bool ok=TRUE;

#ifdef GRASPITDBG
	std::cout << "in read dof vals"<<std::endl;
#endif

	if (strPtr == lineStrList.end()) ok=FALSE;
	if (ok) robNum = (*strPtr).toInt(&ok);

	if (!ok || robNum < 0 ||
		robNum >= graspItGUI->getIVmgr()->getWorld()->getNumRobots()) {
			os <<"Error: Robot does not exist.\n";
			return FAILURE;
	}
	rob = graspItGUI->getIVmgr()->getWorld()->getRobot(robNum);

#ifdef GRASPITDBG
	std::cout << "robnum: "<<robNum<<std::endl;
#endif

	strPtr++;
	if (strPtr == lineStrList.end()) return FAILURE;

	numDOF=(*strPtr).toInt(&ok);
	if (!ok) return FAILURE;
	strPtr++;

#ifdef GRASPITDBG
	std::cout << "read robot has: "<< numDOF << " DOF?"<<std::endl;
#endif

	if (numDOF < 1) {

#ifdef GRASPITDBG
		std::cout << "numDOF was zero."<<std::endl;
#endif
		return FAILURE;
	}
	if (numDOF != rob->getNumDOF()) {
		os <<"Error: robot has " << rob->getNumDOF() <<" DOF."<<endl;
		return FAILURE;
	}

	val = new double[numDOF];
	stepby = new double[numDOF];

	for (i=0;i<rob->getNumDOF();i++) {
		if (strPtr == lineStrList.end()) return FAILURE;
		val[i] = (*strPtr).toDouble(&ok);
		if (!ok) return FAILURE;
		strPtr++;
#ifdef GRASPITDBG
		std::cout<<val[i]<<" ";
#endif
	}

#ifdef GRASPITDBG
	std::cout<<std::endl;
#endif

	for (i=0;i<rob->getNumDOF();i++) {
		if (strPtr == lineStrList.end()) return FAILURE;
		stepby[i] = (*strPtr).toDouble(&ok);
		if (!ok) return FAILURE;
		strPtr++;
	}

	rob->moveDOFToContacts(val,stepby,true);

	// these should be separate commands
	graspItGUI->getIVmgr()->getWorld()->findAllContacts();
	graspItGUI->getIVmgr()->getWorld()->updateGrasps();

	for (i=0;i<rob->getNumDOF();i++) {
		os << rob->getDOF(i)->getVal() << "\n";

#ifdef GRASPITDBG
		std::cout << "Sending: "<< rob->getDOF(i)->getVal() << "\n";
#endif
	}
	delete [] val;
	delete [] stepby;
	return SUCCESS;
}

/*!
After the readDOFForces command was read by readClient, this expects to
the correct number of %DOF's for this robot, and then a desired force for
each %DOF.  It sets each %DOF force and sends a line for each containing
the current force.
*/
int
ClientSocket::readDOFForces(Robot *rob)
{
	double val;
	bool ok;
	// QTextStream is(this);
	QTextStream os(this);
	int numDOF,i;

	if (strPtr == lineStrList.end()) return FAILURE;

	numDOF=(*strPtr).toInt(&ok);
	if (!ok) return FAILURE;
	strPtr++;

#ifdef GRASPITDBG
	std::cout << "read robot has: "<< numDOF << " DOF?"<<std::endl;
#endif

	if (numDOF < 1) {
#ifdef GRASPITDBG
		std::cout << "numDOF was zero."<<std::endl;
#endif
		return FAILURE;
	}

	if (numDOF != rob->getNumDOF()) {
		os <<"Error: robot has " << rob->getNumDOF() <<" DOF."<<endl;
		return FAILURE;
	}

	for (i=0;i<rob->getNumDOF();i++) {
		if (strPtr == lineStrList.end()) return FAILURE;
		val = (*strPtr).toDouble(&ok);
		if (!ok) return FAILURE;
		strPtr++;
		rob->getDOF(i)->setForce(val);

#ifdef GRASPITDBG
		std::cout<<val<<" ";
#endif
	}

#ifdef GRASPITDBG
	std::cout<<std::endl;
#endif

	for (i=0;i<rob->getNumDOF();i++) {
		os << rob->getDOF(i)->getForce() << "\n";

#ifdef GRASPITDBG
		std::cout << "Sending: "<< rob->getDOF(i)->getForce() << "\n";
#endif
	}
	return SUCCESS;
}

/*
//not finished yet
void
ClientSocket::moveBody(Body *bod)
{
double x,y,z,ax,ay,az,r;

QTextStream is(this);
is>>x>>y>>z>>ax>>ay>>az>>r;
bod = NULL;  // unused parameter warning
}
*/

/*!
If given a positive time step value, this will move the dynamic world bodies
with that value.  Otherwise it uses the world default time step.  A line
with the actual length of that timestep is then sent back.
*/
void
ClientSocket::moveDynamicBodies(double timeStep)
{
	QTextStream os(this);
	if (timeStep<0)
		timeStep = graspItGUI->getIVmgr()->getWorld()->getTimeStep();

	double actualTimeStep =
		graspItGUI->getIVmgr()->getWorld()->moveDynamicBodies(timeStep);
	if (actualTimeStep < 0)
		os << "Error: Timestep failsafe reached.\n";
	else 
		os << actualTimeStep << "\n";
}

/*!
This calls the computeNewVelocities routine in the dynamics with the given
value of the timestep.  Afterwards, it sends out a line containing the
result code from that operation.
*/
void
ClientSocket::computeNewVelocities(double timeStep)
{
	QTextStream os(this);
	int result = graspItGUI->getIVmgr()->getWorld()->computeNewVelocities(timeStep);
	os << result << "\n";
}

// read in 7 param transf given as  pos(x y z) Qauternion(w x y z) 
int ClientSocket::readTransf(transf * tr){
	bool ok = true;
	try{
		*tr = transf(Quaternion((*(strPtr + 3)).toDouble(&ok),  //qw
			(*(strPtr + 4)).toDouble(&ok),  //qx
			(*(strPtr + 5)).toDouble(&ok),  //qy
			(*(strPtr + 6)).toDouble(&ok)), //qz
			vec3((*strPtr).toDouble(&ok), //x
			(*(strPtr+1)).toDouble(&ok),  //y
			(*(strPtr + 2)).toDouble(&ok)));  //z
		strPtr +=7;
	}
	catch(...){
		if (!ok)
			return FAILURE;
		else
			std::cout <<"unknown error in ClientSocket::readTransf \n";
	}
	return SUCCESS;
}

/*!
This reads a list of transforms from the string
*/
int ClientSocket::readTransfList(std::vector<transf> & transfList){
	//The string should look like [x y z qw qx qy qz x y z qw qx qy qz
	bool ok = true;
	if(*strPtr != "[")
		return FAILURE;
	strPtr ++;
	while(*strPtr != "]")
	{
		try{
			transf t = transf(Quaternion((*(strPtr + 3)).toDouble(&ok),  //qw
				(*(strPtr + 4)).toDouble(&ok),  //qx
				(*(strPtr + 5)).toDouble(&ok),  //qy
				(*(strPtr + 6)).toDouble(&ok)), //qz
				vec3((*strPtr).toDouble(&ok), //x
				(*(strPtr+1)).toDouble(&ok),  //y
				(*(strPtr + 2)).toDouble(&ok)));  //z
			strPtr +=7;
			transfList.push_back(t);
		}
		catch(...){
			if (!ok)
			{
				std::cout << "C" << std::endl;
				return FAILURE;
			}
			else
				std::cout <<"unknown error in ClientSocket::readTransfList \n";
		}
	}
	std::cout << "D" << std::endl;
	strPtr++; //skip "]" character
	return SUCCESS;
}


/////////////////////////// new functions for communicating with ROS, can be used for other purposes as well
/*
method_type = RETRIVAL, object_name is the scaled model name in the scaled_model table
method_type = ONLINE_PLAN, object_name should be the path to the scaled model
the hand used here should be assumed as Barrett
*/
GraspPlanningService* ClientSocket::mService;
HandControlAPI* ClientSocket::mHandControlAPI;
QualityEstimator* ClientSocket::mQualityEstimator;
GraspExperienceBase* ClientSocket::mGeb;
#ifdef LOCAL_EXPLORE_ENABLED
LocalExplore* ClientSocket::mLocalExplore;
#endif
void ClientSocket::planGrasp()
{	
	QString object_name;
	//Initialize defaults to standard dbase
	QString dbURL("fiji.cs.columbia.edu");
	QString dbPassword("super");
	QString dbLogin("postgres");
	QString dbName("arm_db"); 
	QString dbPort("5432"); 
	QString method_type("RETRIEVAL");
	QString hand_name = QString("BARRETT_RUBBER");
	vec3 workspace_approach_vector(1/sqrt(2.0),1/sqrt(2.0),0);
	double workspace_approach_angle(M_PI/4);
	unsigned int grasp_constraint = 0;
	transf table_pose(transf::IDENTITY), object_pose(transf::IDENTITY);
	std::vector<transf> uncertainPoses;
	strPtr ++;
	while (strPtr !=lineStrList.end()){
		bool ok = true;
		QString current_argument = *strPtr;
		if (*strPtr == "--object_pose"){
			//get object's pose: x y z qw qx qy qz
			strPtr ++;
			ok = readTransf(&object_pose) == SUCCESS;

		}
		else if(*strPtr == "--table_pose"){
			//get table's pose: x y z qw qx qy qz
			strPtr ++;
			ok = readTransf(&table_pose) == SUCCESS;
		}
		else if(*strPtr == "--object_name"){
			object_name = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--hand_name"){
			hand_name = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--method"){
			//get the type of the planning method, for task 0, we use RETRIVAL
			method_type = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--dbURL"){
			dbURL = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--dbName"){
			dbName = *(strPtr+1);	
			strPtr +=2;
		}
		else if(*strPtr == "--dbLogin"){
			dbLogin = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--dbPort"){
			dbPort = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--dbPassword"){
			dbPassword = *(strPtr+1);
			strPtr +=2;
		}
		else if(*strPtr == "--workspace_vector"){
			workspace_approach_vector = vec3((*(strPtr +1)).toDouble(&ok), 
				(*(strPtr +2)).toDouble(&ok),
				(*(strPtr +3)).toDouble(&ok));
			workspace_approach_angle = (*(strPtr +4)).toDouble(&ok);
			strPtr +=5;
		}
		else if(*strPtr == "--grasp_constraint"){
			grasp_constraint = (*(strPtr +1)).toInt(&ok);
			strPtr +=2;
		}
		else if(*strPtr == "--uncertain_pose_list"){
			strPtr +=1;
			ok = (readTransfList(uncertainPoses) == SUCCESS);
		}
		else{
			break;
		}
		if (!ok){
			std::cout << "Failed to process agument " << current_argument.toStdString();
		}
	}

	//TODO: if mservice's database has lost its connection, the validity of the mservice is not being tested here
	if(!mService){
		mService = new GraspPlanningService(dbURL, dbName, dbPort, dbLogin, dbPassword, grasp_constraint);
	}
	//	std::cout << "method type is: " << method_type.latin1() << std::endl;
	if(!strcmp(method_type.latin1(),"PLAN_TASK"))
	{
		mService->setParams(hand_name, object_name, method_type, this, object_pose,  workspace_approach_vector, workspace_approach_angle);

		mService->plan_from_tasks();
		return;
	}

	if(strcmp(method_type.latin1(),"RETRIEVAL") && strcmp(method_type.latin1(), "RETRIEVAL_UNCERTAIN"))
	{
		std::cout <<"Only retrieval type is supported so far, please input the object name in the CGDB and a set of pre-computed grasps will be returned" <<
			std::endl;
		feedback(QString("[]"));
		return;
	}

	if(!strcmp(method_type.latin1(),"RETRIEVAL"))
	{
		mService->setParams(hand_name, object_name, method_type, this, object_pose,  workspace_approach_vector, workspace_approach_angle);
		std::cout << "parameters set" << std::endl;
		mService->retrieve();
		std::cout << "grasps retrieved" << std::endl;
		mService->check(table_pose);
		std::cout << "grasps checked" << std::endl;
		mService->transform();
		std::cout << "grasps transformed" << std::endl;
		mService->rank();
		std::cout << "grasps ranked" << std::endl;
		feedback(mService->report());
		std::cout << "grasps returned" << std::endl;
	}
	if(!strcmp(method_type.latin1(),"RETRIEVAL_UNCERTAIN"))
	{
		mService->setParams(hand_name, object_name, method_type, this, object_pose,  workspace_approach_vector, workspace_approach_angle);
		std::cout << "parameters set" << std::endl;
		mService->retrieve();
		std::cout << "grasps retrieved" << std::endl;
		mService->check(table_pose);
		std::cout << "grasps checked" << std::endl;
		mService->planWithUncertainties(uncertainPoses);
		std::cout << "plan with uncertainties done" << std::endl;
		mService->transform();
		std::cout << "grasps transformed" << std::endl;
		mService->rank();
		std::cout << "grasps ranked" << std::endl;
		feedback(mService->report());
		std::cout << "grasps returned" << std::endl;
	}
}



void
ClientSocket::feedback(QString msg)
{
	QTextStream os(this);
	os.flush();
	os << msg.latin1();
	if(msg.length() > 2)
	{
		std::cout << "msg size is: " << msg.length() << std::endl;
		std::cout << "msg sent out: " << msg.latin1() << std::endl;
	}
//	else
//		std::cout << msg.latin1() << std::endl;
	os.flush();
}
//Body transforms are sent backwards by the planner for some reason compared to the overloading
//of << for transfs
void ClientSocket::setBodyTransf(){
	std::vector <Body *> bd;
	readBodyIndList(bd);
	transf object_pose;
	for(std::vector<Body *>::iterator bp = bd.begin(); bp != bd.end(); ++bp){
		readTransf(&object_pose);
		(*bp)->setTran(object_pose);
	}
}

void ClientSocket::sendBodyTransf(){
	std::vector <Body *> bd;
	readBodyIndList(bd);
	QTextStream os(this);
	for(std::vector<Body *>::iterator bp = bd.begin(); bp != bd.end(); ++bp){
		//this is a hack around the overloading for standard strings and not qstrings.  
		//this should be templated
		std::stringstream ss(std::stringstream::in);
		ss << (*bp)->getTran().translation() << (*bp)->getTran().rotation();
		os << QString(ss.str().c_str());
	}
	os << "\n";
	os.flush();
}

//Body transforms are sent backwards by the planner for some reason compared to the overloading
//of << for transfs
void ClientSocket::setRobotTransf(){
	std::vector <Robot *> rb;
	readRobotIndList(rb);
	transf robot_pose;
	for(std::vector<Robot *>::iterator rp = rb.begin(); rp != rb.end(); ++rp){
		readTransf(&robot_pose);
		(*rp)->setTran(robot_pose);
	}
}

void ClientSocket::sendRobotTransf(){
	std::vector <Robot *> rb;
	readRobotIndList(rb);
	QTextStream os(this);
	for(std::vector<Robot *>::iterator rp = rb.begin(); rp != rb.end(); ++rp){
		//this is a hack around the overloading for standard strings and not qstrings.  
		//this should be templated
		std::stringstream ss(std::stringstream::in);
		ss << (*rp)->getTran().translation() << (*rp)->getTran().rotation();
		os << QString(ss.str().c_str());
	}
	os << "\n";
	os.flush();
}

bool ClientSocket::autoGraspCommand(){
	std::vector <Robot *> rb;
	readRobotIndList(rb);
	bool render((strPtr++)->toInt());
	bool succeeded(true);
	for(std::vector<Robot *>::iterator rp = rb.begin(); rp != rb.end(); ++rp){
		//attempt to cast to hand
		if(Hand * h = dynamic_cast<Hand *>(*rp))
			if(!h->autoGrasp(render, 1.0, false))
				succeeded = false;
	}
	return succeeded;
}

bool ClientSocket::approachToContactCommand(){
	std::vector <Robot *> rb;
	readRobotIndList(rb);
	double moveDist((strPtr++)->toDouble());
	bool succeeded(true);
	for(std::vector<Robot *>::iterator rp = rb.begin(); rp != rb.end(); ++rp){
		//attempt to cast to hand
		if(Hand * h = dynamic_cast<Hand *>(*rp))
			if (!h->approachToContact(moveDist))
				succeeded=false;
	}
	return succeeded;
}


void ClientSocket::getRobotVertices(std::vector<Robot *> & rl){
	std::vector<Body *> bd;
	for (std::vector<Robot *>::iterator r = rl.begin(); r != rl.end(); ++r){
		for(int k = 0; k < (*r)->getNumChains();++k)
			for(int l =0; l<(*r)->getChain(k)->getNumLinks(); l++)
				bd.push_back((*r)->getChain(k)->getLink(l));
		bd.push_back((*r)->getBase());
		getBodyVertices(bd);
	}
}

void ClientSocket::getBodyVertices(std::vector<Body *> & b){
	QTextStream os(this);
	//iterate over all chain, iterate over all links and output triangle list
	std::vector<position> vertices;
	for(std::vector<Body *>::iterator bi = b.begin(); bi < b.end(); ++bi)//{
		(*bi)->getGeometryVertices( &vertices);
	//    for (std::vector<position>::iterator p= localVertices.begin(); p != localVertices.end(); ++p)
	//      *p = (*p)*(*bi)->getTran();


	//transform and copy over
	// vertices.insert(vertices.end(), localVertices.begin(), localVertices.end());
	//}
	for (std::vector<position>::iterator s = vertices.begin(); s != vertices.end(); ++s){
		os << (*s)[0] << " " <<(*s)[1] << " " << (*s)[2] << "\n";
	}
}

void ClientSocket::estimateStability()
{
	if(!mHandControlAPI)
		mHandControlAPI = new HandControlAPI();
	if(!mQualityEstimator)
		mQualityEstimator = new QualityEstimator();

	mHandControlAPI->importHand();

	std::vector<double> joints;
	std::vector<double> sensors;
	joints.clear();
	sensors.clear();
	std::string modelPath,codebookPath;

	strPtr ++;
	while (strPtr !=lineStrList.end()){
		if (*strPtr == "--jointValues"){
			//get joint values for the hand
			strPtr ++;
			int numJoints = (*strPtr).toInt();
			for(int i = 0; i < numJoints; ++i)
			{
				strPtr ++;
				joints.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--tactile_readings"){
			//get the tactile sensor readings
			strPtr ++;
			int numSensors = (*strPtr).toInt();
			for(int i = 0; i < numSensors; ++i)
			{
				strPtr ++;
				sensors.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--model_path"){
			strPtr ++;
			modelPath = (*strPtr).toStdString();
		}
		else if(*strPtr == "--codebook_path"){
			strPtr ++;
			codebookPath = (*strPtr).toStdString();
		}
		else{
			strPtr ++;
		}
	}
	if(joints.size() > 0)
	{
		std::cout << "to set joint values" << std::endl;
		mHandControlAPI->setJointValues(joints);
	} else {
		std::cout << "no joint values detected" << std::endl;
	}

	if(sensors.size() > 0)
	{
		std::cout << "to set sensor values" << std::endl;
		mQualityEstimator->setTactileReadings(sensors);
	} else {
		std::cout << "no srnsor values detected" << std::endl;
	}
	//setup the svm model trained from libsvm
	mQualityEstimator->setModel(modelPath);
	//setup the path to the codebook and normalizer
	mQualityEstimator->setCodebook(codebookPath);

	mQualityEstimator->setTactileLocations(mHandControlAPI->getTactileLocations());

	double e;
	e =  mQualityEstimator->estimateStability();
	QString msg, val;
	val.setNum(e);
	msg += QString("{");
	msg += val;
	msg += QString("}");
	std::cout << "stability is: " << e << std::endl;
	feedback(msg);
	//mHandControlAPI->outputTactileLocations();
}

Body* bd = NULL;

void ClientSocket::queryExperience()
{
	QString object_name;
	//Initialize defaults to standard dbase
	QString dbURL("fiji.cs.columbia.edu");
	QString dbPassword("super");
	QString dbUserName("postgres");
	QString dbName("hao_grasps"); 
	QString dbPort("5432");

	//clear the world
	World *world = graspItGUI->getIVmgr()->getWorld();

	if(bd)
		graspItGUI->getIVmgr()->getWorld()->destroyElement(bd, true);


	if(!mHandControlAPI)
		mHandControlAPI = new HandControlAPI();

	if(!mGeb)
		mGeb = new GraspExperienceBase();

	mHandControlAPI->importHand();

	std::vector<double> joints;
	std::vector<double> sensors;
	joints.clear();
	sensors.clear();
	std::string experiencePath,posePath;
	//posePath = "/home/arm_user/rearm/columbia_stacks/RearmGraspit/experience/handpose.txt";

	strPtr ++;
	while (strPtr !=lineStrList.end()){
		if (*strPtr == "--jointValues"){
			//get joint values for the hand
			strPtr ++;
			int numJoints = (*strPtr).toInt();
			for(int i = 0; i < numJoints; ++i)
			{
				strPtr ++;
				joints.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--tactile_readings"){
			//get the tactile sensor readings
			strPtr ++;
			int numSensors = (*strPtr).toInt();
			for(int i = 0; i < numSensors; ++i)
			{
				strPtr ++;
				sensors.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--experience_path"){
			strPtr ++;
			experiencePath = (*strPtr).toStdString();
		}
		else if(*strPtr == "--pose_path"){
			strPtr ++;
			posePath = (*strPtr).toStdString();
		}
		else{
			strPtr ++;
		}
	}
	if(joints.size() > 0)
	{
		std::cout << "to set joint values" << std::endl;
		mHandControlAPI->setJointValues(joints);
	} else {
		std::cout << "no joint values detected" << std::endl;
	}

	if(sensors.size() > 0)
	{
		std::cout << "to set sensor values" << std::endl;
	} else {
		std::cout << "no srnsor values detected" << std::endl;
	}

	if( mGeb->loadGraspExperienceBase(experiencePath) )
	{
		std::cout << "Experience data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Experience data base not loaded" << std::endl;
	}

	if( mGeb->loadHandPoses(posePath) )
	{
		std::cout << "Pose data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Pose data base not loaded" << std::endl;
	}

	//return;
	GraspExperienceEntry queryGrasp;
	queryGrasp.setContactList(getContactInfoList(sensors));
	queryGrasp.setSpread(getCurrentHandSpread());
	//queryGrasp.printMe();
	//mGeb->getEntry(0).printMe();
	std::vector<GraspExperienceEntry> queryGraspNNList;
	queryGraspNNList = mGeb->getKNNBasedOnTactile(queryGrasp);
	std::cout << "distance to NN is: " << queryGraspNNList[0].getDistance() << std::endl;
	//return;

	//examine the quality
	if(!mQualityEstimator)
		mQualityEstimator = new QualityEstimator();
	if(sensors.size() > 0)
	{
		std::cout << "to set sensor values" << std::endl;
		mQualityEstimator->setTactileReadings(sensors);
	} else {
		std::cout << "no srnsor values detected" << std::endl;
	}
	//setup the svm model trained from libsvm
	mQualityEstimator->setModel("modeli00");
	//setup the path to the codebook and normalizer
	mQualityEstimator->setCodebook("codebooki00.txt");
	//setup the current grasp tactile sensing data
	mQualityEstimator->setTactileLocations(mHandControlAPI->getTactileLocations());

	double e;
	e =  mQualityEstimator->estimateStability();
	if(e > 0.5)
	{
		std::cout << "positive quality" << std::endl;
		return;
	}
	//done estimating the quality

#define VERSION_PRECOMP
#ifdef VERSION_CGDB //use the traditional style, computing perturbation online
	/*
	GraspExperienceEntry gee;
	gee.setContactList(getCurrentGraspContactInfoList());
	gee.setSpread(getCurrentHandSpread());
	*/
	HandAdjust ha;
	ha.connectToDB(dbURL, dbName, dbPort, dbUserName, dbPassword);
	ha.init(mGeb);
	ha.setTactileNNList(queryGrasp, queryGraspNNList);
	double spread;
	transf t = ha.getHandAdjustmentONLINE(&spread);
#endif

#ifdef VERSION_PRECOMP //use precomputed perturbation
	if(queryGraspNNList[0].getDistance() < 10)
	{
		std::cout << "very close to the grasp, may be good enough" << std::endl;
		return;
	}
	else if(queryGraspNNList[0].getDistance() > 40)
	{
		std::cout << "very far from the nn grasp, need local exploration" << std::endl;
		return;
	}

	HandAdjust ha;
	ha.init(mGeb);
	double spread;
	transf t = ha.getHandAdjustmentUsePrecomputedPerturbation(queryGrasp, &spread);
#endif //VERSION_PRECOMP

	//turn off the command activity
	world->setCommandActive(false);

	std::stringstream msg;
	msg << "A [-1, -1, " <<
		//pregrasp position
		t.translation().x() << ", " << 
		t.translation().y() << ", " << 
		t.translation().z() << ", " << 
		t.rotation().w << ", " <<
		t.rotation().x << ", " <<
		t.rotation().y << ", " <<
		t.rotation().z << ", " <<
		//pregrasp dof
		spread << ",0,0,0" << ", " <<
		//source
		"3, " <<
		// group id
		"1" <<
		"]" <<
		" [-1, -1, " <<
		//finalgrasp position
		t.translation().x() << ", " << 
		t.translation().y() << ", " << 
		t.translation().z() << ", " << 
		t.rotation().w << ", " <<
		t.rotation().x << ", " <<
		t.rotation().y << ", " <<
		t.rotation().z << ", " <<
		//finalgrasp dof
		spread << ", " << 
		0 << ", " <<
		0 << ", " <<
		0 << "]";
	world->setCommand(msg.str());
	world->setCommandActive(true);
	std::cout << "msg defined: " << msg.str() << std::endl;
}

void ClientSocket::retrieveCommand()
{
	World *world = graspItGUI->getIVmgr()->getWorld();
	std::string cmd = world->getCommand();
	QString msg;
	msg += QString("{");
	if(!cmd.empty() && world->isCommandACtive())
	{
		msg += QString(cmd.c_str());
		world->resetCommand();
	}
	msg += QString("}");
	feedback(msg);
}

#ifdef LOCAL_EXPLORE_ENABLED
void ClientSocket::startExplore()
{
	if(!mLocalExplore)
		mLocalExplore = new LocalExplore();
	mLocalExplore->startExplore();
	std::cout << "start explore" << std::endl;
}

void ClientSocket::addPointCloud()
{
	FILE * fp = fopen("commands.txt", "a");
	fprintf(fp, "%s\n", line.ascii());
	fclose(fp);
	mHandControlAPI->importHand();

	std::vector<double> joints;
	std::vector<double> sensors;
	joints.clear();
	sensors.clear();
	transf offsetTran;

	strPtr ++;
	while (strPtr !=lineStrList.end()){
		if (*strPtr == "--jointValues"){
			//get joint values for the hand
			strPtr ++;
			int numJoints = (*strPtr).toInt();
			for(int i = 0; i < numJoints; ++i)
			{
				strPtr ++;
				joints.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--tactile_readings"){
			//get the tactile sensor readings
			strPtr ++;
			int numSensors = (*strPtr).toInt();
			for(int i = 0; i < numSensors; ++i)
			{
				strPtr ++;
				sensors.push_back((*strPtr).toDouble());
			}
		}
		else if(*strPtr == "--offset"){
			//get offset transform: x,y,z, qw, qx, qy, qz
			strPtr ++;
			readTransf(&offsetTran);
		}
		else{
			strPtr ++;
		}
	}
	if(joints.size() > 0)
	{
		std::cout << "to set joint values" << std::endl;
		mHandControlAPI->setJointValues(joints);
	} else {
		std::cout << "no joint values detected" << std::endl;
	}

	if(sensors.size() > 0)
	{
		std::cout << "to set sensor values" << std::endl;
	} else {
		std::cout << "no srnsor values detected" << std::endl;
	}

	//mLocalExplore->addPointCloud(getContactInfoList(sensors), offsetTran);
	std::vector<ContactInfo> palm, f1, f2, f3;
	getContactInfoListPerPad(sensors, palm, f1, f2, f3);
	mLocalExplore->addPointCloud(getContactWithLargestNormalForce(palm),
		getContactWithLargestNormalForce(f1),
		getContactWithLargestNormalForce(f2),
		getContactWithLargestNormalForce(f3),
		offsetTran);

}

void ClientSocket::endExplore()
{
	std::cout << "end explore" << std::endl;
	mLocalExplore->endExplore();
}
#endif

/*!
This is not complete yet.

void
ClientSocket::readTorques()
{
QString line;
line = readLine();
int numDOF = line.toInt();

if (numDOF < 0) {} //unused parameter warning
}
*/

/*!
Starts a TCP server that listens on port \a port.  \a backlog specifies
the number of pending connections the server can have.
*/
GraspItServer::GraspItServer(Q_UINT16 port, int backlog,
							 QObject *parent,const char *name) :
Q3ServerSocket(port,backlog,parent,name)
{
	if (!ok()) {
		qWarning("Failed to bind to port");
	}
}

/*! 
Creates a new ClientSocket to handle communication with this client.
*/
void
GraspItServer::newConnection(int socket)
{
	(void)new ClientSocket(socket, this);

#ifdef GRASPITDBG
	std::cout << "new connection" << std::endl;
#endif
}


