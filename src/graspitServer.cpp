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
    
    else if (*strPtr == "render")
      graspItGUI->getIVmgr()->getViewer()->render();
    
    else if (*strPtr == "setDOFForces") {
      strPtr++;
      if (readRobotIndList(robVec)) continue;
      numRobots = robVec.size();
      for (i=0;i<numRobots;i++)
	if (readDOFForces(robVec[i])==FAILURE) continue;
    }
    else if (*strPtr == "moveToContacts")
      graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->approachToContact(30, true);
 
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


