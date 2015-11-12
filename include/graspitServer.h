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
// Author(s):  Andrew T. Miller 
//
// $Id: graspitServer.h,v 1.3 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the GraspItServer and the ClientSocket classes
 */

#ifndef GRASPITSERVER_HXX
#include <q3serversocket.h>
#include <q3socket.h>
#include <qstringlist.h>
#include <iostream>
#include <vector>
#include "matvec3D.h"

class Body;
class Robot;
class GraspPlanningService;
class HandControlAPI;
class QualityEstimator;
class GraspExperienceBase;
#ifdef LOCAL_EXPLORE_ENABLED
class LocalExplore;
#endif

//! Subclass of QSocket that parses input, implements commands, and writes output
/*!
  A ClientSocket is created whenever the GraspitServer accepts a new
  connection.  Whenever there is data ready to be read, it calls the
  readClient method, which parses the text command and calls the appropriate
  method.  Some of these methods write results back to the socket.  When
  it receives a connection closed signal the ClientSocekt deletes itself.

  The following commands are currently recognized and processed:

  - \p getContacts   numData   bodyIndexList
  - \p getAverageContacts   bodyIndexList
  - \p getBodyName   bodyIndexList
  - \p getRobotName   robotIndexList
  - \p getDOFVals   robotIndexList
  - \p moveDOFs   robotIndexList
  - \p render
  - \p moveDynamicBodies   timeStepLength
  - \p computeNewVelocities   timeStepLength

  Each command requiring an index list, accept the word "ALL" meaning every
  body or robot in the world, or a number of bodies or robots followed by a
  space sparated list of the body or robot indicies.

  getContacts also requires numData, which specifies the number of vectors
  of data that should be returned (1, 2 or 3) about each contact.

  The routines will send back an error message if there is a problem with
  the input.

*/
class ClientSocket : public Q3Socket
{
  Q_OBJECT
    
public:

  /*! 
    Connects the readyRead signal to the readClient slot, and the
    connectionClosed signal to the connectionClosed slot.  Sets the socket
    to use \a sock .
  */
  ClientSocket( int sock, QObject *parent=0, const char *name=0 ) :
    Q3Socket( parent, name )//, mService(NULL)
    {
      connect( this, SIGNAL(readyRead()), SLOT(readClient()) );
      connect( this, SIGNAL(connectionClosed()), SLOT(connectionClosed()) );
      setSocket( sock );
    }
  
  ~ClientSocket();
  
private:

  //! The current line of text read from the socket
  QString line;
  
  //! The list of strings after splitting the line at each space character
  QStringList lineStrList;

  //! An iterator into the string list
  QStringList::const_iterator strPtr;

  //! An instance to the service
  
  static GraspPlanningService* mService;
  static HandControlAPI* mHandControlAPI;
  static QualityEstimator* mQualityEstimator;
  static GraspExperienceBase* mGeb;
#ifdef LOCAL_EXPLORE_ENABLED
  static LocalExplore* mLocalExplore;
#endif

  int readBodyIndList(std::vector<Body *> &bodyVec);
  int readRobotIndList(std::vector<Robot *> &robVec);
  void sendContacts(Body *bod,int numData);
  void sendAverageContacts(Body *bod);
  void sendBodyName(Body* bod);
  void computeNewVelocities(double ts);
  void moveDynamicBodies(double ts);

  void sendRobotName(Robot* rob);
  void sendDOFVals(Robot *rob);

  int readDOFVals();
  int readDOFForces(Robot *rob);


  //for ros communication
  void planGrasp();
  void feedback(QString msg);
  // not finished yet:
  // void readTorques();
  // void moveBody(Body* bod); 
  int readTransf(transf * tr);
  void sendBodyTransf();
  void setBodyTransf();
  void sendRobotTransf();
  void setRobotTransf();
  bool autoGraspCommand();
  bool approachToContactCommand();
  void estimateStability();
  void queryExperience();
  int readTransfList(std::vector<transf>& transfList);//a TransfList is enclosed by two brackets '[' and ']'
  //assume there is a command buffer inside graspit, a ros node talks to Graspit and queries for any available command to execute
  void retrieveCommand();

  //related to locally explore the surface of the object
#ifdef LOCAL_EXPLORE_ENABLED
  void startExplore();
  void addPointCloud();
  void endExplore();
#endif


  void getBodyVertices(std::vector<Body *> & b);
  void getRobotVertices(std::vector<Robot *> & rl);
  
private slots:
  void readClient();

/*! Deletes this instance of ClientSocket */ 
  void connectionClosed() { delete this;}

};

//! TCP server that listens for connections and spawns new ClientSockets 
/*!
  The server is a subclass of the QT QServerSocket and listens on a particular
  port and if a connection is requested, it creates a new ClientSocket, which
  will handle all communication.
*/
class GraspItServer : public Q3ServerSocket
{
  Q_OBJECT
  //  std::vector<SocketNotifier *> snVec;

 public:
  GraspItServer(Q_UINT16 port, int backlog = 1, QObject *parent = 0,
		const char *name = 0);

  /*! Stub */
  ~GraspItServer() {}
  void newConnection(int socket);
};
#define GRASPITSERVER_HXX
#endif
