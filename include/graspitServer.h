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

class Body;
class Robot;

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
    Q3Socket( parent, name )
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

  // not finished yet:
  // void readTorques();
  // void moveBody(Body* bod); 

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
