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
// $Id: main.cpp,v 1.15 2009/10/08 16:18:56 cmatei Exp $
//
//######################################################################

/*! \mainpage GraspIt! Developer Documentation
  \image html logo.jpg

  These pages document the GraspIt! source code. Please remember this is
  research code. There are still plenty of pieces of code that are unfinished
  and several bugs that need to be fixed.

  More information and original source code for the packages included with
  GraspIt! can be found in the following places:

  - <b>qhull:</b> http://www.qhull.org
  - <b>maxdet:</b> http://www.stanford.edu/~boyd/old_software/MAXDET.html
  - <b>tinyxml:</b> http://www.grinninglizard.com/tinyxml/
*/

/*! \file
  \brief Program execution starts here.  Server is started, main window is built, 
  and the interactive loop is started.

  The main call returns an exit code as indicated by the graspit GUI (0 by default)
  to provide feedback to calling program, if desired.
 */

#define GRASPITDBG

#include <iostream>
#include <graspitApp.h>
#include "graspitGUI.h"
#include "graspitServer.h"
#include "mainWindow.h"

#ifdef Q_WS_WIN
#include <windows.h>
#include <wincon.h>
#endif

int main(int argc, char **argv)
{
#ifdef GRASPITDBG
#ifdef Q_WS_WIN
  AllocConsole(); 
  freopen("conin$", "r", stdin); 
  freopen("conout$", "w", stdout); 
  freopen("conout$", "w", stderr); 
  //ios::sync_with_stdio();
#endif
#endif

  GraspItApp app(argc, argv);
 
//  app.showSplash();
//  QApplication::setOverrideCursor( Qt::waitCursor );

  GraspItGUI gui(argc,argv);
  
  //This is the GraspIt TCP server. It can be used to connect to GraspIt from
  //external programs, such as Matlab.
  //On some machines, the Q3Socket segfaults at exit, so this is commented out by
  //default

  /*GraspItServer's constructor uses the argv to look for command line input of 
   *the port number for the graspit server.  This is necessary so that several
   *instantiations of graspit servers can run at the same time on many core machines 
   *rather than dealing with the horror of making the whole collision system 
   *multithreaded to an arbitrary degree.
   */

  //GraspItServer server(GraspItServerTools::parseArgs(argc,argv));
  GraspItServer server(4766);
 
//  app.setMainWidget(gui.getMainWindow()->mWindow);
//  QObject::connect(qApp, SIGNAL(lastWindowClosed()), qApp, SLOT(quit()));

//  app.closeSplash();
//  QApplication::restoreOverrideCursor();

  if (!gui.terminalFailure()) {
	  gui.startMainLoop();
  }
  if (gui.useConsole())
    app.exec();

  return gui.getExitCode();
}
