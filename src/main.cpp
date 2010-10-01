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
// $Id: main.cpp,v 1.16 2010/01/13 23:09:30 cmatei Exp $
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
 
  if (app.splashEnabled()) {
    app.showSplash();
    QApplication::setOverrideCursor( Qt::waitCursor );
  }

  GraspItGUI gui(argc,argv);
  
  //This is the GraspIt TCP server. It can be used to connect to GraspIt from
  //external programs, such as Matlab.
  //On some machines, the Q3Socket segfaults at exit, so this is commented out by
  //default
  //GraspItServer server(4765);
 
  app.setMainWidget(gui.getMainWindow()->mWindow);
  QObject::connect(qApp, SIGNAL(lastWindowClosed()), qApp, SLOT(quit()));

  if (app.splashEnabled()) {
    app.closeSplash();
    QApplication::restoreOverrideCursor();
  }

  if (!gui.terminalFailure()) {
	  gui.startMainLoop();
  }
  return gui.getExitCode();
}
