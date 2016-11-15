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
// $Id: graspitGUI.h,v 1.5 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines a graspit user interface class that contains subpieces of the UI.
*/

#ifndef GRASPITGUI_HXX

#include <string>
#include <vector>
#include <list>

class MainWindow;
class IVmgr;
class TaskDispatcher;
class Application;
class Plugin;
class PluginCreator;
class SoIdleSensor;
class SoSensor;

//! This is the main user interface class that is responsible for creating and destroying the MainWindow and IVmgr.
/*!
  This class also processes command line arguments and holds pointers to both
  the MainWindow and IVmgr.  There is one global instance of this class, which
  allows access to these two main pieces of the UI.  This class also has
  methods for both the entry and exit to the interactive program loop.

  This class can also initialize a task dispatcher which is then in charge of 
  batch execution of tasks based on information form a grasp database.
*/
class GraspItGUI
{
  //! A pointer to the MainWindow.
  MainWindow *mainWindow;

  //! A pointer to the IVmgr.
  IVmgr *ivmgr;

  //! A pointer to the Task Dispatcher, if any
  TaskDispatcher *mDispatch;

  //! TRUE if this class has been initialized.
  static bool initialized;

  //! Holds result of UI initialization.
  static int initResult;

  //! Holds the exit code of the UI execution
  int mExitCode;
  
  //! Plugins currently running
  std::list< std::pair<Plugin*,std::string> > mActivePlugins;

  //! Available plugin creators
  std::vector<PluginCreator*> mPluginCreators;

  //! Idle sensor for calling the plugins from GraspIt's event loop
  SoIdleSensor *mPluginSensor;

 protected:
  int processArgs(int argc, char **argv);

 public:
  GraspItGUI(int argc,char **argv);
  ~GraspItGUI();
  
  /*! Returns whether the UI pieces were successfully initialized. */
  bool terminalFailure() const;

  //! Returns the exit code (set internally based on the application)
  int getExitCode() const {return mExitCode;}

  /*! Returns a pointer to the MainWindow. */
  MainWindow *getMainWindow() const {return mainWindow;}

  /*! Returns a pointer to the IVmgr. */
  IVmgr *getIVmgr() const {return ivmgr;}

  //! Static sensor callback, just calls processPlugins()
  static void sensorCB(void *data, SoSensor*);
  
  //! Calls the main processing routine of all active plugins
  void processPlugins();

  //! Starts a plugin from the given creator
  void startPlugin(PluginCreator* creator, std::string args);

  //! Stops and deletes the specified plugin
  void stopPlugin(Plugin *plugin);

  //! Stops and deletes all currently active plugins
  void stopAllPlugins();

  void startMainLoop();
  void exitMainLoop();
};

#ifdef WIN32
#ifdef GRASPIT_EXPORTS
#define GRASPIT_API __declspec(dllexport)
#else
#define GRASPIT_API __declspec(dllimport)
#endif
#else
#define GRASPIT_API
#endif

extern GRASPIT_API GraspItGUI *graspItGUI;

#define GRASPITGUI_HXX
#endif
