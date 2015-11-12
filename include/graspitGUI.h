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
// $Id: graspitGUI.h,v 1.4 2009/10/08 16:21:15 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines a graspit user interface class that contains subpieces of the UI.
*/

#ifndef GRASPITGUI_HXX
class MainWindow;
class IVmgr;
class TaskDispatcher;

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

  bool isConsole;
  
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
  
  bool useConsole(){return isConsole;}

  void startMainLoop();
  void exitMainLoop();
};

extern GraspItGUI *graspItGUI;

#define GRASPITGUI_HXX
#endif
