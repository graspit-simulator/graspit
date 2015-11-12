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
// $Id: graspitApp.h,v 1.3 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines GraspItApp, a subclass of QApplication.
*/

#ifndef _GRASPITAPP_H_
#include <qapplication.h>
//Added by qt3to4:
#include <QLabel>

class QLabel;

//! This is the specific QApplication class for this program, and it defines routines for the splash screen.
/*!
  The main purpose of this class is to show the splash screen at the
  beginning when the application is started.  In the future, more may be
  added to this.  One instance of this class should be defined in the main
  program.
*/
class GraspItApp : public QApplication
{
 public:
  /*! Stub constructor */
  GraspItApp(int &argc, char **argv) : QApplication(argc,argv) {}

  /*! Returns the name of this class. */
  const char *className() const { return "GraspItApp"; }

  static void showSplash();
  static void closeSplash();

};
#define _GRASPITAPP_H_
#endif
