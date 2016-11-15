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
// $Id: graspitApp.h,v 1.4 2010/01/13 23:08:10 cmatei Exp $
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
 private:
  bool splash_enabled_;
 public:
  /*! Stub constructor */
 GraspItApp(int &argc, char **argv) : QApplication(argc,argv), splash_enabled_(true) 
    {
      //for db_dispatch jobs, splash is disabled
      if (argc > 1 && !strcmp(argv[1],"db_dispatch")) {
	splash_enabled_ = false;
      }
    }

  /*! Returns the name of this class. */
  const char *className() const { return "GraspItApp"; }

  /*! Returns wether the splash is enabled or not */
  bool splashEnabled() const {return splash_enabled_;}

  static void showSplash();
  static void closeSplash();

};
#define _GRASPITAPP_H_
#endif
