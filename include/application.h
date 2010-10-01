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
// $Id: application.h,v 1.1 2010/08/11 21:34:54 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines an abstract application that can be started inside GraspIt's event loop
*/

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <string>

//! An abstract application that gets called from inside GraspIt's event loop
/*! At start time, an application can be initialized and registered with GraspIt's main
  event loop, which will then call the application's mainLoop() whenever GraspIt is 
  idle.
 */
class Application
{
public:
  //! Called at start time. Must surrender control when init is done
  virtual int init(int argc, char **argv) = 0;

  //! Called whenever GraspIt's main event loop is idle. 
  /*! Should surrender control back to graspit when it's processing is done */
  virtual void mainLoop() = 0;

  //! Creates an instance of an application based on the application's name
  static Application* createApplication(std::string appName);
};

#endif
