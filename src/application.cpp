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
// $Id: application.cpp,v 1.1 2010/08/11 21:34:54 cmatei Exp $
//
//######################################################################

#include "application.h"
#include "debug.h"

//include here the relevant headers for your applications
#ifdef ROS_INTERFACE_NODE
#include <graspit_interface/ros_graspit_interface.h>
#endif

//add here the instantiation for your applications
Application* Application::createApplication(std::string appName)
{
  if (appName == "ros_interface_node") {
#ifdef ROS_INTERFACE_NODE
    return new graspit_interface::RosGraspitInterface();
#else
    DBGA("Ros interface node is not enabled; edit graspit.pro to include it");
    return NULL;
#endif
  } else {
    DBGA("Unrecognized application name: " << appName);
  }
  return NULL;
}
