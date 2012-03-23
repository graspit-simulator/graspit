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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: worldElement.h,v 1.17 2009/06/18 20:41:02 saint Exp $
//
//######################################################################

#include "worldElementFactory.h"

/* Since we are using a class down here to register all the built in types
   at runtime, we have to include all relevant headers here. Maybe we can
   find a better solution at some point.
*/
#include "robot.h"
#include "body.h"
#include "robot.h"
#include "humanHand.h"
#include "robonaut.h"
#include "pr2Gripper.h"
#include "m7.h"
#include "m7tool.h"
#include "barrett.h"
#include "shadow.h"
#include "puma560.h"
#include "mcGrip.h"
#include "robotiq.h"

WorldElementFactory::~WorldElementFactory()
{
  for (std::map<std::string, WorldElementCreator*>::iterator it=mCreators.begin(); it!=mCreators.end(); it++)
  {
    delete it->second;
  }
}

WorldElement* 
WorldElementFactory::createElement(std::string elementType, World *parent,const char *name)
{
  std::map<std::string, WorldElementCreator*>::iterator it = mCreators.find(elementType);
  if (it==mCreators.end()) return NULL;
  return (*(it->second))(parent, name);
}

void 
WorldElementFactory::registerCreator(std::string elementType, WorldElementCreator *creator)
{
  mCreators[elementType] = creator;
}

void 
WorldElementFactory::registerBuiltinCreators()
{
  REGISTER_CREATOR("Body",Body);
  REGISTER_CREATOR("GraspableBody",GraspableBody);
  REGISTER_CREATOR("Robot",Robot);
  REGISTER_CREATOR("Hand",Hand);
  REGISTER_CREATOR("Puma560",Puma560);
  REGISTER_CREATOR("Barrett",Barrett);
  REGISTER_CREATOR("Robonaut",Robonaut);
  REGISTER_CREATOR("Pr2Gripper",Pr2Gripper);
  REGISTER_CREATOR("Pr2Gripper2010",Pr2Gripper2010);
  REGISTER_CREATOR("M7",M7);
  REGISTER_CREATOR("M7Tool",M7Tool);
  REGISTER_CREATOR("HumanHand",HumanHand);
  REGISTER_CREATOR("Shadow",Shadow);
  REGISTER_CREATOR("McGrip",McGrip);
  REGISTER_CREATOR("RobotIQ",RobotIQ);
}
