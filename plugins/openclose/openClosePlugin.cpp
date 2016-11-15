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
// Author(s):  Matei T. Ciocarlie
//
// $Id: taskDispatcher.h,v 1.3 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

#include "openClosePlugin.h"

#include "mainWindow.h"
#include "graspitGUI.h"
#include "world.h"
#include "robot.h"

#include <iostream>

namespace openclose {

OpenClosePlugin::OpenClosePlugin()
{
  std::cerr << "Open-close plugin created\n";
}

OpenClosePlugin::~OpenClosePlugin()
{
  std::cerr << "Open-close plugin destroyed\n";
}

int OpenClosePlugin::init(int, char**)
{
  std::cerr << "Open-close plugin initialized\n";
  return 0;
}

int OpenClosePlugin::mainLoop()
{
  static int direction = 1.0;

  World *world = graspItGUI->getMainWindow()->getMainWorld();
  if (!world) std::cerr << "Open-close plugin main loop: no world?!?\n";
  else if (!world->getCurrentHand()) std::cerr << "Open-close plugin main loop: no hand selected\n";
  else world->getCurrentHand()->autoGrasp(true, direction);
  direction *= -1;
  return 0;
}

}

extern "C" PLUGIN_API
Plugin* createPlugin() {
  return new openclose::OpenClosePlugin();
}

extern "C" PLUGIN_API
std::string getType() {
  return "open-close";
}
