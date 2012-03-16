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

#ifndef _OPEN_CLOSE_PLUGIN_H_
#define _OPEN_CLOSE_PLUGIN_H_
#include "plugin.h"
namespace openclose {

class OpenClosePlugin : public Plugin
{
public:  
  OpenClosePlugin();
  ~OpenClosePlugin();
  int init(int argc, char **argv);
  int mainLoop();
};

}
#endif
