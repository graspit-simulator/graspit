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
// Author(s): Matei T. Ciocarlie
//
// $Id: application.h,v 1.1 2010/08/11 21:34:54 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the interface for a plugin that can be loaded dynamically and used with GraspIt
*/

#include <string>

//! Defines a plugin that can can be loaded dynamically and used with GraspIt
class Plugin
{
public:
  //! Stub destructor
  virtual ~Plugin(){}
  //! Called once when user starts the plugin (or on startup if the plugin is automatically started)
  virtual int init(int argc, char **argv) = 0;
  //! Called whenever GraspIt's main loop is idle
  virtual int mainLoop() = 0;
};

class PluginCreator
{
public:
  typedef Plugin* (*CreatePluginFctn)();
  typedef std::string (*GetTypeFctn)();
private:
  void* mLibraryHandle;
  CreatePluginFctn mCreatePluginFctn;

  bool mAutoStart;
  std::string mType;
  std::string mDefaultArgs;
public:
  PluginCreator(void* libraryHandle, CreatePluginFctn createPluginFctn,
                bool autoStart, std::string type, std::string defaultArgs) : 
    mLibraryHandle(libraryHandle),
    mCreatePluginFctn(createPluginFctn),
    mAutoStart(autoStart),
    mType(type),
    mDefaultArgs(defaultArgs) 
  {}
  
  ~PluginCreator();

  Plugin* createPlugin(std::string args);
  
  bool autoStart() const {return mAutoStart;}
  std::string type() const {return mType;}
  std::string defaultArgs() const {return mDefaultArgs;}

  static PluginCreator* loadFromLibrary(std::string libName);
};
