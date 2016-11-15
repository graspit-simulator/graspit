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

#ifdef WIN32
#include "dlfcn-win32.h"
#define LIBRARY_SUFFIX ".dll"
#else
extern "C"{
#include <dlfcn.h>
}
#define LIBRARY_SUFFIX ".so"
#endif

#include <QFile>

#include "plugin.h"
#include "mytools.h"
#include "debug.h"

PluginCreator::~PluginCreator()
{
  dlclose(mLibraryHandle);
}

Plugin* PluginCreator::createPlugin(std::string args)
{	
  Plugin* plugin = (*mCreatePluginFctn)(); 
  args = args;
  if (!plugin) return NULL;
  if (plugin->init(0,NULL) != SUCCESS) {
    DBGA("Failed to initialize new plugin of type " << mType);
    delete plugin;
    return NULL;
  }
  return plugin;
}

PluginCreator* PluginCreator::loadFromLibrary(std::string libName)
{
  QString filename = QString::fromStdString(libName);
  //append library suffix if it does not already have it
  if (!filename.endsWith(LIBRARY_SUFFIX)) {
    filename.append(LIBRARY_SUFFIX);
  }

  if (filename.startsWith("/")) {
    //filename is an absolute path
    QFile file(filename);
    if (!file.exists()) {
      DBGA("Could not find absolute plugin file " << filename.latin1());
      return NULL;
    }
  } else{
    //filename is relative to GRASPIT_PLUGIN_DIR
    QString pluginDirs = QString(getenv("GRASPIT_PLUGIN_DIR"));
    if (pluginDirs.isNull()) {
      DBGA("Relative plugin file specified, but GRASPIT_PLUGIN_DIR is not set");
      return NULL;
    }
    bool found = false;
    for (int i=0; i<=pluginDirs.count(","); i++) {
      QString dir = pluginDirs.section(',',i,i);
      if (!dir.endsWith("/")) dir.append("/");
      dir.append(filename);
      QFile file(dir);
      if (file.exists()) {
        found = true;
        filename = dir;
        break;
      }
    }
    if (!found) {
      DBGA("Could not find relative plugin file " << filename.latin1() << 
           " in any directory specified in GRASPIT_PLUGIN_DIR");
      return NULL;
    }    
  }

  //look for the library file and load it
   void* handle = dlopen(filename.toAscii().constData(), RTLD_NOW | RTLD_GLOBAL);
  char *errstr = dlerror();
  if (!handle) {
    DBGA("Failed to open dynamic library " << filename.toAscii().constData() );
    if (errstr) DBGA("Error: " << errstr);
    return NULL;
  }

  //instantiate the plugin inside of the library
  //this is pretty bad, but is based on example here:
  // http://pubs.opengroup.org/onlinepubs/009695399/functions/dlsym.html
  //see also discussion here:
  // http://www.trilithium.com/johan/2004/12/problem-with-dlsym/
  //maybe in the future a better solution can be found...
  PluginCreator::CreatePluginFctn createPluginFctn;
  *(void **)(&createPluginFctn) = dlsym(handle,"createPlugin");
  if (dlerror()) {
    DBGA("Could not load symbol createPlugin from library " << filename.toAscii().constData());
    return NULL;
  }

  //read the type of plugin
  PluginCreator::GetTypeFctn getTypeFctn;
  *(void **)(&getTypeFctn) = dlsym(handle,"getType");
  if (dlerror()) {
    DBGA("Could not load symbol getType from library " << filename.toAscii().constData());
    return NULL;
  }
  std::cout << "Function name " << (*getTypeFctn)() <<std::endl;
  std::string type = (*getTypeFctn)();
  if (type.empty()) {
    DBGA("Could not get plugin type from library " << filename.toAscii().constData());
  }

  //create the PluginCreator and push it back
  bool autoStart = true;
  std::string defaultArgs;
  PluginCreator *creator = new PluginCreator(handle, createPluginFctn, autoStart, type, defaultArgs);
  return creator;
}
