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

#include <QFile>

#include "graspit/plugin.h"
#include "graspit/mytools.h"
#include "graspit/debug.h"
#include "string.h"



#ifdef GRASPIT_USE_WIN_DYNLIB
#include <windows.h>

#define PLUGIN_DYNLIB_OPEN    LoadLibrary
#define PLUGIN_DYNLIB_CLOSE   FreeLibrary
#define PLUGIN_DYNLIB_IMPORT  GetProcAddress
#define LIBRARY_SUFFIX ".dll"

static char *plugin_dynlib_error(void)
{
  static char buf[32];
  DWORD dw = GetLastError();
  if (dw == 0) { return NULL; }
  sprintf(buf, "error 0x%x", (unsigned int)dw);
  return buf;
}

#define PLUGIN_DYNLIB_ERROR plugin_dynlib_error

#else // GRASPIT_USE_WIN_DYNLIB
// extern "C"{  // it seems extern C is not needed (any more?)
#include <dlfcn.h>
// }
#define PLUGIN_DYNLIB_OPEN(path)  dlopen(path, RTLD_NOW | RTLD_GLOBAL)
#define PLUGIN_DYNLIB_CLOSE       dlclose
#define PLUGIN_DYNLIB_IMPORT      dlsym

#define PLUGIN_DYNLIB_ERROR dlerror

#define LIBRARY_SUFFIX ".so"
#endif // GRASPIT_USE_WIN_DYNLIB








PluginCreator::~PluginCreator()
{
  PLUGIN_DYNLIB_CLOSE(mLibraryHandle);
}


Plugin *PluginCreator::createPlugin(int argc, char **argv)
{
  Plugin *plugin = (*mCreatePluginFctn)();
  if (!plugin)
  {
    return NULL;
  }

  //make copy of argv, so plugins cannot interfere with each other.
  char **argv_copy = new char*[argc + 1];
  for (int i = 0; i < argc; i++)
  {
    argv_copy[i] = strdup(argv[i]);
  }
  argv_copy[argc] = NULL;

  if (plugin->init(argc, argv_copy) != SUCCESS)
  {
    DBGA("Failed to initialize new plugin of type " << mType);
    delete plugin;
    return NULL;
  }
  return plugin;
}

PluginCreator *PluginCreator::loadFromLibrary(std::string libName)
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
  } else {
    //filename is relative to GRASPIT_PLUGIN_DIR
    QString pluginDirs = QString(getenv("GRASPIT_PLUGIN_DIR"));
    if (pluginDirs.isNull()) {
      DBGA("Relative plugin file specified, but GRASPIT_PLUGIN_DIR is not set");
      return NULL;
    }
    bool found = false;
    for (int i = 0; i <= pluginDirs.count(","); i++) {
      QString dir = pluginDirs.section(',', i, i);
      if (!dir.endsWith("/")) { dir.append("/"); }
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
  PLUGIN_DYNLIB_HANDLE handle = PLUGIN_DYNLIB_OPEN(filename.toAscii().constData());
  char *errstr = PLUGIN_DYNLIB_ERROR();
  if (!handle) {
    DBGA("Failed to open dynamic library " << filename.toAscii().constData());
    if (errstr) { DBGA("Error: " << errstr); }
    return NULL;
  }

  //instantiate the plugin inside of the library
  //this is pretty bad, but is based on example here:
  // http://pubs.opengroup.org/onlinepubs/009695399/functions/dlsym.html
  //see also discussion here:
  // http://www.trilithium.com/johan/2004/12/problem-with-dlsym/
  //maybe in the future a better solution can be found...
  PluginCreator::CreatePluginFctn _createPluginFctn = (CreatePluginFctn) PLUGIN_DYNLIB_IMPORT(handle, "createPlugin");
  if (PLUGIN_DYNLIB_ERROR()) {
    DBGA("Could not load symbol createPlugin from library " << filename.toAscii().constData());
    return NULL;
  }
  PluginCreator::CreatePluginFctn createPluginFctn = reinterpret_cast<PluginCreator::CreatePluginFctn>(_createPluginFctn);

  //read the type of plugin
  PluginCreator::GetTypeFctn _getTypeFctn = (GetTypeFctn) PLUGIN_DYNLIB_IMPORT(handle, "getType");
  if (PLUGIN_DYNLIB_ERROR()) {
    DBGA("Could not load symbol getType from library " << filename.toAscii().constData());
    return NULL;
  }
  PluginCreator::GetTypeFctn getTypeFctn = reinterpret_cast<PluginCreator::GetTypeFctn>(_getTypeFctn);

  std::cout << "Function name " << (*getTypeFctn)() << std::endl;
  std::string type = (*getTypeFctn)();
  if (type.empty()) {
    DBGA("Could not get plugin type from library " << filename.toAscii().constData());
  }

  //create the PluginCreator and push it back
  bool autoStart = true;
  PluginCreator *creator = new PluginCreator(handle, createPluginFctn, autoStart, type);
  return creator;
}
