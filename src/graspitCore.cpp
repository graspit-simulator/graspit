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
// $Id: graspitGUI.cpp,v 1.19 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements core graspit functionality.
  Responsible for initializating and holding pointers to the following:
    world,
    mainWindow,
    Dbmgr

  This class is also responsible for managing the SOQT mainloop and managing
  all plugins.
*/

#include <Q3GroupBox>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <QLayout>

#ifdef Q_WS_X11
#include <unistd.h>
#endif

#include "graspit/cmdline/cmdline.h"
#include "graspit/graspitCore.h"
#include "mainWindow.h"
#include "graspit/ivmgr.h"
#include "graspit/world.h"
#include "graspit/mytools.h"
#include "graspit/SoComplexShape.h"
#include "graspit/SoArrow.h"
#include "graspit/SoTorquePointer.h"
#include "graspit/plugin.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/graspitParser.h"

#ifdef CGDB_ENABLED
#include "graspit/DBase/dbase_grasp.h"
#include "graspit/DBase/taskDispatcher.h"
#endif

#include "graspit/EGPlanner/energy/searchEnergyFactory.h"
int GraspitCore::initResult = SUCCESS;

//! This is the system wide pointer to the graspit user interface.
GraspitCore *graspitCore = 0;

/*!
  Deletes both the IVmgr and the MainWindow.
*/
GraspitCore::~GraspitCore()
{
  //clean up plugins and creators
  stopAllPlugins();
  for (size_t i = 0; i < mPluginCreators.size(); i++) {
    delete mPluginCreators[i];
  }

#ifdef CGDB_ENABLED
  if (mDispatch) {
    delete mDispatch;
  }
#endif
  delete ivmgr;
  delete mainWindow;
}

/*!
  Initializes GraspitCore based on the commandline arguments.
*/
GraspitCore::GraspitCore(int argc, char **argv):
  mDispatch(NULL),
  ivmgr(NULL),
  mainWindow(NULL),
  world(NULL)
{
  //Register all of the available built in searchEnergy functions for the EG planner
  SearchEnergyFactory::registerBuiltinCreators();

  GraspitParser *graspitParser = new GraspitParser();
  graspitParser->parseArgs(argc, argv);
  cmdline::parser *args = graspitParser->parseArgs(argc, argv);

  bool headless = args->exist("headless");

  if (headless) {
    if (argc < 1)
    {
      DBGA("At least 1 command line argument required");
      initResult = FAILURE;
      return;
    }
    // need to initialize with this init() instead of passing argc, argv
    // or it segfaults
    SoQt::init(argv[0], "SOQT");
  }
  else {
    mainWindow = new MainWindow;
    SoQt::init(mainWindow->mWindow);
  }

  //Initialize the world.  It has no parent,
  world = new World(NULL, //QObject parent
                    "mainWorld"); // World Name

  // initialize Inventor additions
  SoComplexShape::initClass();
  SoArrow::initClass();
  SoTorquePointer::initClass();

  //Do not initialize the IVmgr if we are running headless. ivmgr will stay NULL
  if (!headless) {
    ivmgr = new IVmgr(world, (QWidget *)mainWindow->mUI->viewerHolder, "myivmgr");
    ivmgr->getViewer()->getWidget()->setFocusPolicy(Qt::StrongFocus);
    world->setIVMgr(ivmgr);
  }

  graspitCore = this;
  mExitCode = 0;

  int errorFlag = 0;

  //Ensure that $GRASPIT is set.
  QString graspitRoot = QString(getenv("GRASPIT"));
  if (graspitRoot.isNull()) {
    std::cerr << "Please set the GRASPIT environment variable to the root directory of graspIt." << std::endl;
    initResult =  FAILURE;
    return;
  }

  if (args->exist("plugin"))
  {
    QString plugins_arg_string = QString::fromStdString(args->get<std::string>("plugin"));
    QStringList plugins_list = plugins_arg_string.split(',');
    for (int i = 0; i < plugins_list.size(); i++)
    {
      std::cout << "Loading Plugin: " << plugins_list.at(i).toStdString().c_str();
      PluginCreator *creator = PluginCreator::loadFromLibrary(plugins_list.at(i).toStdString());
      if (creator) {
        mPluginCreators.push_back(creator);
      } else {
        DBGA("Failed to load plugin: " << plugins_list.at(i).toStdString());
      }
    }
  }

  //start any plugins with auto start enabled
  mPluginSensor = new SoIdleSensor(GraspitCore::sensorCB, (void *)this);
  for (size_t i = 0; i < mPluginCreators.size(); i++)
  {
    std::cout << "plugin creator autostart " << mPluginCreators[i]->autoStart() << std::endl;
    if (mPluginCreators[i]->autoStart())
    {
      startPlugin(mPluginCreators[i], argc, argv);
    }
  }

  if (argc > 1) {
#ifdef CGDB_ENABLED
    if (!strcmp(argv[1], "dbase")) {
      DBaseBatchPlanner *dbp = new DBaseBatchPlanner(graspitCore->getIVmgr(), this);
      dbp->processArguments(argc, argv);
      dbp->startPlanner();
    }
    if (!strcmp(argv[1], "db_dispatch")) {
      mDispatch = new TaskDispatcher();
      if (mDispatch->connect("wgs36", 5432, "willow", "willow", "household_objects")) {
        std::cerr << "DBase dispatch failed to connect to database\n";
        initResult = TERMINAL_FAILURE;
        return;
      }
      std::cerr << "DBase dispatch connected to database\n";
      mDispatch->mainLoop();
      if (mDispatch->getStatus() != TaskDispatcher::RUNNING) {
        mExitCode = mDispatch->getStatus();
        initResult = TERMINAL_FAILURE;
        return;
      }
    }
#endif
  }

#ifdef Q_WS_X11

  if (args->exist("world"))
  {
    QString filename = graspitRoot + QString("/worlds/") + QString::fromStdString(args->get<std::string>("world")) + QString(".xml");
    if (world->load(filename) == FAILURE) {
      ++errorFlag;
    }
    else {
      if (!headless)
      {
        mainWindow->mUI->worldBox->setTitle(filename);
      }
    }
  }

  if (args->exist("robot"))
  {
    QString filename = graspitRoot +
                       QString("/models/robots/") +
                       QString::fromStdString(args->get<std::string>("robot")) +
                       QString("/") +
                       QString::fromStdString(args->get<std::string>("robot")) +
                       QString(".xml");
    if (world->importRobot(filename) == NULL) {
      ++errorFlag;
    }
  }

  if (args->exist("obstacle"))
  {
    QString filename = graspitRoot +
                       QString("/models/obstacles/") +
                       QString::fromStdString(args->get<std::string>("obstacle")) +
                       QString(".iv");

    if (!world->importBody("Body", filename))
    {
      ++errorFlag;
    }
  }

  if (args->exist("object"))
  {
    QString filename = graspitRoot +
                       QString("/models/objects/") +
                       QString::fromStdString(args->get<std::string>("object")) +
                       QString(".iv");

    if (!world->importBody("GraspableBody", filename))
    {
      ++errorFlag;
    }
  }

  mDBMgr = NULL;

  if (errorFlag)
  {
    std::cerr << "Failed to Parse args." << std::endl;
    initResult =  FAILURE;
    return;
  }

#endif // Q_WS_X11
  initResult =  SUCCESS;
  return;
}

/*!
  Starts the Qt event loop.  If using the user interface, this
  also shows the mainWindow and sets its size.
*/
void
GraspitCore::startMainLoop()
{
  if (ivmgr)
  {
    mainWindow->setMainWorld(world);
    SoQt::show(mainWindow->mWindow);
    mainWindow->mWindow->resize(QSize(1070, 937));
  }

  SoQt::mainLoop();
}

/*!
  Exits the Qt event loop.
*/
void
GraspitCore::exitMainLoop()
{
  SoQt::exitMainLoop();
}


/*!
  Deletes the world, and creates a new one.
*/
void
GraspitCore::emptyWorld(const char *name)
{
  World *new_world = new World(NULL, name);

  if (ivmgr)
  {
    ivmgr->setWorld(new_world);
    new_world->setIVMgr(ivmgr);
  }

  delete world;
  world = new_world;
}


/*!
  Checks if the initialization of graspitCore resulted in a TERMINAL_FAILURE.
*/
bool
GraspitCore::terminalFailure() const
{
  return initResult == TERMINAL_FAILURE;
}

void
GraspitCore::sensorCB(void *, SoSensor *)
{
  graspitCore->processPlugins();
}

void
GraspitCore::startPlugin(PluginCreator *creator, int argc, char **argv)
{
  Plugin *plugin = creator->createPlugin(argc, argv);
  if (plugin) { mActivePlugins.push_back(std::pair<Plugin *, std::string>(plugin, creator->type())); }
  if (!mActivePlugins.empty()) {
    mPluginSensor->schedule();
  }
}

void GraspitCore::processPlugins()
{
  std::list< std::pair<Plugin *, std::string> >::iterator it = mActivePlugins.begin();
  while (it != mActivePlugins.end()) {
    if (it->first->mainLoop() == SUCCESS) {
      it++;
    } else {
      delete it->first;
      it = mActivePlugins.erase(it);
    }
  }
  if (!mActivePlugins.empty()) {
    mPluginSensor->schedule();
  }
}

void GraspitCore::stopPlugin(Plugin *plugin)
{
  std::list< std::pair<Plugin *, std::string> >::iterator it;
  for (it = mActivePlugins.begin(); it != mActivePlugins.end(); it++) {
    if (it->first == plugin) {
      delete it->first;
      mActivePlugins.erase(it);
      return;
    }
  }
  DBGA("Stop plugin: plugin not found");
}

void GraspitCore::stopAllPlugins()
{
  std::list< std::pair<Plugin *, std::string> >::iterator it = mActivePlugins.begin();
  while (it != mActivePlugins.end()) {
    delete it->first;
    it = mActivePlugins.erase(it);
  }
}

