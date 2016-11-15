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
  \brief Implements the graspit user interface.  Responsible for creating both MainWindow and IVmgr.
*/

#include <Q3GroupBox>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <QLayout>

#ifdef Q_WS_X11
  #include <unistd.h>
#endif

#include "graspitGUI.h"
#include "mainWindow.h"
#include "ivmgr.h"
#include "world.h"
#include "mytools.h"
#include "SoComplexShape.h"
#include "SoArrow.h"
#include "SoTorquePointer.h"
#include "plugin.h"
#include "debug.h"

#ifdef CGDB_ENABLED
#include "dbase_grasp.h"
#include "taskDispatcher.h"
#endif

bool GraspItGUI::initialized = false;
int GraspItGUI::initResult = SUCCESS;

// Used by getopt in ProcessArgs
extern int optind,optopt,opterr;
extern char *optarg;

////////////////////////// SYSTEM WIDE GLOBALS ///////////////////////////////

#ifdef GRASPITDBG
FILE *debugfile;
#endif

//! This is the system wide pointer to the graspit user interface.
GraspItGUI *graspItGUI = 0;

//////////////////////////////////////////////////////////////////////////////

/*!
  If this class hasn't been initialized in another instance, it performs
  the following operations:
  - creates a new MainWindow,
  - starts Coin by initializing SoQt,
  - initializes our Coin add on classes,
  - creates a new IVmgr,
  - sets the focus policy of the SoQt viewer so keyboard events are accepted
  - calls a method to process the command line arguments.
 */
GraspItGUI::GraspItGUI(int argc,char **argv) : mDispatch(NULL)
{
  if (!initialized) {
    mainWindow = new MainWindow; 
    SoQt::init(mainWindow->mWindow);

    // initialize my Inventor additions
    SoComplexShape::initClass();
    SoArrow::initClass();
    SoTorquePointer::initClass();

    ivmgr = new IVmgr((QWidget *)mainWindow->mUI->viewerHolder,"myivmgr");
	
//	mainWindow->viewerHolder->setFocusProxy(ivmgr->getViewer()->getWidget());
//	mainWindow->viewerHolder->setFocusPolicy(QWidget::StrongFocus);
    
    ivmgr->getViewer()->getWidget()->setFocusPolicy(Qt::StrongFocus);

    initialized = true;
    graspItGUI = this;
    mExitCode = 0;
    initResult = processArgs(argc,argv);
  }
}

/*!
  Deletes both the IVmgr and the MainWindow.
*/
GraspItGUI::~GraspItGUI()
{
  //originally, ivmgr is first
//	fprintf(stderr,"Delete children\n");
//  mainWindow->destroyChildren();
//  fprintf(stderr,"Delete ivmgr\n");
  //if a dispatcher was being used, set exit code based on its result

  //clean up plugins and creators
  stopAllPlugins();
  for (size_t i=0; i<mPluginCreators.size(); i++) {
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
  Processes the command line arguments.  It first checks to make sure the
  GRASPIT environment variable is set, then if this is run under X11 it
  examines the command line.  The usage is the following:
\p graspit \p [-w worldname] \p [-r robotname] \p [-o objectname]
\p [-b obstaclename]
*/
int
GraspItGUI::processArgs(int argc, char** argv)
{
  QString filename;
  int errflag=0; 
  errflag = errflag;
  QString graspitRoot = QString(getenv("GRASPIT"));
  if (graspitRoot.isNull() ) {
    std::cerr << "Please set the GRASPIT environment variable to the root directory of graspIt." << std::endl;
    return FAILURE;
  }

  //look for plugins of the form plugin:name in the arguments
  for (int i=1; i<argc; i++) {
    QString arg(argv[i]);
    if (arg.section(',',0,0)=="plugin") {
      QString libName = arg.section(',',1,1);
	  std::cout << "Processing arguments \n ";
      PluginCreator* creator = PluginCreator::loadFromLibrary(libName.toStdString());	  
      if (creator) {
        mPluginCreators.push_back(creator);
      } else {
        DBGA("Failed to load plugin: " << libName.latin1());
      }
    }
  }
  
  //start any plugins with auto start enabled
  mPluginSensor = new SoIdleSensor(GraspItGUI::sensorCB, (void*)this);
  for (size_t i=0; i<mPluginCreators.size(); i++) {
	  std::cout << "plugin creator autostart " << mPluginCreators[i]->autoStart() << std::endl;
    if (mPluginCreators[i]->autoStart()) {
      startPlugin(mPluginCreators[i], mPluginCreators[i]->defaultArgs());
    }    
  }
  
  if(argc > 1){
#ifdef CGDB_ENABLED
	  if(!strcmp(argv[1],"dbase")){
		  DBaseBatchPlanner *dbp = new DBaseBatchPlanner(graspItGUI->getIVmgr(), this);
		  dbp->processArguments(argc, argv);
		  dbp->startPlanner();
	  }
	  if (!strcmp(argv[1],"db_dispatch")) {
		  mDispatch = new TaskDispatcher();
		  if (mDispatch->connect("wgs36",5432,"willow","willow","household_objects")) {
			  std::cerr << "DBase dispatch failed to connect to database\n";
			  return TERMINAL_FAILURE;
		  }
		  std::cerr << "DBase dispatch connected to database\n";
		  mDispatch->mainLoop();
		  if (mDispatch->getStatus() != TaskDispatcher::RUNNING) {
			  mExitCode = mDispatch->getStatus();
			  return TERMINAL_FAILURE;
		  }
	  }
#endif
  }
 
#ifdef Q_WS_X11
  char c;
  while((c=getopt(argc, argv, "r:w:o:b:")) != EOF) {
    switch(c) {
    case 'r':
      filename = graspitRoot + QString("/models/robots/")+
	QString(optarg) + QString("/") + QString(optarg) + QString(".cfg");
      if (ivmgr->getWorld()->importRobot(filename)==NULL)
	++errflag;
      break;
    case 'w':
      filename = graspitRoot + QString("/worlds/")+ QString(optarg) +
	QString(".wld");
      if (ivmgr->getWorld()->load(filename)==FAILURE)
	++errflag;
      else
	mainWindow->mUI->worldBox->setTitle(filename);
      break;
    case 'o':
      filename = graspitRoot + QString("/models/objects/")+ QString(optarg) +
	QString(".iv");
      if (!ivmgr->getWorld()->importBody("GraspableBody",filename))
	++errflag;
      break;
    case 'b':
      filename = graspitRoot + QString("/models/obstacles/")+ QString(optarg) +
	QString(".iv");
      if (!ivmgr->getWorld()->importBody("Body",filename))
	++errflag;
      break;
    default: 
      ++errflag;
      break;
    }
  }
  if (errflag) {
    std::cerr << "Usage: graspit [-w worldname] [-r robotname] [-o objectname] [-b obstaclename]" << std::endl;
    return FAILURE;
  }
#endif
  return SUCCESS;
}

/*!
  Shows the mainWindow, sets its size, and starts the Qt event loop.
*/
void
GraspItGUI::startMainLoop()
{	
  SoQt::show(mainWindow->mWindow);
  mainWindow->setMainWorld(ivmgr->getWorld());
  mainWindow->mWindow->resize(QSize(1070,937));  
  SoQt::mainLoop();
}  

/*!
  Exits the Qt event loop.
*/
void
GraspItGUI::exitMainLoop()
{
  SoQt::exitMainLoop();
}

bool
GraspItGUI::terminalFailure() const
{
  return initResult == TERMINAL_FAILURE;
}

void
GraspItGUI::sensorCB(void*, SoSensor*)
{
  graspItGUI->processPlugins();
}

void 
GraspItGUI::startPlugin(PluginCreator* creator, std::string args)
{
  Plugin *plugin = creator->createPlugin(args);
  if (plugin) mActivePlugins.push_back( std::pair<Plugin*, std::string>(plugin, creator->type()) );
  if (!mActivePlugins.empty()) {
    mPluginSensor->schedule();
  }  
}

void GraspItGUI::processPlugins()
{
  std::list< std::pair<Plugin*, std::string> >::iterator it = mActivePlugins.begin();
  while (it != mActivePlugins.end()) {
    if (it->first->mainLoop() == SUCCESS) {
      it++;
    } else{
      delete it->first;
      it = mActivePlugins.erase(it);
    }
  }
  if (!mActivePlugins.empty()) {
    mPluginSensor->schedule();
  }
}

void GraspItGUI::stopPlugin(Plugin *plugin)
{
  std::list< std::pair<Plugin*, std::string> >::iterator it;
  for (it = mActivePlugins.begin(); it!=mActivePlugins.end(); it++) {
    if (it->first == plugin) {
      delete it->first;
      mActivePlugins.erase(it);
      return;
    }
  }
  DBGA("Stop plugin: plugin not found");
}

void GraspItGUI::stopAllPlugins()
{
  std::list< std::pair<Plugin*, std::string> >::iterator it = mActivePlugins.begin();
  while (it != mActivePlugins.end()) {
    delete it->first;
    it = mActivePlugins.erase(it);
  }
}
