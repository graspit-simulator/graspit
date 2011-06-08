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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mainWindow.cpp,v 1.25 2009/09/29 22:26:41 cmatei Exp $
//
//######################################################################

#include "mainWindow.h"

#include <QDialog>
#include <QStatusBar>

#include <QLineEdit>
#include <QMessageBox>
#include <QApplication>
#include <QComboBox>
#include <QStatusBar>
#include <QTime>
#include <QFileDialog>

#include <list>

#include "defines.h"
#include "mytools.h"
#include "robot.h"
#include "ivmgr.h"
#include "world.h"
#include "body.h"
#include "grasp.h"
#include "gwsprojection.h"
#include "quality.h"
#include "graspitGUI.h"
#include "humanHand.h"
//#include "about.h"
//#include "GeometryLibInterface/geometryLibDlgWrapper.h"
//#include "rosWrapper.h"
#include "arch.h"

#include "debug.h"

#include "settingsDlg.h"
#include "ui_about.h"
#include "bodyPropDlg.h"
#include "qmDlg.h"
#include "gwsProjDlg.h"
#include "plannerdlg.h"
#include "eigenGraspDlg.h"
#include "compliantPlannerDlg.h"
#include "optimizerDlg.h"
#include "gfoDlg.h"
#include "contactExaminerDlg.h"
#include "egPlannerDlg.h"
#ifdef CGDB_ENABLED
#include "DBase/dbaseDlg.h"
#include "DBase/dbasePlannerDlg.h"
#include "graspit_db_model.h"
#endif
#ifdef ARIZONA_PROJECT_ENABLED
#include "arizona/arizonaProjectDlg.h"
#endif
#ifdef STAUBLI_CONTROL_ENABLED
#include "staubli/staubliControlDlg.h"
#endif
#include "gloveCalibrationDlg.h"
#include "graspCaptureDlg.h"
#include "barrettHandDlg.h"
#include "archBuilderDlg.h"
#ifdef EIGENGRIDS
#include "eigenGridsDlg.h"
#endif
#ifdef HARDWARE_LIB
#include "sensorInputDlg.h"
#endif

//------------------------------------ CONSTRUCTOR AND DESTRUCTOR -------------------------------------

MainWindow::MainWindow(QWidget *parent) 
{
  mWindow = new Q3MainWindow(parent);
  //mWindow = new QMainWindow(parent);
  mUI = new Ui::MainWindowUI;
  mUI->setupUi(mWindow);
  init();
  // -- file menu
  QObject::connect(mUI->fileNewAction, SIGNAL(triggered()), this, SLOT(fileNew()));
  QObject::connect(mUI->fileOpenAction, SIGNAL(triggered()), this, SLOT(fileOpen()));
  QObject::connect(mUI->fileSaveAction, SIGNAL(triggered()), this, SLOT(fileSave()));
  QObject::connect(mUI->fileSaveAsAction, SIGNAL(triggered()), this, SLOT(fileSaveAs()));
  QObject::connect(mUI->fileImportRobotAction, SIGNAL(triggered()), this, SLOT(fileImportRobot()));
  QObject::connect(mUI->fileImportObstacleAction, SIGNAL(triggered()), this, SLOT(fileImportObstacle()));
  QObject::connect(mUI->fileImportObjectAction, SIGNAL(triggered()), this, SLOT(fileImportObject()));
  QObject::connect(mUI->fileEditSettingsAction, SIGNAL(triggered()), this, SLOT(fileEditSettings()));
  QObject::connect(mUI->fileSaveImageAction, SIGNAL(triggered()), this, SLOT(fileSaveImage()));
  QObject::connect(mUI->fileExitAction, SIGNAL(triggered()), this, SLOT(fileExit()));
  // -- help menu
  QObject::connect(mUI->helpManualAction, SIGNAL(triggered()), this, SLOT(helpManual()));
  QObject::connect(mUI->helpAboutAction, SIGNAL(triggered()), this, SLOT(helpAbout()));
  QObject::connect(mUI->helpAboutQTAction, SIGNAL(triggered()), this, SLOT(helpAboutQT()));
  // -- body meny
  QObject::connect(mUI->elementGroup, SIGNAL(selected(QAction*)), this, SLOT(setTool(QAction*)));
  QObject::connect(mUI->elementCollisionToggleAction, SIGNAL(activated()),
                   this, SLOT(elementTurnOffCollisions()));
  QObject::connect(mUI->elementCollisionToggleAction, SIGNAL(toggled(bool)),
                   this, SLOT(updateCollisionAction(bool)));
  QObject::connect(mUI->elementBodyPropertiesAction, SIGNAL(activated()),
                   this, SLOT(elementBodyProperties()));
  // -- grasp menu, part I
  QObject::connect(mUI->graspAutoGraspAction, SIGNAL(triggered()), this, SLOT(graspAutoGrasp()));
  QObject::connect(mUI->graspAuto_OpenAction, SIGNAL(triggered()), this, SLOT(graspAutoOpen()));
  QObject::connect(mUI->graspQualityMeasuresAction, SIGNAL(triggered()), this, SLOT(graspQualityMeasures()));
  QObject::connect(mUI->graspCreateProjectionAction, SIGNAL(triggered()), this, SLOT(graspCreateProjection()));
  QObject::connect(mUI->graspPlannerAction, SIGNAL(triggered()), this, SLOT(graspPlanner()));
  QObject::connect(mUI->graspGFOAction, SIGNAL(triggered()), this, SLOT(graspForceOptimization()));
  // -- grasp menu, part I
  QObject::connect(mUI->graspEigenGrasp_InterfaceAction, SIGNAL(triggered()), 
                   this, SLOT(eigenGraspActivated()));
  QObject::connect(mUI->graspContact_ExaminerAction, SIGNAL(triggered()),
                   this, SLOT(graspContactExaminer_activated()));
  QObject::connect(mUI->graspEigenGrasp_PlannerAction, SIGNAL(triggered()),
                   this, SLOT(eigenGraspPlannerActivated()));
  // -- dbase menu
  QObject::connect(mUI->dbaseGUIAction, SIGNAL(triggered()), 
                   this, SLOT(dbaseGUIAction_activated()));
  QObject::connect(mUI->dbasePlannerAction, SIGNAL(triggered()), 
                   this, SLOT(dbasePlannerAction_activated()));
  QObject::connect(mUI->dbaseGraspCaptureAction, SIGNAL(triggered()),
                   this, SLOT(graspCapture()));
  // -- sensors menu
  QObject::connect(mUI->sensorsSensor_InputAction, SIGNAL(triggered()), 
                   this, SLOT(sensorsSensor_InputAction_activated()));
  QObject::connect(mUI->sensorsBarrett_HandAction, SIGNAL(triggered()),
                   this, SLOT(sensorsBarrettHandAction()));
  // -- stereo menu
  QObject::connect(mUI->stereoOnAction, SIGNAL(triggered()), this, SLOT(stereoOn()));
  QObject::connect(mUI->stereoOffAction, SIGNAL(triggered()), this, SLOT(stereoOff()));
  QObject::connect(mUI->stereoFlip_leftrightAction, SIGNAL(triggered()), this, SLOT(stereoFlip()));
  // -- misc menu
  QObject::connect(mUI->dynamicsArch_BuilderAction, SIGNAL(triggered()), this, SLOT(archBuilder()));
  QObject::connect(mUI->miscOptimizerAction, SIGNAL(triggered()), this, SLOT(miscOptimizer()));
  QObject::connect(mUI->actionArizona_Project, SIGNAL(triggered()), 
                   this, SLOT(miscArizonaProjectDlg_activated()));
  QObject::connect(mUI->staubliControl, SIGNAL(triggered()), 
                   this, SLOT(misStaubliControlDlg()));
  QObject::connect(mUI->miscEigengridsAction, SIGNAL(triggered()), this, SLOT(miscEigengridsAction_activated()));
  // -- contacts
  QObject::connect(mUI->contactsListBox, SIGNAL(highlighted(int)), this, SLOT(contactSelected(int)));
  // -- dynamics
  QObject::connect(mUI->dynamicsPlayAction, SIGNAL(activated()), this, SLOT(toggleDynamics()));
  QObject::connect(mUI->dynamicsPopAction, SIGNAL(activated()), this, SLOT(dynamicsPopState()));
  QObject::connect(mUI->dynamicsPushAction, SIGNAL(activated()), this, SLOT(dynamicsPushState()));
  // -- toolbars
  QObject::connect(mUI->materialComboBox, SIGNAL(activated(int)), this, SLOT(materialSelected(int)));
  QObject::connect(mUI->graspedBodyBox, SIGNAL(activated(int)), this, SLOT(selectGraspedBody(int)));
  QObject::connect(mUI->handSelectionBox, SIGNAL(activated(int)), this, SLOT(setCurrentHand(int)));
  // -- tendon toolbar
  QObject::connect(mUI->TendonForceInput, SIGNAL(valueChanged(int)), this, SLOT(TendonForceInput_valueChanged(int)));
  QObject::connect(mUI->tendonNamesBox, SIGNAL(activated(int)), this, SLOT(tendonNamesBoxActivated(int)));
  QObject::connect(mUI->tendonVisibleCheckBox, SIGNAL(toggled(bool)), this, SLOT(tendonVisibleCheckBox_toggled(bool)));
  QObject::connect(mUI->forcesVisibleCheckBox, SIGNAL(toggled(bool)), this, SLOT(forcesVisibleCheckBox_toggled(bool)));}

/*!
  UI constructor.  Zeros the time readout, sets the correct states for the
  dynamics and collision checking buttons.
*/
void MainWindow::init()
{
  world = NULL;
  mUI->timeReadout->display("00:00.000");
  mWindow->statusBar()->message("Ready",2000);
  QIcon playIconSet = mUI->dynamicsPlayAction->iconSet();
  playIconSet.setPixmap(load_pixmap( "pause.xpm" ),QIcon::Automatic,QIcon::Normal,QIcon::On);
  mUI->dynamicsPlayAction->setIconSet(playIconSet);
  
  QIcon collisionIconSet = mUI->elementCollisionToggleAction->iconSet();
  collisionIconSet.setPixmap(load_pixmap("nocollide.xpm"),QIcon::Automatic,QIcon::Normal,QIcon::On);
  mUI->elementCollisionToggleAction->setIconSet(collisionIconSet);
}

/*!
  Stub UI destructor.
*/
void MainWindow::destroy()
{
}

void MainWindow::destroyChildren()
{
 //attempts to kill any open children;
 //QWidget::destroy(FALSE, TRUE);
}

/*!
  Sets the world that the main window interface will deal with.
  Sets up all signal/slot connections with this world.
*/
void MainWindow::setMainWorld( World *w )
{
  world = w;
  QObject::connect(world,SIGNAL(dynamicStepTaken()),this,SLOT(updateTimeReadout()));
  QObject::connect(world,SIGNAL(dynamicStepTaken()),graspItGUI->getIVmgr(),SLOT(drawDynamicForces())); 
  QObject::connect(world,SIGNAL(dynamicsError(const char *)),this,SLOT(showDynamicsError(const char *)));
  QObject::connect(world,SIGNAL(selectionsChanged()),this,SLOT(updateElementMenu()));
  QObject::connect(world,SIGNAL(selectionsChanged()),this,SLOT(updateMaterialBox()));
  QObject::connect(world,SIGNAL(numElementsChanged()),this,SLOT(updateGraspBoxes()));  
  QObject::connect(world,SIGNAL(numElementsChanged()),this,SLOT(updateGraspMenu()));
  // This one is not ideal, but if a grasped object is deleted the grasp will be updated and we need to show it
  QObject::connect(world,SIGNAL(numElementsChanged()),this,SLOT(updateQualityList()));
  QObject::connect(world,SIGNAL(tendonSelectionChanged()),this,SLOT(handleTendonSelectionArea()));
  QObject::connect(world,SIGNAL(tendonDetailsChanged()),this,SLOT(handleTendonDetailsArea()));
  QObject::connect(world,SIGNAL(handSelectionChanged()),this,SLOT(handleHandSelectionChange()));
  QObject::connect(world,SIGNAL(graspsUpdated()),this,SLOT(updateQualityList()));
  QObject::connect(world,SIGNAL(graspsUpdated()),graspItGUI->getIVmgr(),SLOT(drawWorstCaseWrenches()));
  QObject::connect(world,SIGNAL(handRemoved()),this,SLOT(updateQualityList()));
  updateTimeReadout();
  updateMaterialBoxList();
  updateGraspBoxes();
  updateQualityList();
  updateElementMenu();
  updateGraspMenu();
  updateTendonNamesBox();
  handleTendonSelectionArea();

#ifdef EIGENGRIDS
  mUI->miscEigengridsAction->setEnabled(TRUE);
#else
  mUI->miscEigengridsAction->setEnabled(FALSE);
#endif

#ifdef HARDWARE_LIB
  mUI->sensorsSensor_InputAction->setEnabled(TRUE);
#else
  mUI->sensorsSensor_InputAction->setEnabled(FALSE);
#endif

}

//-------------------------------------------------- FILE MENU ----------------------------------------

/*!
  First checks if the user wants to save the current world.  Then if the
  new command is not aborted, it empties the world, and resets the name to
  "Untitled".
*/
void MainWindow::fileNew()
{
  if ( saveAndContinue( "New" ) ) {
    graspItGUI->getIVmgr()->emptyWorld();
    setMainWorld(graspItGUI->getIVmgr()->getWorld());
    mUI->worldBox->setTitle("Untitled");
  }
}

/*!
  First checks if the user wants to save the current world.  Then if the open
  command is not aborted, it brings up a file opened dialog box with the
  $GRASPIT/worlds directory open.  When the user chooses a saved world, this
  empties the current world and loads the chosen one.  
*/
void MainWindow::fileOpen()
{
  if ( !saveAndContinue( "Open" ) ) {
    return;
  }
  QString fn( QFileDialog::getOpenFileName(mWindow, QString(), QString(getenv("GRASPIT"))+QString("/worlds"),
                                           "GraspIt World Files (*.xml)") );
  if ( fn.isEmpty() ) {
    return;
  }
  fileName = fn;
  mUI->worldBox->setTitle(fileName);
  graspItGUI->getIVmgr()->emptyWorld();
  graspItGUI->getIVmgr()->getWorld()->load(fileName);
  //graspItGUI->getIVmgr()->getViewer()->viewAll();
  setMainWorld(graspItGUI->getIVmgr()->getWorld());
}

/*!
  If the current world has a filename, this saves the world overwriting that
  file.  If there is no current filename, this opens the saveAs dialog box.
*/
void MainWindow::fileSave()
{
  if ( fileName.isEmpty() ) {
    fileSaveAs();
  } else {
    graspItGUI->getIVmgr()->getWorld()->save(fileName);
  }
}

/*!
  A file dialog box is opened with the $GRASPIT/worlds directory opened, and
  the user is prompted to save the world file.  If the user does not add an
  extension, the ".xml" extension is added.
*/
void MainWindow::fileSaveAs()
{
  QString fn = QFileDialog::getSaveFileName(mWindow, QString(), QString(getenv("GRASPIT"))+QString("/worlds"),
                                            "GraspIt World Files (*.xml)" );
  if ( !fn.isEmpty() ) {
    fileName = fn;
    if (fileName.section('.',1).isEmpty()) {
      fileName.append(".xml");
    }
    fileSave();
    mUI->worldBox->setTitle(fileName);
  }
}

/*!
  A file dialog box is opened with the $GRASPIT/models/robots directory
  opened, and the user can choose a robot configuration file to open.
  The Robot is imported into the world and positioned at the origin.
*/
void MainWindow::fileImportRobot()
{
  QString dir = QString(getenv("GRASPIT"))+QString("/models/robots");
  QString fn( QFileDialog::getOpenFileName(mWindow, QString(), dir, "XML GraspIt Robot Files (*.xml)") );
  if ( !fn.isEmpty() ) {
    world->importRobot(fn);
  }
}

/*!
  A file dialog box is opened with the $GRASPIT/models/obstacles directory
  opened, and the user can choose a model file to open.  If this is
  compiled with COIN2 the model type can be in either VRML 2.0 or Inventor
  format, otherwise it must be in Inventor format.
  The Body is imported into the world and positioned at the origin.
*/
void MainWindow::fileImportObstacle()
{
  QString fn( QFileDialog::getOpenFileName(mWindow, QString(),
                                           QString(getenv("GRASPIT"))+QString("/models/obstacles"),
	    "Graspit XML Files (*.xml*);;IV files (*.iv);;WRL files (*.wrl);;OFF files (*.off);;PLY files (*.ply)" ) );
  if ( !fn.isEmpty() ) world->importBody("Body",fn);
}

/*!
  A file dialog box is opened with the $GRASPIT/models/objects directory
  opened, and the user can choose a model file to open.  If this is
  compiled with COIN2 the model type can be in either VRML 2.0 or Inventor
  format, otherwise it must be in Inventor format.
  The GraspableBody is imported into the world and positioned at the origin.
*/
void MainWindow::fileImportObject()
{
  QString fn( QFileDialog::getOpenFileName(mWindow, QString(),
                                           QString(getenv("GRASPIT"))+QString("/models/objects"),
  "Graspit XML Files (*.xml*);;IV files (*.iv);;WRL files (*.wrl);;OFF files (*.off);;PLY files (*.ply)" ) );

  if ( !fn.isEmpty() ) world->importBody("GraspableBody",fn);
}

/*!
  First checks if the user wants to save the current world.  Then if the quit
  is not aborted, this exits the main interaction loop, to begin application
  shutdown.
*/
void MainWindow::fileExit()
{
  if ( saveAndContinue( "Exit" ) )
    //qApp->exit();
    graspItGUI->exitMainLoop();
}

/*!
  If the world was modified since the last time it was saved, this function
  will ask the user if they would like to save the world before completing
  the requested operation (which is passed in as a string).  The three options
  are: save and continue the operation, don't save but continue the operation,
  or cancel the operation.
*/
int MainWindow::saveAndContinue(const QString & action)
{
  int continueAction = 1;
  
  if (world->wasModified()) { //if modified...
    switch( QMessageBox::information(
     mWindow, "GraspIt!",
     "The document contains unsaved changes.\n"
     "Do you want to save the changes?",
     "&Save", "&Don't Save", "&Cancel " + action,
     0, // Enter == button 0
     2 ) ) { // Escape == button 2
      case 0: // Save; continue
      fileSave();
      break;
      case 1: // Do not save; continue
      break;
      case 2: // Cancel
      continueAction = 0;
      break;
    }
  }
 
  return continueAction;
}

/*!
  Opens a dialog box that allows the user to edit application settings.
  If the dialog box is accepted (ok is pressed), it makes the settings changes
  to the world.  At the end of this routine, friction cones are redrawn in
  case and of the COF's have changed, and the material selection box is
  updated in case any of the material names have been changed.
*/
void MainWindow::fileEditSettings()
{
  int i,j;
  SettingsDlg *dlg = new SettingsDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, true);
  
  if ( dlg->exec() == QDialog::Accepted) {
    //Process the changes to the COFs
    for (i=0;i<dlg->dlgUI->staticFrictionTable->numRows()-1;i++) {
      world->materialNames[i] = dlg->dlgUI->staticFrictionTable->text(i+1,0);
      
      for (j=0;j<dlg->dlgUI->staticFrictionTable->numCols()-1;j++) {
		world->cofTable[i][j] = dlg->dlgUI->staticFrictionTable->text(i+1,j+1).toDouble();
		world->kcofTable[i][j] = dlg->dlgUI->kineticFrictionTable->text(i+1,j+1).toDouble();
      }
    }
    world->dynamicsTimeStep = dlg->dlgUI->timeStepLine->text().toDouble()*1.0e-3;
    updateMaterialBoxList();  
    
    //this will redraw friction cones in case any cofs have changed
	for (i=0;i<world->getNumBodies();i++) {
      world->getBody(i)->setMaterial(world->getBody(i)->getMaterial());
	}
    world->updateGrasps();
  }  
  delete dlg;
}

/*!
  A file dialog box is opened with the $GRASPIT/images directory
  opened, and the user is asked to choose a name for the image.
  Currently this only save jpg images, but functionality should be added to
  save other formats.  If the user does not add an extension, ".jpg" is added
  to the filename.  A rendered images of the current scence is then saved.
*/
void MainWindow::fileSaveImage()
{
  QString fn = QFileDialog::getSaveFileName( mWindow, QString(), QString(getenv("GRASPIT"))+QString("/images"),
                                             "Image Files (*.jpg)" );
  if ( !fn.isEmpty() ) {
    if (fn.section('.',-1)!="jpg") fn.append(".jpg");
    graspItGUI->getIVmgr()->saveImage(fn);
  }
}

//----------------------------------------- Help menu --------------------------------------

/*! 
  Brings up a dialog saying where the manual can be found.
*/
void MainWindow::helpManual()
{
  QMessageBox::warning(NULL,"GraspIt!","You can find the GraspIt! User Manual in $GRASPIT/doc, " \
                       "or online at http://www.cs.columbia.edu/~cmatei/graspit/",
                       QMessageBox::Ok, Qt::NoButton,Qt::NoButton);
}

/*!
 Brings up a dialog box with the graspit logo, the version number,
 a copyright, and an OK button.
*/
void MainWindow::helpAbout()
{
  Ui::AboutDlg dlgUI;
  QDialog dlgImpl(mWindow);
  dlgUI.setupUi(&dlgImpl);
  QString versionText("Version ");
  dlgUI.versionLabel->setText( versionText + GRASPIT_VERSION);
  dlgImpl.exec();
}

void MainWindow::helpAboutQT()
{
  QMessageBox::aboutQt(mWindow,"GraspIt!");
}

//-----------------------------------------  Element Menu ---------------------------------

/*!
  Based on what tool button was chosen this sets the current tool in the IVMgr.
*/
void MainWindow::setTool( QAction *a )
{
  if (a==mUI->translateToolAction) graspItGUI->getIVmgr()->setTool(TRANSLATE_TOOL);
  else if (a==mUI->rotateToolAction) graspItGUI->getIVmgr()->setTool(ROTATE_TOOL);
  else if (a==mUI->selectToolAction) graspItGUI->getIVmgr()->setTool(SELECT_TOOL);
}

/*!
  Toggles world collisions.  Actuall effect depends on what is currently selected.
*/
void MainWindow::elementTurnOffCollisions()
{
  world->toggleAllCollisions(!mUI->elementCollisionToggleAction->isOn());
}

/*!
  Opens the body properties dialog box, allowing the user to change the
  properties of the currently selected body(ies).  After the dialog box
  is accepted, all grasps are updated since materials may have been changed.
*/
void MainWindow::elementBodyProperties()
{	
  BodyPropDlg *dlg = new BodyPropDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, true);
  if ( dlg->exec() == QDialog::Accepted) {
    world->updateGrasps();
  }
  delete dlg;
}

void MainWindow::elementPrimitives()
{
#ifdef GEOMETRY_LIB
  //needs upgrade to Qt4
  /*
    GeometryLibDlg *dlg = new GeometryLibDlg(this,QString::null,FALSE);
    assert( world->getNumSelectedBodies() > 0 );
    dlg->setBody( world->getSelectedBody(0) );
    dlg->show();
  */
#endif
}

/*!
  Enables items in the element menu based on how many world elements are 
  selected.  Also sets the state of the toggleCollisions action based on
  which elements are selected.
*/
void MainWindow::updateElementMenu()
{
  bool collisionsOFF;
  std::list<WorldElement *> selectedElementList;
  std::list<WorldElement *>::iterator  ep;
  
  if (world->getNumSelectedElements()==0)
    mUI->elementBodyPropertiesAction->setEnabled(false);
  else
    mUI->elementBodyPropertiesAction->setEnabled(true);
  
#ifdef GEOMETRY_LIB
  if (world->getNumSelectedBodies()==1)
    elementPrimitivesAction->setEnabled(true);
  else
    elementPrimitivesAction->setEnabled(false);
#endif

  if (world->getNumSelectedElements()==0)
   collisionsOFF = world->collisionsAreOff();
  
  else if (world->getNumSelectedElements()==2)
    collisionsOFF = world->collisionsAreOff(world->getSelectedElementList().front(),
                world->getSelectedElementList().back());
  else {
    collisionsOFF = true;
    selectedElementList = world->getSelectedElementList();
    for (ep=selectedElementList.begin();ep!=selectedElementList.end();ep++)
      if (!world->collisionsAreOff(*ep)) {
      collisionsOFF= false;
      break;
    }
  }
  mUI->elementCollisionToggleAction->setOn(collisionsOFF);
   
}

//------------------------------------------- Grasp Menu ----------------------------------

/*!
  Enables items in the grasp menu based on whether a current hand can be found or not.
*/
void MainWindow::updateGraspMenu()
{
    bool handFound = (world->getCurrentHand() !=NULL);
    bool bodyFound = (world->getNumBodies() != 0);
    mUI->graspAutoGraspAction->setEnabled(handFound);
    mUI->graspAuto_OpenAction->setEnabled(handFound);
    mUI->graspCreateProjectionAction->setEnabled(handFound);
    mUI->graspQualityMeasuresAction->setEnabled(handFound);
    mUI->graspPlannerAction->setEnabled(handFound);
    mUI->graspGFOAction->setEnabled(handFound);
    mUI->graspEigenGrasp_InterfaceAction->setEnabled(handFound);
    mUI->graspContact_ExaminerAction->setEnabled(bodyFound);
    mUI->graspEigenGrasp_PlannerAction->setEnabled(handFound);
    mUI->graspCompliantPlannerAction->setEnabled(handFound);
#ifdef CGDB_ENABLED
    mUI->dbaseGUIAction->setEnabled(handFound);
    mUI->dbasePlannerAction->setEnabled(handFound);
#else
    mUI->dbaseGUIAction->setEnabled(false);
    mUI->dbasePlannerAction->setEnabled(false);
#endif
    
#ifdef ARIZONA_PROJECT_ENABLED
    mUI->actionArizona_Project->setEnabled(true);
#else
    mUI->actionArizona_Project->setEnabled(false);
#endif
    
    mUI->dbaseGraspCaptureAction->setEnabled(handFound);
    mUI->sensorsBarrett_HandAction->setEnabled(handFound);
}

/*!
  Opens a GWS Projection dialog box.  After the user has chosen which
  coordinates to fix at particular values, this creates a new gws projection
  using those values.  It then adds the projection to the grasp of the
  currently selected hand.
*/
void MainWindow::graspCreateProjection(Grasp *g)
{
  GWSProjDlg *dlg = new GWSProjDlg(mWindow);
  if ( dlg->exec() == QDialog::Accepted ) {
    Grasp *grasp;
    if(!g)
      grasp = world->getCurrentHand()->getGrasp();
    else
      grasp = g;
    GWS *gws = grasp->addGWS(dlg->gwsTypeComboBox->currentText().latin1());
    
    double w[6];
    w[0] = dlg->fxCoord->text().toDouble();
    w[1] = dlg->fyCoord->text().toDouble();
    w[2] = dlg->fzCoord->text().toDouble();
    w[3] = dlg->txCoord->text().toDouble();
    w[4] = dlg->tyCoord->text().toDouble();
    w[5] = dlg->tzCoord->text().toDouble();
    
    GWSprojection *gp = new GWSprojection(graspItGUI->getIVmgr()->getViewer(),gws,w,dlg->whichFixed);  
    grasp->addProjection(gp);
  }    
  delete dlg;
}

/*!
  Opens the Quality Measures dialog box, which allows the user to 
  create new qm's, edit current ones, or delete them.  After the dialog
  box is closed it revaluates all grasps.
*/
void MainWindow::graspQualityMeasures()
{
  QMDlg *dlg = new QMDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, true);
  if ( dlg->exec() == QDialog::Accepted ) {
    Grasp *g = world->getCurrentHand()->getGrasp();
    if (g->getNumQM()) {
      connect(g,SIGNAL(graspUpdated()),this,SLOT(updateQualityList()));
    } else {
      disconnect(g,SIGNAL(graspUpdated()),this,SLOT(updateQualityList()));
    }    
  }
  world->getCurrentHand()->setContactsChanged();
  world->updateGrasps();
  delete dlg;
}

/*!
  First checks the current hand is a Barrett hand (only one the planner works
  with).  Then it opens the grasp planner dialog box.  The is not a modeless
  box (does not prevent user interaction with the main window) so control
  is returned to the main loop after the dlg box is opened.
*/
void MainWindow::graspPlanner()
{
  if (!world->getCurrentHand()->getName().contains("Barrett")) {
    QMessageBox::warning(NULL,"GraspIt!",
                         "The planner currently only works with the Barrett hand.",
                         QMessageBox::Ok, Qt::NoButton,Qt::NoButton);
    return;
  }
  PlannerDlg *dlg =new PlannerDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();  
}

void
MainWindow::graspForceOptimization()
{
  if (!world->getCurrentHand()) {
    DBGA("No hand selected");
    return;
  }
  GFODlg *dlg = new GFODlg(this, world->getCurrentHand(), mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();  
}

/*! This fires up a window for grasp planning with compliant hands. In the future
	this interface might be redesigned, to use the same frameworks as either the
	regular planner or the eigengrasp planner 
*/
void MainWindow::graspCompliantPlanner()
{	
  int gb = mUI->graspedBodyBox->currentItem();
  if ( gb < 0 || world->getNumGB() < gb+1 ) {
    fprintf(stderr,"No object selected\n");
    return;
  }
  CompliantPlannerDlg *dlg =new CompliantPlannerDlg(world->getCurrentHand(),
                                                    world->getGB(gb), mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();  
}
/*!
  Starts an autograsp for the current hand and updated all grasps when it
  is complete.
*/
void MainWindow::graspAutoGrasp()
{
  world->getCurrentHand()->autoGrasp(true);
  world->updateGrasps();
}

/*!
  Opens the fingers by performing an autograsp in the opposite directions
*/

void MainWindow::graspAutoOpen()
{
  world->getCurrentHand()->autoGrasp(true,-1.0);
  world->updateGrasps();
}

void MainWindow::eigenGraspActivated()
{ 
  EigenGraspDlg *dlg = new EigenGraspDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  
  if (!dlg->setWorld(world) ) {
    delete dlg;
    return;
  } 
  dlg->show();
}

void MainWindow::graspContactExaminer_activated()
{
  ContactExaminerDlg *dlg = new ContactExaminerDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
}

void MainWindow::eigenGraspPlannerActivated()
{
  assert(world->getCurrentHand());	
  if (world->getCurrentHand()->getEigenGrasps() == NULL) {
    fprintf(stderr,"Current hand has no EigenGrasp information!\n");
    return;
  }
  int gb = mUI->graspedBodyBox->currentItem();
  if ( gb < 0 || world->getNumGB() < gb+1 ) {
    fprintf(stderr,"No object selected\n");
    return;
  }
  
  EigenGraspPlannerDlg *dlg = new EigenGraspPlannerDlg(mWindow);
  dlg->setMembers(world->getCurrentHand(), world->getGB(gb));
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
}

void MainWindow::dbaseGUIAction_activated()
{
#ifdef CGDB_ENABLED
  DBaseDlg *dlg = new DBaseDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
#endif
}

void MainWindow::dbasePlannerAction_activated()
{
#ifdef CGDB_ENABLED
  if (!world->getCurrentHand()) {
    QTWARNING("No hand selected");
    return;
  }
  int gb = mUI->graspedBodyBox->currentItem();
  if ( gb < 0 || world->getNumGB() < gb+1 ) {
    QTWARNING("No object selected");
    return;
  }
  if (!graspItGUI->getIVmgr()->getDBMgr()) {
    QTWARNING("Connection to database not established. Connect to database first.");
    return;
  }
  if (!world->getGB(gb)->getDBModel()) {
    QTWARNING("DBase Planner currently works only with models loaded from the database");
    return;
  }
  DBasePlannerDlg *dlg = new DBasePlannerDlg(mWindow, graspItGUI->getIVmgr()->getDBMgr(), 
                                             world->getGB(gb)->getDBModel(), world->getCurrentHand());
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
#endif
}


void MainWindow::miscArizonaProjectDlg_activated()
{
#ifdef ARIZONA_PROJECT_ENABLED
  ArizonaProjectDlg *dlg = new ArizonaProjectDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
#endif
}

void MainWindow::misStaubliControlDlg()
{
#ifdef STAUBLI_CONTROL_ENABLED
  StaubliControlDlg *dlg = new StaubliControlDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
#endif
}
//--------------------------------------- Sensors menu -------------------------------

void MainWindow::sensorsSensor_InputAction_activated()
{
#ifdef HARDWARE_LIB
  SensorInputDlg *dlg = new SensorInputDlg(world, mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();
#endif
}

void MainWindow::graspCapture()
{
  GraspCaptureDlg *dlg = new GraspCaptureDlg(world,mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  //QObject::connect(world, SIGNAL(graspsUpdated()),dlg, SLOT(updateGraspQuality())); 
  dlg->show(); 
}

void MainWindow::sensorsBarrettHandAction()
{
	
#ifdef HARDWARE_LIB
  BarrettHandDlg *dlg = new BarrettHandDlg(mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  if (dlg->setWorld(world)) {
    dlg->show();
  } else {
    delete dlg;
    return;
  }
#endif
 
}

//------------------------------------- Stereo menu ------------------------------------------

void MainWindow::stereoOn()
{
	DBGA("Stereo on");

	mUI->worldBox->setTitle(NULL);
	mUI->contactsGroupBox->hide();
	mUI->qualityGroupBox->hide();

	mUI->toolBar->hide();
	mUI->Toolbar_2->hide();
	mUI->ToolbarDynamics->hide();
	mUI->graspToolbar->hide();
	mUI->tendonToolbar->hide();

	mWindow->menuBar()->hide();
	graspItGUI->getIVmgr()->getViewer()->setDecoration(false);
	mWindow->showFullScreen();

	graspItGUI->getIVmgr()->setStereo(true);
/*
	StereoWindow* mStereoWindow = new StereoWindow();
	graspItGUI->getIVmgr()->setStereoWindow( mStereoWindow->viewerHolder );
	mStereoWindow->show();
	mStereoWindow->showFullScreen();
*/
}

void MainWindow::stereoOff()
{
	mUI->contactsGroupBox->show();
	mUI->qualityGroupBox->show();

	mUI->toolBar->show();
	mUI->Toolbar_2->show();
	mUI->ToolbarDynamics->show();
	mUI->graspToolbar->show();
	mUI->tendonToolbar->show();

	mWindow->menuBar()->show();
	DBGA("Stereo off");
	graspItGUI->getIVmgr()->getViewer()->setDecoration(true);
	mWindow->showNormal();
	graspItGUI->getIVmgr()->setStereo(false);
}

void MainWindow::stereoFlip()
{
	DBGA("Stereo flip");
	graspItGUI->getIVmgr()->flipStereo();
}

// --------------------------------- Misc menu ------------------------------------------------

void MainWindow::archBuilder()
{
	ArchBuilderDlg dlg(mWindow);
  
	if ( dlg.exec() != QDialog::Accepted) {
		return;
	}
  
	double innerRadius = dlg.innerRadiusEdit->text().toDouble();
	double outerRadius = dlg.outerRadiusEdit->text().toDouble();
	double thickness = dlg.thicknessEdit->text().toDouble();
	int nBlocks = dlg.numberBlocksBox->value();
	if (innerRadius <= 0 || outerRadius <= 0 || thickness <= 0 || nBlocks <= 1) {
		QMessageBox::warning(mWindow,"Error","Invalid parameters",
							 QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
		return;
	}
	bool addSupports = dlg.supportsCheckBox->isChecked();
	create_arch(world, innerRadius, outerRadius, thickness, nBlocks, addSupports);
}

void MainWindow::miscOptimizer()
{
	OptimizerDlg* dlg = new OptimizerDlg(world, mWindow);
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show();  
}

void MainWindow::miscEigengridsAction_activated()
{
#ifdef EIGENGRIDS
	EigenGridsDlg *dlg = new EigenGridsDlg(mWindow);
	dlg->setAttribute(Qt::WA_ShowModal, false);
	dlg->setAttribute(Qt::WA_DeleteOnClose, true);
	dlg->show(); 
#endif
}

/*!
  Gets the current contacts from each of the graspable bodies in the worlds,
  and adds the dynamic contact force acting at each one to the contact force
  list.  The force is listed as 3 values specifying the local x and y
  tangential forces (relative to the contact frame) and the z normal force.
*/
void MainWindow::updateContactsList()
{
  std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  int b,contactNum = 0;;
  double *contactForce;
  
  clearContactsList();
  //selectedContact = -1;
  for (b=0;b<world->getNumGB();b++) {
    contactList = world->getGB(b)->getContacts();
    
    for (cp=contactList.begin();cp!=contactList.end();cp++,contactNum++) {
      contactForce = (*cp)->getDynamicContactWrench();      
      mUI->contactsListBox->insertItem(QString("Contact %1:  force %2 %3 %4 torque %5 %6 %7").
        arg(contactNum+1). 
		arg(contactForce[0],5,'f',2).arg(contactForce[1],5,'f',2).arg(contactForce[2],5,'f',2). 
		arg(contactForce[3],5,'f',4).arg(contactForce[4],5,'f',4).arg(contactForce[5],5,'f',4));
    }
  }
}

/*!
  When a contact from the list is selected by the user, it will cause the
  corresponding arrow indicator in the scene to blink.  If a currently
  selected contact is chosen again, it is unselected and the blinking stops.
*/
void MainWindow::contactSelected(int newSelection)
{
  if (selectedContact == newSelection) {
    graspItGUI->getIVmgr()->unhilightObjContact(selectedContact);
    selectedContact = -1;
  }
  else {
    if (selectedContact >= 0) {
      graspItGUI->getIVmgr()->unhilightObjContact(selectedContact);
    }
    graspItGUI->getIVmgr()->hilightObjContact(newSelection);
    selectedContact = newSelection;
  }
}

/*!
  Removes all contacts from the contacts list.
*/
void MainWindow::clearContactsList()
{
  selectedContact = -1;
  mUI->contactsListBox->clear();
}

/*!
  Gets the simulation time from the world and displays it in the main window.
*/
void MainWindow::updateTimeReadout()
{
  QTime simTime;
  double seconds = world->getWorldTime();
  simTime = simTime.addMSecs(ROUND(seconds*1000));
  mUI->timeReadout->display(simTime.toString("mm:ss.zzz"));
  mUI->timeReadout->update();
}

/*!
  Toggles the state of the dynamics.  If dynamics are now paused it redraws
  and lists the last dynamic contact forces.  If dynamics have just been
  started or resumed, it clears the contact forces list.
*/
void MainWindow::toggleDynamics()
{
  if (world->dynamicsAreOn()) {
    world->turnOffDynamics();
    mUI->dynamicsPlayAction->setText("Start Simulation");
    graspItGUI->getIVmgr()->drawDynamicForces();
    updateContactsList();
  } else {
    world->turnOnDynamics();
    mUI->dynamicsPlayAction->setText("Pause Simulation");
    clearContactsList();
  }
}

/*!
  Clears the current list of quality mesure results and goes through
  the qm's of each grasp and writes out the latest results.
*/
void MainWindow::updateQualityList()
{
  QString valStr;
  Hand *h;
  Grasp *g;
  int numHands = world->getNumHands();
  
  mUI->qualityListBox->clear();
  
  for (int i=0;i<numHands;i++) {
    h = world->getHand(i);    
    g = h->getGrasp();
    
    mUI->qualityListBox->insertItem(h->getName());
    
    for (int j=0;j<g->getNumQM();j++) {
      valStr = "  " + g->getQM(j)->getName() +
        QString(": %1").arg(g->getQM(j)->evaluate(),6,'g',3);
      mUI->qualityListBox->insertItem(valStr);
    }
  }
}

/*!
  Changes the state of the dynamics button back to its off state and
  puts up an alert message box warning of a error occurring in the dynamics.
*/
void MainWindow::showDynamicsError( const char *errMsg )
{
  mUI->dynamicsPlayAction->setOn(false);
  QTWARNING(errMsg);
}

/*!
  Pushes the current dynamic state onto a world stack.
*/
void MainWindow::dynamicsPushState()
{
  world->pushDynamicState();
}

/*!
  Pops the top dyanmic state from the world and uses it to set the current
  world state.
*/
void MainWindow::dynamicsPopState()
{
  world->popDynamicState();
}

/*!
  Clears the combo box of materials and repopulates it with the current
  material names defined for this world.
*/
void MainWindow::updateMaterialBoxList()
{
  mUI->materialComboBox->clear();
  for (int i=0;i<world->getNumMaterials();i++)
    mUI->materialComboBox->insertItem(world->getMaterialName(i));
}

/*!
  Sets the material of any selected bodies, and updates all grasps.
*/
void MainWindow::materialSelected( int whichMat )
{
  if (whichMat >= 0 && whichMat < world->getNumMaterials()) {
    for (int i=0;i<world->getNumSelectedBodies();i++)
      world->getSelectedBody(i)->setMaterial(whichMat);
    world->updateGrasps();
  }
}

/*!
  Whenever a new body is selected, this show that body's material as the
  currently selected material in the combo box.  If more that one body is
  selected and they have different materials, a blank material is shown.
*/
void MainWindow::updateMaterialBox()
{
  int i;
  if (world->getNumSelectedBodies() > 0) {
    int firstMat = world->getSelectedBody(0)->getMaterial();
    for (i=1;i<world->getNumSelectedBodies();i++)
      if (world->getSelectedBody(i)->getMaterial() != firstMat) break;
    if (i==world->getNumSelectedBodies()) {
      mUI->materialComboBox->setCurrentItem(firstMat);
      if (mUI->materialComboBox->count() > world->getNumMaterials())
		mUI->materialComboBox->removeItem(world->getNumMaterials());
    }
    else {
      if (mUI->materialComboBox->count() == world->getNumMaterials())
		mUI->materialComboBox->insertItem(QString(" "));
      mUI->materialComboBox->setCurrentItem(world->getNumMaterials());
    }
  }
}

/*!
  This repopulates the current hand and grasped object combo boxes and sets
  the correct selection for each.
*/
void MainWindow::updateGraspBoxes()
{
	mUI->handSelectionBox->clear();
	for (int i=0; i<world->getNumHands(); i++) {
		mUI->handSelectionBox->insertItem(world->getHand(i)->getName());
		if (world->getCurrentHand() == world->getHand(i)) {
			mUI->handSelectionBox->setCurrentItem(i);
		}
		updateTendonNamesBox();
		world->deselectTendon();
	}
	mUI->graspedBodyBox->clear();
	if (world->getCurrentHand()) {
		for (int i=0;i<world->getNumGB();i++) {
			mUI->graspedBodyBox->insertItem(world->getGB(i)->getName());
			if (world->getCurrentHand()->getGrasp()->getObject() == world->getGB(i)) {
				mUI->graspedBodyBox->setCurrentItem(i);
			}
		}
	}
}

/*this sets drop-down list to reflect change in hand selection*/
void MainWindow::handleHandSelectionChange()
{
 int i;
 for (i=0;i<world->getNumHands();i++) {
  if (world->getCurrentHand() == world->getHand(i))
   mUI->handSelectionBox->setCurrentItem(i);
 }
  updateTendonNamesBox();
}

/*!
  When the user selects a graspbable body from the combo box, this
  sets the object that should be the focus of the current hand's grasp.
  It then updates the quality list.
*/
void MainWindow::selectGraspedBody(int sb)
{
  GraspableBody *b = world->getGB(sb);
  world->getHand(mUI->handSelectionBox->currentItem())->getGrasp()->setObject(b);
  updateQualityList();
}

/*!
  When the user chooses a new hand from the combo box, this sets the
  current hand for the world.
*/
void MainWindow::setCurrentHand( int sh )
{
  world->setCurrentHand(world->getHand(sh));
  updateGraspBoxes();

  updateTendonNamesBox();
  world->deselectTendon();
  //emits tendon selection changed
}

/*!
  Updates the state of the collisions button.
*/
void MainWindow::updateCollisionAction(bool state)
{
if (state) 
    mUI->elementCollisionToggleAction->setText("Collisions OFF");
  else
    mUI->elementCollisionToggleAction->setText("Collisions ON");
}

void MainWindow::handleTendonSelectionArea()
{
  if (!world->queryTendonSelected()) {
    mUI->tendonActiveForceLabel->setEnabled(false);
    mUI->TendonForceInput->setEnabled(false);
    mUI->tendonPassiveForceLabel->setEnabled(false);
    mUI->tendonPassiveForceEdit->setEnabled(false);
    mUI->tendonExcursionLabel->setEnabled(false);
    mUI->tendonExcursionEdit->setEnabled(false);
    mUI->tendonVisibleLabel->setEnabled(false);
    mUI->tendonVisibleCheckBox->setEnabled(false);
    mUI->forcesVisibleLabel->setEnabled(false);
    mUI->forcesVisibleCheckBox->setEnabled(false);
    if (mUI->tendonNamesBox->isEnabled() ) {
      mUI->tendonNamesBox->setCurrentItem ( mUI->tendonNamesBox->count() - 1 ); /* "none selected" */
    }
  } else {
    mUI->tendonActiveForceLabel->setEnabled(true);
    mUI->TendonForceInput->setEnabled(true);
    mUI->tendonPassiveForceLabel->setEnabled(true);
    mUI->tendonPassiveForceEdit->setEnabled(true);
    mUI->tendonExcursionLabel->setEnabled(true);
    mUI->tendonExcursionEdit->setEnabled(true);
    mUI->tendonVisibleLabel->setEnabled(true);
    mUI->tendonVisibleCheckBox->setEnabled(true);
    if (mUI->tendonVisibleCheckBox->isChecked()) {
      mUI->forcesVisibleLabel->setEnabled(true);
      mUI->forcesVisibleCheckBox->setEnabled(true);
    } else {
      mUI->forcesVisibleLabel->setEnabled(false);
      mUI->forcesVisibleCheckBox->setEnabled(false);
    }
    //there's got to be a better way to do this...
    for (int i=0; i<mUI->tendonNamesBox->count(); i++) {
      if ( mUI->tendonNamesBox->text(i)==world->getSelectedTendon()->getName() ) {
        mUI->tendonNamesBox->setCurrentItem(i);
        break;
      }
    }
    
    // this also triggers "valueChanged" so it sets the same value back to the tendon.
    // a bit strange, but should do no harm 
    // careful with conversion - reconversion from int though
    float getForce = world->getSelectedTendon()->getActiveForce();
    //the spin box works in Newtons; convert from graspit units
    mUI->TendonForceInput->setValue( int(getForce*1.0e-6) ); 
    
    handleTendonDetailsArea();
  }
}

void MainWindow::handleTendonDetailsArea()
{
  /*this synchronizes info on tendon on the GUI bar with actual valuea from selected tendon*/
  if (!world->queryTendonSelected())
    return;
  
  QString exc;
  exc.setNum( world->getSelectedTendon()->getExcursion() , 'f' , 1);
  mUI->tendonExcursionEdit->setText( exc );
  
  QString psf;
  float getForce = world->getSelectedTendon()->getPassiveForce() * 1.0e-6; //convert to Newtons
  psf.setNum( getForce , 'f' , 2);
  mUI->tendonPassiveForceEdit->setText( psf );
  
  mUI->tendonVisibleCheckBox->setChecked( world->getSelectedTendon()->isVisible() );
  mUI->forcesVisibleCheckBox->setChecked( world->getSelectedTendon()->forcesVisible() );
}

void MainWindow::TendonForceInput_valueChanged( int f)
{
  float newForce = (float)f * 1.0e6; //the spin box works in Newtons; convert to graspit units
  world->getSelectedTendon()->setActiveForce( newForce );
}

void MainWindow::tendonNamesBoxActivated( int i)
{
  if ( i<mUI->tendonNamesBox->count()-1) {
    //the last element in the list is "nothing selected"
    world->selectTendon(i);
  } else {
    world->deselectTendon();
  }
  //both of these automatically emit tendonSelectionChanged
}


void MainWindow::tendonVisibleCheckBox_toggled( bool vis)
{
  world->getSelectedTendon()->setVisible( vis );
  if (vis) {
    mUI->forcesVisibleLabel->setEnabled(true);
    mUI->forcesVisibleCheckBox->setEnabled(true);
  } else {
    mUI->forcesVisibleLabel->setEnabled(false);
    mUI->forcesVisibleCheckBox->setEnabled(false);
  }
}

void MainWindow::forcesVisibleCheckBox_toggled( bool vis)
{
  world->getSelectedTendon()->setForcesVisible( vis );
}

/*this populates tendon names box according to selected hand*/
void MainWindow::updateTendonNamesBox()
{
  int i, nrTendons = world->getCurrentHandNumberTendons();
  if (nrTendons==0) {
    mUI->tendonNamesBox->clear();
    mUI->Tendon_force_label->setEnabled(false);
    mUI->tendonNamesBox->setEnabled(false);
    return;
  }
  
  mUI->Tendon_force_label->setEnabled(true);
  mUI->tendonNamesBox->setEnabled(true); 
  mUI->tendonNamesBox->clear();
  for (i=0; i<nrTendons; i++) {
    mUI->tendonNamesBox->insertItem( world->getSelectedHandTendonName(i) );
  }
  mUI->tendonNamesBox->insertItem( QString("--none selected--") );
}
