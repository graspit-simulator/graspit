//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mainWindow.h,v 1.13 2009/09/14 18:07:18 hao Exp $
//
//######################################################################

//
// $Header: 
//

/*! \file 
  \brief Implements the UI functions and slots for the main window. 
 */

/*!
  \class MainWindow
  \brief Main user interface, includes menus, tools, widget for viewer, etc. 
  
  This class is derived from the QT mainWindow widget.  It creates
  the main application window which includes the menu bar, tool bar, main
  viewer window, quality measure result box, and the contacts list box.  

*/

#include "ui_mainWindow.h"

#include <Q3MainWindow>
//#include <QMainWindow>
#include <QString>
#include <QObject>

class World;
class QAction;
class QAssistantClient;
class Grasp;

class MainWindow : public QObject
{
	Q_OBJECT
private:
	//QAssistantClient *assistantClient;
    World *world;
    QString fileName;
    int selectedContact;

	void init();
	void destroy();
	void destroyChildren();
public:
	Ui::MainWindowUI *mUI;
	Q3MainWindow *mWindow;

	MainWindow(QWidget *parent = 0);
	virtual ~MainWindow(){
		destroy();
		delete mWindow;
		delete mUI;
	}
	void setMainWorld( World *w );
    void startTapTap();
public slots:
	//-------------
	void fileNew();
	void fileOpen();
	void fileSave();
	void fileSaveAs();
	void fileImportRobot();
	void fileImportObstacle();
	void fileImportObject();
	void fileEditSettings();
	void fileExit();
	void fileSaveImage();
	int saveAndContinue(const QString & action);
	//-------------
	void helpManual();
	void helpAbout();
	void helpAboutQT();
	//-------------
	void setTool( QAction *a );
	void elementTurnOffCollisions();
	void elementBodyProperties();
	void elementPrimitives();
	void updateElementMenu();
	//-------------
	void graspAutoGrasp();
	void graspAutoOpen();
	void graspQualityMeasures();
	void graspCreateProjection(Grasp* g = NULL);
	void graspForceOptimization();
	void graspPlanner();
	void updateGraspMenu();
	//-------------
	void eigenGraspActivated();
	void graspContactExaminer_activated();
	void eigenGraspPlannerActivated();
	void graspCompliantPlanner();
	//-------------
	void dbaseGUIAction_activated();
	void dbasePlannerAction_activated();
	void graspCapture();
	//-------------
	void sensorsSensor_InputAction_activated();
	void sensorsBarrettHandAction();
	//-------------
	void stereoOn();
	void stereoOff();
	void stereoFlip();
	//-------------
	void archBuilder();
	void miscOptimizer();
	void miscEigengridsAction_activated();
	void miscArizonaProjectDlg_activated();
	void misStaubliControlDlg();
	void misDofControlDlg();
	void misTaskControlDlg();
	void misBlindPlannerDlg();
	void misUncertaintyPlannerDlg();
	void misStrategyPlannerDlg();
	//-------------
	void updateContactsList();
	void contactSelected(int newSelection);
	void clearContactsList();
	void updateQualityList();
	//--------------
	void updateTimeReadout();
	void toggleDynamics();
	void showDynamicsError( const char *errMsg );
	void dynamicsPushState();
	void dynamicsPopState();
	//--------------
	void updateMaterialBoxList();
	void materialSelected( int whichMat );
	void updateMaterialBox();
	void updateGraspBoxes();
	void handleHandSelectionChange();
	void selectGraspedBody(int sb);
	void setCurrentHand( int sh );
	void updateCollisionAction(bool state);
	//---------------
	void updateTendonNamesBox();
	void handleTendonSelectionArea();
	void handleTendonDetailsArea();
	void TendonForceInput_valueChanged( int f);
	void tendonNamesBoxActivated( int i);
	void tendonVisibleCheckBox_toggled( bool vis);
};
