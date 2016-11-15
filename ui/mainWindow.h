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
class Grasp;

class MainWindow : public QObject
{
	Q_OBJECT
private:
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
        World* getMainWorld(){return world;}                                     

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
	void forcesVisibleCheckBox_toggled( bool vis);
};
