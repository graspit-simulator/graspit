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
// Authors: Steffen Knoop
//          Andrew T. Miller 
//
// $Id: 
//
//######################################################################

#include "plannerdlg.h"

#include <QLineEdit>
#include <QFileDialog>
#include <QValidator>

#include "robot.h"
#include "mainWindow.h"
#include "graspitGUI.h"
#include "grasp_manager.h"
#include "grasp_tester.h"
#include "grasp_planner.h"
#include "grasp.h"
#include "world.h"
#include "ivmgr.h"
#include "quality.h"
#include "mytools.h"

/*!
  First this creates a new grasp_manager and gets the default planning and
  testing parameters.  Then it sets up number validators for the text entry
  boxes.  Finally it populates the quality measure comboBox with the names
  of the currently defined quality measures for this grasp.  If there are
  no quality measures defined, the generate button is disabled, but the user
  can add QM's by pressing the new button in this dialog box.
*/
void PlannerDlg::init()
{
    Grasp *grasp;
    int a,b,c,d,e;
    int density;
    double f;
    int i;
   
  myGraspManager = new grasp_manager;
  myGraspManager->get_graspPlanner()->get_planningParameters(a,b,c,d);
  myGraspManager->get_graspTester()->get_testingParameters(e,f);
  density = myGraspManager->get_graspPlanner()->get_parameterMode();
  
  densityFactorLine->setText(QString::number(density));
  densityFactorLine->setValidator(new QIntValidator(1,100,this));

  num360stepsLine->setText(QString::number(a));
  num360stepsLine->setValidator(new QIntValidator(1,999,this));
  
  numParPlanesLine->setText(QString::number(b));
  numParPlanesLine->setValidator(new QIntValidator(1,999,this));
  
  num180graspsLine->setText(QString::number(c));
  num180graspsLine->setValidator(new QIntValidator(1,999,this));
  
  numGraspRotsLine->setText(QString::number(d));
  numGraspRotsLine->setValidator(new QIntValidator(1,999,this));
    
  maxNumStepsLine->setText(QString::number(e));
  maxNumStepsLine->setValidator(new QIntValidator(1,999,this));
  
  backstepSizeLine->setText(QString::number(f));
  backstepSizeLine->setValidator(new QDoubleValidator(0,1000,6,this));
  
  qmComboBox->clear();
  grasp = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
  if (grasp->getNumQM() == 0) GenerateButton->setEnabled(false);
  else {
      for (i=0;i<grasp->getNumQM();i++)
        qmComboBox->insertItem(grasp->getQM(i)->getName());
  }
}

/*!
  Deletes the grasp_manager.
*/
void PlannerDlg::destroy() 
{
  delete myGraspManager;
  if (masterFile.isOpen()) masterFile.close();
}

/*!
  Reads the planning and testing parameters from the text entry boxes.
  If the automatic sampling box is checked or the input filename box is
  empty, this calls the planner to generate a set of candidate grasps.
  Otherwise it reads the candidate grasps from a text file.  The test button
  is enabled, and a connection is set up so that after testing is completed
  the show button will be enabled.
*/
void PlannerDlg::generateGrasps()
{
    int a,b,c,d,e;
    double f; 
    int density;
    
    if (automaticCheckBox->isChecked()) {
      density = densityFactorLine->text().toInt(); 
      myGraspManager->get_graspPlanner()->set_parameterMode(density);
  }
    else {
      a = num360stepsLine->text().toInt();
      b = numParPlanesLine->text().toInt();
      c = num180graspsLine->text().toInt();
      d = numGraspRotsLine->text().toInt();
      myGraspManager->get_graspPlanner()->set_parameterMode(0);
      myGraspManager->get_graspPlanner()->set_planningParameters(a,b,c,d);
  }
    e = maxNumStepsLine->text().toInt();
    f = backstepSizeLine->text().toDouble();
    
    myGraspManager->get_graspTester()->set_testingParameters(e,f);
    myGraspManager->get_graspTester()->
	    useQM(qmComboBox->currentItem());
    myGraspManager->set_render(visualizeBox->isChecked());
    
    if (automaticCheckBox->isChecked() || filenameLineEdit->text().isEmpty()) {
	myGraspManager->generateGrasps();
	QObject::connect(myGraspManager->get_graspTester(),SIGNAL(testingComplete()),
			     this,SLOT(enableShowButton()));
      }
    else {
	  if (filenameLineEdit->text().contains("master.txt")) {
	    masterFile.setName(filenameLineEdit->text());
	    if (!masterFile.open(QIODevice::ReadOnly)) {
	      QTWARNING("Could not open master grasp file");
	      return;
	    }
	    stream.setDevice( &masterFile );
	    
	    QObject::connect(myGraspManager->get_graspTester(),SIGNAL(testingComplete()),
			     this,SLOT(testGrasps()));
	  }
	  else if (myGraspManager->
	      readCandidateGraspsFile(filenameLineEdit->text())) {
	    QTWARNING("Could not read grasps from file.");
	    QObject::connect(myGraspManager->get_graspTester(),SIGNAL(testingComplete()),
			     this,SLOT(enableShowButton()));
	    return;
	}
    }
       TestButton->setEnabled(true);
}

/*!
  Calls on the grasp_manager to show the next planned grasp.
*/
void PlannerDlg::showGrasp()
{
  myGraspManager->showGrasps(1);
}

/*!
  When the user clicks the new button, a quality measure dialog box
  ( QMDlg ) is created.  If on the return from that, there is at least one
  QM defined, the generate button is enabled, and the QM comboBox is populated
  with the currently defined quality measures.
*/
void PlannerDlg::newQM()
{
  Grasp *grasp=graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();
  graspItGUI->getMainWindow()->graspQualityMeasures();
  if (grasp->getNumQM()>0) GenerateButton->setEnabled(true);
  qmComboBox->clear();
  for (int i=0;i<grasp->getNumQM();i++)
      qmComboBox->insertItem(grasp->getQM(i)->getName());
}

/*!
  Opens a file selection dialog box, with a text file (.txt) filter, to allow
  the user to choose a grasp candidates file.
*/
void PlannerDlg::chooseFile()
{
    QString fn(QFileDialog::getOpenFileName(this, QString(), QString(getenv("GRASPIT")), "Text Files (*.txt)") );
    if ( !fn.isEmpty() )
      filenameLineEdit->setText(fn); 
}

/*!
  Opens a file selection dialog box, with a text file (.txt) filter, to allow
  the user to choose a results 
*/
void PlannerDlg::chooseSaveFile()
{
    QString fn( QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT")), "Text Files (*.txt)") );
   if ( !fn.isEmpty() )
      savefileLineEdit->setText(fn); 
}

/*!
  This calls on the grasp_manager to start the testing process.  If the
  results are to be saved, the filename is also provided to the grasp_tester.
*/
void PlannerDlg::testGrasps()
{
  QString planFilename,quadFilename;
  World *world = graspItGUI->getIVmgr()->getWorld();
 /*  
  if (TestButton->text() == "Pause") {
	TestButton->setText("Continue");
	myGraspManager->get_graspTester()->pauseTests();
	return;
  }
  
  if (TestButton->text() == "Continue") {
	TestButton->setText("Pause");
	myGraspManager->get_graspTester()->continueTests();
	return;
  }
*/
 if (!masterFile.isOpen()) {
   if (!savefileLineEdit->text().isEmpty())
       myGraspManager->get_graspTester()->
          saveGraspsToFile(savefileLineEdit->text(),false);
   
   //TestButton->setText("Pause");
    myGraspManager->testGrasps();
  }
 else {  // This is just a hack for a project we are working on
   
   if (stream.atEnd()) {
     masterFile.close();
     return;
   }
     
   if (!savefileLineEdit->text().isEmpty())
     myGraspManager->get_graspTester()->
        saveGraspsToFile(savefileLineEdit->text(),true);
  
   if (world->getNumGB() > 0)
    world->destroyElement(world->getGB(0));
   stream >> quadFilename >> planFilename; stream.readLine();
//     if (quadFilename.stripWhiteSpace().isEmpty() || planFilename.stripWhiteSpace().isEmpty())
     
   world->importBody("GraspableBody",quadFilename);
   world->getHand(0)->getGrasp()->setObject(world->getGB(0));
     
   QFile logfile("grasplog.txt");
	if (logfile.open(QIODevice::WriteOnly | QIODevice::Append)) {
		QTextStream lout(&logfile);
		lout << "Evaluating grasps of "<<
		  world->getGB(0)->getName() << endl;
		logfile.close();
	}

	if (myGraspManager->
	  readCandidateGraspsFile(planFilename)) {
	  QTWARNING("Could not read grasps from file.");
	  masterFile.close();
	  return;
	}
    
	myGraspManager->testGrasps();
  }

 //TestButton->setText("Test");
}

/*!
  Enables the show button in the dialog box.
*/
void PlannerDlg::enableShowButton()
{
  ShowButton->setEnabled(true);
}
