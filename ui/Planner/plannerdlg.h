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

/*! \file
  \brief Implements the PlannerDlg, the grasp planner dialog box. 
*/

//Added by qt3to4:
#include <QTextStream>
#include <QDialog>
#include <QFile>
#include "ui_plannerdlg.h"

class grasp_manager;

/*! \class PlannerDlg
  \brief Creates and controls the grasp planner dialog box.

  The dialog box is split in two parts.  The first half has parameter
  settings for the grasp generator (planner), and the second half has
  settings for the grasp tester.  The user can either have the planner
  automatically generate the candidate grasps or they can be read from
  file.  If the planner will generate the grasps, the user can either
  choose automatic sampling where they just chose a density factor to
  roughly control the total number of candiates, or they can fine tune the
  number of samples in each direction using the text entry boxes.

  The tester side of the box has parameters for how many steps back along
  the grasp approach vector the palm should be moved in search of a force
  closure grasp, and how big each step should be.  The user must also select
  a quality measure for evaluating each grasp.  A "New" button allows the
  user to add new quality measures directly from this dialog box.  If the
  visualize process box is checked, the entire testing process will be
  rendered.  This allows the user to see what is going on, but slows down
  the testing.  The user can also choose a file in which to save the
  results of the testing.

  After setting the desired parameter values and choosing a quality measure,
  the user can click the generate button.  After the grasp candidates are
  generated, the test button can then be clicked, and after that is completed
  the show button is enabled.  The show button allows the user to see
  each of the best grasps found during testing.  Each successive click of
  the button causes the next grasp to be shown.
*/

class PlannerDlg : public QDialog, public Ui::PlannerDlgUI
{
	Q_OBJECT
private:
    QTextStream stream;
    QFile masterFile;
    grasp_manager *myGraspManager;

	void init();
	void destroy() ;
public:
	PlannerDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	~PlannerDlg(){destroy();}
public slots:
	void generateGrasps();
	void showGrasp();
	void newQM();
	void chooseFile();
	void chooseSaveFile();
	void testGrasps();
	void enableShowButton();
};
