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
// Author(s): Maximilian Haas-Heger
//
// $Id: gsaDlg.h,v 1.3 2009/08/17 22:17:32 cmatei Exp $
//
//######################################################################

#ifndef _gsadlg_h_
#define _gsadlg_h_

#include "ui_gsaDlg.h"

class Hand;
class MainWindow;
class GraspSolver;
class QHBoxLayout;
struct UIParamT;

//! Provides an interface for running Grasp Stability Analysis routines
/*! This dialog allows running some of the Grasp Stability Analysis routines
  and display the results for visual inspection. 
*/
class GSADlg : public QDialog, public Ui::GSADlgUI
{
    Q_OBJECT
  private:
    //! A pointer to the main window used to ask for an update of the contact list
    MainWindow *mMainWindow;

    //! The hand that is the target of the optimization
    Hand *mHand;

    //! Structure holding pointers to UI items
    UIParamT *mParams;

    //! Grasp Solver object that is used to analyze grasp stability
    GraspSolver *mGraspSolver;

    //! Widget for settings area
    QWidget *mSettingsArea;

    //! Computes the default actuator preloads
    std::vector<double> getDefaultPreload();

    //! Build the settings area
    void buildSettingsArea();

    //! Build the solver settings area
    void refinementSettingsArea(QHBoxLayout *hl);
    void iterativeSettingsArea(QHBoxLayout *hl);

    //! Build the task specific settings area
    void forcesSettingsArea(QHBoxLayout *hl);
    void maxWrenchSettingsArea(QHBoxLayout *hl);
    void optPreloadSettingsArea(QHBoxLayout *hl);
    void resMapSettingsArea(QHBoxLayout *hl);

    void init();

  public:
    GSADlg(MainWindow *mw, Hand *h, QWidget *parent = 0);
    ~GSADlg();
  public Q_SLOTS:
    void solverTypeSelected(const QString &typeStr);
    void solveForSelected(const QString &typeStr);
    void solveButtonClicked();
    void countAxes();
};

#endif
