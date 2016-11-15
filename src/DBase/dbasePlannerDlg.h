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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: dbasePlannerDlg.h,v 1.8 2009/07/02 21:07:11 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %DBasePlannerDlg class
 */


#ifndef _DBASEPLANNERDLG_H_
#define _DBASEPLANNERDLG_H_

#include "ui_dbasePlannerDlg.h"
#include <utility>
#include <QDialog>

#include "DBPlanner/neighbor_finder.h"
#include "DBPlanner/aligner.h"
#include "DBPlanner/grasp_ranker.h"
#include "DBPlanner/db_manager.h"

namespace db_planner {
	class DatabaseManager;
	class Model;
	class Grasp;
}

class Hand;
class GraspitDBPlanner;
class GraspableBody;
class GraspitDBModel;
class GraspPlanningState;

class DBasePlannerDlg : public QDialog, public Ui::DBasePlannerDlgUI
{
	Q_OBJECT	
private:
	//! Database manager that talks with the database
	db_planner::DatabaseManager *mDBMgr;
	//! Instance of neighbor finder
	db_planner::NeighborFinder<db_planner::Model> *mNeighborFinder;
	//! Grasp ranker that ranks a bag of grasps
	db_planner::GraspRanker *mGraspRanker;
	//! Aligner that does alignments between two models
	db_planner::Aligner<db_planner::Model> *mAligner;
	//! Planner that does the kernel test for grasp planning
	GraspitDBPlanner* mPlanner;
	//! A simple table that records the neighbors and the distances from current model
	std::vector<std::pair<db_planner::Model*, double> >mNeighbors;
	//! A vector that stores the original grasps from neighbors
	std::vector<db_planner::Grasp*>mOriginalGrasps;
	//! A vector that stores the tested grasps after test all is called
	std::vector<db_planner::Grasp*>mTestedGrasps;
	//! A pointer to the model to be planned
	db_planner::Model *mPlanningModel;
	//! A pointer to the model shown in GraspIt world
	db_planner::Model *mModelShown;
	//! A pointer to the hand involved in the current grasps
	Hand* mHand;
	//! Index of the current grasp in mOriginalGrasps
	int mCurrentOriginalGrasp;
	//! Index of the current grasp in mTestedGrasps
	int mCurrentTestedGrasp;
	//! Widget for displaying thumbnails
	QGraphicsScene * mModelScene;
	//! Helper variable that indicates whether the neighbor combo box is in reconstruction
	bool neighborComboBoxInReconstruction;

	void init();
	void destroy();
	void initializeDistanceComboBox(std::vector<string>);
	void updateNeighborList();
	void updateModelImage(db_planner::Model*);
	void updateOriginalGraspInfo();
	void updateTestedGraspInfo();
	void show3DObject(bool isNbr = false);

	template <class vectorType, class treatAsType>
	void deleteVectorElements(std::vector<vectorType>& v);

	void previousGrasp(int& i, std::vector<db_planner::Grasp*> graspList);
	void nextGrasp(int& i, std::vector<db_planner::Grasp*> graspList);
	void showGrasp(db_planner::Grasp* grasp);
	void setGroupBoxEnabled(bool nbrGen, bool alignment, bool ranking, bool grasp, bool execute);

public:
	DBasePlannerDlg(QWidget *parent = 0, db_planner::DatabaseManager* dbm = NULL, 
				    db_planner::Model* m = NULL, Hand* h = NULL) :
					QDialog(parent), mDBMgr(dbm), mPlanningModel(m), mModelShown(m), 
					mHand(h), mModelScene(NULL), mCurrentOriginalGrasp(0), 
					mCurrentTestedGrasp(0), neighborComboBoxInReconstruction(false), mAligner(NULL) {
		setupUi(this);
		QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButton_clicked()));
		QObject::connect(getNeighborButton, SIGNAL(clicked()), this, SLOT(getNeighborButton_clicked()));
		QObject::connect(executeButton, SIGNAL(clicked()), this, SLOT(executeButton_clicked()));
		QObject::connect(retrieveGraspsButton, SIGNAL(clicked()), this, SLOT(retrieveGraspsButton_clicked()));
		QObject::connect(rankGraspsButton, SIGNAL(clicked()), this, SLOT(rankGraspsButton_clicked()));
		QObject::connect(previousGraspButton, SIGNAL(clicked()), this, SLOT(previousGraspButton_clicked()));
		QObject::connect(nextGraspButton, SIGNAL(clicked()), this, SLOT(nextGraspButton_clicked()));
		QObject::connect(createGWSButton, SIGNAL(clicked()), this, SLOT(createGWSButton_clicked()));
		QObject::connect(neighborComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(modelChanged()));
		QObject::connect(alignmentMethodComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(alignmentChanged()));
		QObject::connect(seeNeighborCheckBox, SIGNAL(stateChanged(int)), this, SLOT(neighborCheckBoxChanged()));
		QObject::connect(originalGraspRadioButton, SIGNAL(clicked()), this, SLOT(originalGraspRadioButton_clicked()));
		QObject::connect(testedGraspRadioButton, SIGNAL(clicked()), this, SLOT(testedGraspRadioButton_clicked()));
		init();
	}
	~DBasePlannerDlg(){destroy();}

public slots:
	//! Button events
	void exitButton_clicked();
	void getNeighborButton_clicked();
	void executeButton_clicked();
	void retrieveGraspsButton_clicked();
	void rankGraspsButton_clicked();
	void previousGraspButton_clicked();
	void nextGraspButton_clicked();
	void createGWSButton_clicked();
	void originalGraspRadioButton_clicked();
	void testedGraspRadioButton_clicked();

	//! Trigger events
	void modelChanged();
	void neighborCheckBoxChanged();
	void alignmentChanged();
};

#endif