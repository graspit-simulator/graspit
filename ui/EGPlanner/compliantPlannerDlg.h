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
// $Id: compliantPlannerDlg.h,v 1.8 2009/05/07 19:59:21 cmatei Exp $
//
//######################################################################

#ifndef _compliantplannerdlg_h_
#define _compliantplannerdlg_h_

#include <QDialog>
#include <list>
#include <fstream>

#include "ui_compliantPlannerDlg.h"

#include "matvec3D.h"

class ListPlanner;
class Hand;
class GraspableBody;
class GraspPlanningState;

//! Used for running tests with compliant hands
/*! This dialog is used for running various tests of the compliant hand
	solver framework. It is not finished, and currently not in a very
	well-engineered state. It is in bad need of a re-design.
*/
class CompliantPlannerDlg : public QDialog, public Ui::CompliantPlannerDlgUI
{
	Q_OBJECT
private:
	//! The planner the dialog is using
	ListPlanner *mPlanner;
	//! The hand we are planning on
	Hand *mHand;
	//! The object we are planning on
	GraspableBody *mObject;
	//! The number of candidate grasps that were sampled to be tried out
	int mNumCandidates;
	//! The current grasp displayed from the best list
	int mBestGraspNum;
	//! The reference position of the target object
	transf mObjectRefTran;
	//! The file (if any) that full results are written to
	std::fstream *mOut;

	//! For batch testing for hand design
	double mTFrom, mTTo, mTStep, mTR;
	double mSFrom, mSTo, mSStep, mSR;
	bool mBatch;

	//!
	void ellipsoidSampling(double a, double b, double c, double resolution);
	//!
	void boxSampling(double a, double b, double c, double res);
	//!
	void sampleFace(vec3 x, vec3 y, vec3 z, double sz1, double sz2, vec3 tln, double res,
					std::list<GraspPlanningState*> *sampling);

	//! Samples an ellipsoid using a grid-based method to generate pre-grasps.
	void gridEllipsoidSampling(const GraspPlanningState &seed,
						       std::list<GraspPlanningState*> *sampling, int samples);
	//! Helper function for the above
	void addCartesianSamples(const GraspPlanningState &seed, std::list<GraspPlanningState*> *sampling, 
							 int samples, double x, double y, double z);
	//! Initialization stuff
	void init();
	//! Display one grasp from the result list
	void showResult();
	void startPlanner();
public:
	CompliantPlannerDlg(Hand *h, GraspableBody *gb, QWidget *parent = 0) : 
	    QDialog(parent), mHand(h), mObject(gb) {
		setupUi(this);
		QObject::connect(generateButton, SIGNAL(clicked()), 
						 this, SLOT(generateButtonClicked()));
		QObject::connect(testButton, SIGNAL(clicked()), 
						 this, SLOT(testButtonClicked()));
		QObject::connect(testOneButton, SIGNAL(clicked()),
						 this, SLOT(testOneButtonClicked()));
		QObject::connect(nextButton, SIGNAL(clicked()), 
						 this, SLOT(nextButtonClicked()));
		QObject::connect(prevButton, SIGNAL(clicked()), 
						 this, SLOT(prevButtonClicked()));
		QObject::connect(bestButton, SIGNAL(clicked()), 
						 this, SLOT(bestButtonClicked()));
		QObject::connect(designTestButton, SIGNAL(clicked()), 
						 this, SLOT(designTestButtonClicked()));
		QObject::connect(showOneButton, SIGNAL(clicked()), 
						 this, SLOT(showOneButtonClicked()));
		QObject::connect(prepareOneButton, SIGNAL(clicked()), 
						 this, SLOT(prepareOneButtonClicked()));
		QObject::connect(visualMarkersBox, SIGNAL(clicked()), 
						 this, SLOT(visualMarkersBoxClicked()));
		QObject::connect(resetObjectButton, SIGNAL(clicked()), 
						 this, SLOT(resetObjectButtonClicked()));
		QObject::connect(resultsFileEdit, SIGNAL(editingFinished()),
						 this, SLOT(updateOut()));
		QObject::connect(resultsBox, SIGNAL(activated(int)), 
						 this, SLOT(updateOut()));
		init();
	}
	~CompliantPlannerDlg();

public slots:
	void generateButtonClicked();
	void testButtonClicked();
	void testOneButtonClicked();
	void nextButtonClicked();
	void prevButtonClicked();
	void bestButtonClicked();
	void update();
	void plannerFinished();
	void showOneButtonClicked();
	void prepareOneButtonClicked();
	void visualMarkersBoxClicked();
	void resetObjectButtonClicked();
	void designTestButtonClicked();
	void updateOut();
};

#endif
