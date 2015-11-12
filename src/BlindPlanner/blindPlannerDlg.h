/*
* blindPlannerDlg.h
*
*  Created on: Jun 11, 2011
*      Author: dang
*/
#pragma once
#ifndef BLINDPLANNERDLG_H_
#define BLINDPLANNERDLG_H_

#include "ui_blindPlannerDlg.h"

#include <QDialog>

#include "graspExperienceBase.h"
#include "graspExperienceEntry.h"
#include "handAdjust.h"
#include "matvec3D.h"
#include <Inventor/sensors/SoTimerSensor.h>
#include "debug.h"

class QualEpsilon;
class QualVolume;
class QualityEstimator;
class HandControlAPI;

class BlindPlannerDlg : public QDialog, public Ui::BlindPlannerDlgUI
{
	Q_OBJECT
private:
	GraspExperienceBase * mGeb;
	std::vector<GraspExperienceEntry> mCurrentNNList;
	int mCurrentNNIndex;
	int mK;
	GraspExperienceEntry mCurrentGrasp;
	HandAdjust mHA;
	//the adjustment we need
	transf mT;
	double mSpread;
	//used to update the command
	//SoTimerSensor *mTimerSensor;
	//list of pose error
	std::vector<transf> mPoseError;
	transf mIdealGraspPose;

	//for test use
	//! Epsilon quality measure
	QualEpsilon* mEpsQual;
	//! Volume quality measure
	QualVolume* mVolQual;

	static QualityEstimator* mQualityEstimator;
	static HandControlAPI* mHandControlAPI;

	static int mExpId;
	std::vector<transf> mTestPose;
	transf mOriginalPose;
	int mCurrentPerturbationIndex;
	std::vector<transf> mPerturbationList;

	void updateStatus();
	void testStrategy();

public:
	BlindPlannerDlg(QWidget *parent = 0) : QDialog(parent), mGeb(NULL), mCurrentNNIndex(0), mK(20), mEpsQual(NULL), mVolQual(NULL), mSpread(-1)
	{
		setupUi(this);
		QObject::connect(loadExperienceButton, SIGNAL(clicked()), this, SLOT(loadExperienceButton_clicked()));
		QObject::connect(compareButton, SIGNAL(clicked()), this, SLOT(compareButton_clicked()));
		QObject::connect(previousNNButton, SIGNAL(clicked()), this, SLOT(previousNNButton_clicked()));
		QObject::connect(nextNNButton, SIGNAL(clicked()), this, SLOT(nextNNButton_clicked()));
		QObject::connect(adjustButton, SIGNAL(clicked()), this, SLOT(adjustButton_clicked()));
		QObject::connect(applyAdjustmentButton, SIGNAL(clicked()), this, SLOT(applyAdjustmentButton_clicked()));
		QObject::connect(loadObjectButton, SIGNAL(clicked()), this, SLOT(loadObjectButton_clicked()));
		QObject::connect(loadPoseButton, SIGNAL(clicked()), this, SLOT(loadPoseButton_clicked()));
		QObject::connect(setPoseButton, SIGNAL(clicked()), this, SLOT(setPoseButton_clicked()));
		QObject::connect(allInOneAdjustButton, SIGNAL(clicked()), this, SLOT(allInOneAdjustButton_clicked()));
		QObject::connect(testButton, SIGNAL(clicked()), this, SLOT(testButton_clicked()));
		QObject::connect(rotateButton, SIGNAL(clicked()), this, SLOT(rotateButton_clicked()));
		QObject::connect(generateTestPoseButton, SIGNAL(clicked()), this, SLOT(generateTestPoseButton_clicked()));
		QObject::connect(gotoTestPoseButton, SIGNAL(clicked()), this, SLOT(gotoTestPoseButton_clicked()));
		QObject::connect(perturbButton, SIGNAL(clicked()), this, SLOT(perturbButton_clicked()));
		QObject::connect(recordDistButton, SIGNAL(clicked()), this, SLOT(recordDistButton_clicked()));
		QObject::connect(recordTargetNNQualityButton, SIGNAL(clicked()), this, SLOT(recordTargetNNQualityButton_clicked()));
		QObject::connect(distanceButton, SIGNAL(clicked()), this, SLOT(distanceButton_clicked()));
		QObject::connect(writeoutButton, SIGNAL(clicked()), this, SLOT(writeoutButton_clicked()));
		QObject::connect(sendToRobotButton, SIGNAL(clicked()), this, SLOT(sendToRobotButton_clicked()));
		QObject::connect(generatePerturbationListButton, SIGNAL(clicked()), this, SLOT(generatePerturbationListButton_clicked()));
		QObject::connect(previousPerturbationButton, SIGNAL(clicked()), this, SLOT(previousPerturbationButton_clicked()));
		QObject::connect(nextPerturbationButton, SIGNAL(clicked()), this, SLOT(nextPerturbationButton_clicked()));
		QObject::connect(zeroPerturbationButton, SIGNAL(clicked()), this, SLOT(zeroPerturbationButton_clicked()));
		QObject::connect(loadPoseErrorButton, SIGNAL(clicked()), this, SLOT(loadPoseErrorButton_clicked()));
		QObject::connect(generatePoseErrorButton, SIGNAL(clicked()), this, SLOT(generatePoseErrorButton_clicked()));
		QObject::connect(showGraspWithPoseErrorButton, SIGNAL(clicked()), this, SLOT(showGraspWithPoseErrorButton_clicked()));
		QObject::connect(showIdealGraspButton, SIGNAL(clicked()), this, SLOT(showIdealGraspButton_clicked()));
		QObject::connect(outputPerturbedPoseButton, SIGNAL(clicked()), this, SLOT(outputPerturbedPoseButton_clicked()));
		QObject::connect(dynamicExamButton, SIGNAL(clicked()), this, SLOT(dynamicExamButton_clicked()));
		QObject::connect(autoGraspAtFirstContactButton, SIGNAL(clicked()), this, SLOT(autoGraspAtFirstContact_clicked()));
		//QObject::connect(loadSVMButton, SIGNAL(clicked()), this, SLOT(loadSVMButton_clicked()));
		//QObject::connect(loadCodebookButton, SIGNAL(clicked()), this, SLOT(loadCodebookButton_clicked()));
		QObject::connect(SVMEvalButton, SIGNAL(clicked()), this, SLOT(SVMEvalButton_clicked()));
		



		mGeb = new GraspExperienceBase();

		//mTimerSensor = new SoTimerSensor(sensorCB, this);
		//mTimerSensor->setInterval( SbTime( 1.0 ));
		//mTimerSensor->schedule();

#ifdef JARED_AUTO_RUNNING
        //load stuff
        loadExperienceButton_clicked();
        loadPoseButton_clicked();
        DBGA("=====>JARED_AUTO_RUNNING is set");
#endif
	}

	//~BlindPlannerDlg(){mTimerSensor->unschedule();}


	void recordSVMQuality(std::string surfix = "", bool newLine = false);
	void recordQuality(std::string surfix = "", bool newLine = false);
	void recordHandPose(std::string surfix = "", bool newLine = false);
	void recordDistance(std::string surfix = "", bool newLine = false);
	void recordTargetNNQuality(std::string surfix = "", bool newLine = false);

	//get the smallest distance from the current grasp to the grasps in the grasp experience database
	//this is valid only after compareButton_clicked is triggered
	double getCurrentDistanceToGEDB();

	//is the adjustment just applied an identity one?
	bool isAnIdentityAdjustment();

	//get the qualities of the original grasp with which we have found the most similar match through perturbation
	std::vector<double> getTargetGraspQuality();
	GraspExperienceEntry getTargetGrasp();

	double qualityEstimate();

	
	//static void sensorCB(void *data,SoSensor*);

	public slots:
		void loadExperienceButton_clicked();
		void compareButton_clicked();
		void previousNNButton_clicked();
		void nextNNButton_clicked();
		void adjustButton_clicked();
		void applyAdjustmentButton_clicked();
		void loadObjectButton_clicked();
		void loadPoseButton_clicked();
		void setPoseButton_clicked();
		void allInOneAdjustButton_clicked();
		void testButton_clicked();
		void rotateButton_clicked();
		void generateTestPoseButton_clicked();
		void gotoTestPoseButton_clicked();
		void perturbButton_clicked();
		void recordDistButton_clicked(){recordDistance();};
		void recordTargetNNQualityButton_clicked(){recordTargetNNQuality();};
		void distanceButton_clicked();
		void writeoutButton_clicked();
		void sendToRobotButton_clicked();
		void generatePerturbationListButton_clicked();
		void previousPerturbationButton_clicked();
		void nextPerturbationButton_clicked();
		void zeroPerturbationButton_clicked();
		void loadPoseErrorButton_clicked();
		void generatePoseErrorButton_clicked();
		void showGraspWithPoseErrorButton_clicked();
		void showIdealGraspButton_clicked();
		void outputPerturbedPoseButton_clicked();
		void dynamicExamButton_clicked();
		void autoGraspAtFirstContact_clicked();
		//void loadCodebookButton_clicked();
		//void loadSVMButton_clicked();
		void SVMEvalButton_clicked();
};

#endif /* BLINDPLANNERDLG_H_ */
