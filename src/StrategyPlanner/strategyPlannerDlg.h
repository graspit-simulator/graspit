/*
 * strategyPlannerDlg.h
 *
 *  Created on: Jun 11, 2011
 *      Author: dang
 */
#pragma once
#ifndef STRATEGY_PLANNER_H_
#define STRATEGY_PLANNER_H_

#include "ui_StrategyPlannerDlg.h"


class StrategyPlannerDlg : public QDialog, public Ui::StrategyPlannerDlgUI
{
	Q_OBJECT
private:
public:
	StrategyPlannerDlg(QWidget *parent = 0) : QDialog(parent)
	{
		setupUi(this);
		QObject::connect(planButton, SIGNAL(clicked()), this, SLOT(planButton_clicked()));
		//QObject::connect(compareButton, SIGNAL(clicked()), this, SLOT(compareButton_clicked()));
		//QObject::connect(previousNNButton, SIGNAL(clicked()), this, SLOT(previousNNButton_clicked()));
		//QObject::connect(nextNNButton, SIGNAL(clicked()), this, SLOT(nextNNButton_clicked()));
		//QObject::connect(adjustButton, SIGNAL(clicked()), this, SLOT(adjustButton_clicked()));
		//QObject::connect(applyAdjustmentButton, SIGNAL(clicked()), this, SLOT(applyAdjustmentButton_clicked()));
		//QObject::connect(loadObjectButton, SIGNAL(clicked()), this, SLOT(loadObjectButton_clicked()));
		//QObject::connect(loadPoseButton, SIGNAL(clicked()), this, SLOT(loadPoseButton_clicked()));
		//QObject::connect(setPoseButton, SIGNAL(clicked()), this, SLOT(setPoseButton_clicked()));
		//QObject::connect(allInOneAdjustButton, SIGNAL(clicked()), this, SLOT(allInOneAdjustButton_clicked()));
		//QObject::connect(testButton, SIGNAL(clicked()), this, SLOT(testButton_clicked()));
		//QObject::connect(rotateButton, SIGNAL(clicked()), this, SLOT(rotateButton_clicked()));
		//QObject::connect(generateTestPoseButton, SIGNAL(clicked()), this, SLOT(generateTestPoseButton_clicked()));
		//QObject::connect(gotoTestPoseButton, SIGNAL(clicked()), this, SLOT(gotoTestPoseButton_clicked()));

		//mGeb = new GraspExperienceBase();
	}

public slots:
	void planButton_clicked();

};

#endif /* STRATEGY_PLANNER_H_ */
