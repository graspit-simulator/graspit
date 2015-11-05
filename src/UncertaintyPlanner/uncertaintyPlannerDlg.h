/*
 * UncertaintyPlannerDlg.h
 *
 *  Created on: Jun 24, 2011
 *      Author: dang
 */
#pragma once
#ifndef UNCERTAINTYPLANNERDLG_H_
#define UNCERTAINTYPLANNERDLG_H_

#include "ui_uncertaintyPlannerDlg.h"

#include <string>
#include <QDialog>

#include "matvec3D.h"
#include "uncertaintyPlannerUtil.h"
#include "marchingcube/CIsoSurface.h"

class UncertaintyPlannerDlg : public QDialog, public Ui::UncertaintyPlannerDlgUI
{
	Q_OBJECT

private:
	//original model points
	std::vector<vec3> mModel;
	//the container for all the points overlayed over each other
	std::vector<augmentedVec3> mPoints;
	//the uniform sampling space
	std::vector<augmentedVec3> mSpace;
	double xmax, ymax, zmax, xmin, ymin, zmin;
	double xdim, ydim, zdim;
	int xstep, ystep, zstep;
	double mInterval;
	std::vector<transf> mUncertainties;
	UncertaintySpaceVisualizer mVisualizer;
	CIsoSurface<double> mIsoSurface;

	void loadObject(std::string fileName);
	void generateUncertaintyCombination(std::vector<transf> uncertainPoseList);
	void sampleUncertainty();
	void loadUncertainties(std::string fileName);
	void render(double percentageThreshold = 0.0);
	void generateMesh(double percentageThreshold = 0.25);

//	void naiveSample();
	void smartSample();
public:
	UncertaintyPlannerDlg(QWidget *parent = 0) : QDialog(parent), mInterval(10.0)
	{
		setupUi(this);
		QObject::connect(sampleButton, SIGNAL(clicked()), this, SLOT(sampleButton_clicked()));
		QObject::connect(loadButton, SIGNAL(clicked()), this, SLOT(loadButton_clicked()));
		QObject::connect(visualizeButton, SIGNAL(clicked()), this, SLOT(visualizeButton_clicked()));
		QObject::connect(loadUncertaintyButton, SIGNAL(clicked()), this, SLOT(loadUncertaintyButton_clicked()));
		QObject::connect(marchingCubeButton, SIGNAL(clicked()), this, SLOT(marchingCubeButton_clicked()));
		QObject::connect(percentageSlider, SIGNAL(sliderMoved(int)), this, SLOT(percentageSliderUpdated()));
		QObject::connect(intervalSlider, SIGNAL(sliderMoved(int)), this, SLOT(intervalSliderUpdated()));
	}

public slots:
	void sampleButton_clicked();
	void loadButton_clicked();
	void visualizeButton_clicked();
	void loadUncertaintyButton_clicked();
	void marchingCubeButton_clicked();
	void percentageSliderUpdated();
	void intervalSliderUpdated();
};
#endif /* UNCERTAINTYPLANNERDLG_H_ */
