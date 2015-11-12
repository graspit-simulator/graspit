/*
 * UncertaintyPlannerDlg.cpp
 *
 *  Created on: Jun 24, 2011
 *      Author: dang
 */

#include "uncertaintyPlannerDlg.h"

#include <iostream>
#include <fstream>
#include <QFileDialog>
#include <QSlider>

#include "matvec3D.h"
#include "world.h"
#include "body.h"
#include "graspitGUI.h"
#include "ivmgr.h"

void UncertaintyPlannerDlg::sampleButton_clicked()
{
	generateUncertaintyCombination(mUncertainties);
	smartSample();
}

void UncertaintyPlannerDlg::visualizeButton_clicked()
{
	double p = percentageSlider->value();
	std::cout << "percentage: " << p << "%\n";
	render((double)p/100.0);
}

void UncertaintyPlannerDlg::loadButton_clicked()
{
	QString fileName = QFileDialog::getOpenFileName(this,
			tr("Open Point Cloud"), "/export/home/melrose1/dang/project/voxelize/objects/asc" , tr("point cloud(*.asc)"));

	loadObject(fileName.toStdString());
}

void UncertaintyPlannerDlg::loadUncertaintyButton_clicked()
{
	QString fileName = QFileDialog::getOpenFileName(this,
			tr("Open uncertainties"), "/export/home/melrose1/dang/project/voxelize/objects/asc" , tr("uncertainties(*.txt)"));

	loadUncertainties(fileName.toStdString());
}

void UncertaintyPlannerDlg::marchingCubeButton_clicked()
{
	std::cout << "to generate mesh" << std::endl;
	generateMesh();
}

void UncertaintyPlannerDlg::loadUncertainties(std::string fileName)
{
	std::ifstream file(fileName.c_str());
	if(!file.is_open())
	{
		std::cout << "File not found: " << fileName << std::endl;
		return;
	}
	char line[129];
	//file opened
	double x, y, z, qw, qx, qy, qz;

	mUncertainties.clear();
	while(!file.getline(line, 128).eof())
	{
		//getline(file, line);
		sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &qx, &qy, &qz, &qw, &x, &y, &z);
		mUncertainties.push_back(transf(Quaternion(qw, qx, qy, qz), 1000*vec3(x,y,z)));
	}
	std::cout << mUncertainties.size() << "uncertainties received" << std::endl;
}

void UncertaintyPlannerDlg::loadObject(std::string fileName)
{
	std::ifstream file(fileName.c_str());
	if(!file.is_open())
	{
		std::cout << "File not found: " << fileName << std::endl;
		return;
	}
	char line[129];
	//file opened

	//skip the first five lines for the header
	for(int i = 0; i < 5; ++i)
		file.getline(line, 128);
	vec3 point;
	double x, y, z;
	mModel.clear();
	while(!file.getline(line, 128).eof())
	{
		//getline(file, line);
		sscanf(line, "%lf, %lf, %lf", &x, &y, &z);
		mModel.push_back(1000*vec3(x,y,z)); //unit: meters
	}
	std::cout << "object loaded, with" << mModel.size() << " points" << std::endl;
}

void UncertaintyPlannerDlg::generateUncertaintyCombination(std::vector<transf> uncertainPoseList)
{
	vec3 point;
	augmentedVec3 av;
	int maxNum = 2;
	mPoints.clear();
	for(size_t i = 0; i < mModel.size(); ++i)
	{
		for(size_t j = 0; j < uncertainPoseList.size() || j < maxNum; ++j)
		{
			point = mModel[i] > uncertainPoseList[j];
			av.vector = point;
			av.value = 0;
			mPoints.push_back(av);
		}
	}
}

void UncertaintyPlannerDlg::smartSample()
{
	//double xmax, ymax, zmax, xmin, ymin, zmin;
	xmin = xmax = mPoints[0].vector.x();
	ymin = ymax = mPoints[0].vector.y();
	zmin = zmax = mPoints[0].vector.z();

	for(size_t i = 0; i < mPoints.size(); ++i)
	{
		if(mPoints[i].vector.x() < xmin)
			xmin = mPoints[i].vector.x();
		if(mPoints[i].vector.y() < ymin)
			ymin = mPoints[i].vector.y();
		if(mPoints[i].vector.z() < zmin)
			zmin = mPoints[i].vector.z();

		if(mPoints[i].vector.x() > xmax)
			xmax = mPoints[i].vector.x();
		if(mPoints[i].vector.y() > ymax)
			ymax = mPoints[i].vector.y();
		if(mPoints[i].vector.z() > zmax)
			zmax = mPoints[i].vector.z();
	}


	double interval = 2.0; //unit: mm
	xdim = xmax - xmin;
	ydim = ymax - ymin;
	zdim = zmax - zmin;

	//int xstep, ystep, zstep;
	xstep = xdim/mInterval + 1;
	ystep = ydim/mInterval + 1;
	zstep = zdim/mInterval + 1;

	mSpace.clear();
	augmentedVec3 av;
	for(size_t i = 0; i < xstep; ++i)
	{
		for(size_t j = 0; j < ystep; ++j)
		{
			for(size_t k = 0; k < zstep; ++k)
			{
				av.vector = vec3(xmin + i * mInterval, ymin + j * mInterval, zmin + k * mInterval);
				av.value = 0;
				mSpace.push_back(av);
			}
		}
	}

	std::cout << "Generating " << mSpace.size() << " grids, sampling at interval: " << mInterval << " between: "
			<< std::endl << "x: " << xmin << " - " << xmax
			<< std::endl << "y: " << ymin << " - " << ymax
			<< std::endl << "z: " << zmin << " - " << zmax
			<< std::endl;

	for(size_t i = 0; i < mPoints.size(); ++i)
	{
		int l, m, n;
		l = (mPoints[i].vector.x() - xmin) / mInterval;
		m = (mPoints[i].vector.y() - ymin) / mInterval;
		n = (mPoints[i].vector.z() - zmin) / mInterval;
		mSpace[l * ystep * zstep + m * zstep + n].value += 1;
	}
}

void UncertaintyPlannerDlg::generateMesh(double percentageThreshold)
{
	double maximum = 0;
	for(size_t k = 0; k < mSpace.size(); ++k)
	{
		if(mSpace[k].value > maximum)
			maximum = mSpace[k].value;
	}

	double* scalarField;
	scalarField = (double*)malloc(sizeof(double) * mSpace.size() * 3);

	for(size_t k = 0; k < zstep; ++k)
	{
		for(size_t j = 0; j < ystep; ++j)
		{
			for(size_t i = 0; i < xstep; ++i)
			{
				if (mSpace[i * ystep * zstep + j * zstep + k].value < percentageThreshold * maximum)
					*(scalarField + k * ystep * xstep + j * xstep + i) = 0;
				else
					*(scalarField + k * ystep * xstep + j * xstep + i) = mSpace[i * ystep * zstep + j * zstep + k].value / maximum;
			}
		}
	}

	mIsoSurface.GenerateSurface(scalarField, percentageThreshold, xstep, ystep, zstep, mInterval, mInterval, mInterval);

	std::cout << mIsoSurface.m_nTriangles << std::endl;


	SoSeparator* root = graspItGUI->getIVmgr()->getWorld()->getIVRoot();

	if(mVisualizer.obj)
		root->removeChild(mVisualizer.obj);
	mVisualizer.obj = new SoSeparator;
	mVisualizer.fs = new SoIndexedFaceSet;
	mVisualizer.coordinate = new SoCoordinate3;
	mVisualizer.material = new SoMaterial;
	mVisualizer.binding = new SoMaterialBinding;
	mVisualizer.style = new SoDrawStyle;

	//mVisualizer.style->pointSize = 5.0;
	//mVisualizer.binding->value = SoMaterialBinding::PER_VERTEX_INDEXED;

	for(size_t i = 0; i < mIsoSurface.m_nVertices; ++i)
	{
		mVisualizer.coordinate->point.set1Value(i, mIsoSurface.m_ppt3dVertices[i][0]+xmin, mIsoSurface.m_ppt3dVertices[i][1]+ymin, mIsoSurface.m_ppt3dVertices[i][2]+zmin);
	}

	for(size_t i = 0; i < mIsoSurface.m_nTriangles; ++i)
	{
		mVisualizer.fs->coordIndex.set1Value(4*i, mIsoSurface.m_piTriangleIndices[i*3]);
		mVisualizer.fs->coordIndex.set1Value(4*i+1, mIsoSurface.m_piTriangleIndices[i*3+1]);
		mVisualizer.fs->coordIndex.set1Value(4*i+2, mIsoSurface.m_piTriangleIndices[i*3+2]);
		mVisualizer.fs->coordIndex.set1Value(4*i+3, -1);
		//mVisualizer.fs->materialIndex.set1Value(i, i);
		//mVisualizer.material->diffuseColor.set1Value(i, SbColor(mSpace[i].value / maximum, 0,0));
	}

	mVisualizer.obj->addChild(mVisualizer.coordinate);
	mVisualizer.obj->addChild(mVisualizer.material);
	mVisualizer.obj->addChild(mVisualizer.fs);
	root->addChild(mVisualizer.obj);
}


void UncertaintyPlannerDlg::render(double percentageThreshold)
{
	SoSeparator* root = graspItGUI->getIVmgr()->getWorld()->getIVRoot();

	if(mVisualizer.obj)
		root->removeChild(mVisualizer.obj);
	mVisualizer.obj = new SoSeparator;
	mVisualizer.ps = new SoIndexedPointSet;
	mVisualizer.coordinate = new SoCoordinate3;
	mVisualizer.material = new SoMaterial;
	mVisualizer.binding = new SoMaterialBinding;
	mVisualizer.style = new SoDrawStyle;

	mVisualizer.style->pointSize = 5.0;
	mVisualizer.binding->value = SoMaterialBinding::PER_VERTEX_INDEXED;

	double maximum = 0;
	for(size_t k = 0; k < mSpace.size(); ++k)
	{
		if(mSpace[k].value > maximum)
			maximum = mSpace[k].value;
	}

	for(size_t i = 0; i < mSpace.size(); ++i)
	{
		if(mSpace[i].value / maximum < percentageThreshold)
			continue;
		mVisualizer.coordinate->point.set1Value(i, mSpace[i].vector.x(), mSpace[i].vector.y(), mSpace[i].vector.z());
		mVisualizer.ps->coordIndex.set1Value(i, i);
		mVisualizer.ps->materialIndex.set1Value(i, i);
		mVisualizer.material->diffuseColor.set1Value(i, SbColor(mSpace[i].value / maximum, 0,0));
	}

	mVisualizer.obj->addChild(mVisualizer.coordinate);
	mVisualizer.obj->addChild(mVisualizer.material);
	mVisualizer.obj->addChild(mVisualizer.binding);
	mVisualizer.obj->addChild(mVisualizer.style);
	mVisualizer.obj->addChild(mVisualizer.ps);
	root->addChild(mVisualizer.obj);
}

void UncertaintyPlannerDlg::percentageSliderUpdated()
{
	double p = percentageSlider->value();
	percentageLabel->setText(QString::number(p) + QString("%"));
	render(p/100.0);
}

void UncertaintyPlannerDlg::intervalSliderUpdated()
{
	double i = intervalSlider->value();
	mInterval = i;
	intervalLabel->setText(QString::number(i) + QString("mm"));
}
