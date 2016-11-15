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
// $Id: eigenGraspDlg.cpp,v 1.5 2009/03/30 20:42:12 cmatei Exp $
//
//######################################################################

#include "eigenGraspDlg.h"

#include <QLayout>
#include <QLabel>
#include <QScrollBar>
#include <QCheckBox>
#include <QFileDialog>

#include "world.h"
#include "eigenGrasp.h"
#include "robot.h"

void EigenGraspDlg::init()
{
 SLIDER_STEPS = 100000;
 //the slider conversion translates slider steps into eigengrasp amplitude steps
 mSliderConversion = 0;
 mSlave = NULL;
}

void EigenGraspDlg::destroy()
{
	//just to be sure
	mEigenGrasps->setRigid(false);
}

int EigenGraspDlg::setWorld( World *w )
{
	mWorld = w;
	mHand = mWorld->getCurrentHand();
	assert(mHand);
	mEigenGrasps = mHand->getEigenGrasps();
	if (!mEigenGrasps || mEigenGrasps->getSize() == 0) {
		QTWARNING("Current hand contains no eigen grasp information");
		return 0;
	}
	resetSlave();
	adjustSliders();
	handConfigurationChanged();
        QObject::connect(mHand, SIGNAL(configurationChanged()), this, SLOT(handConfigurationChanged()) );
	return 1;
}

/*! The callback for slider values changed. Reads the values of the sliders,
	converts them to eigengrasp amplitudes. The values are checked against
	min and max values along each eg read from the eigengrasp interface, and
	clamped to be in the legal range. This has never worked completely well.
	
	The dialod then uses the eigengrasp interface to project eg amplitudes into 
	dof values. The interface will behave differently based on whether it is 
	set to "rigid" or not. After that, the dof's are used to set robot pose, but
	no collisions are checked. Finally, the eigengrasp interface is asked to
	recompute its min and max values along each eigengrasp.
*/
void EigenGraspDlg::eigenGraspChanged()
{
	double* amplitudes = new double[mNumberGrasps];
	QString val;
	for (int i=0; i < mNumberGrasps; i++) {
		double t = mBarList[i]->value() * mSliderConversion;
		//std::cout<<"Barlist["<<i<<"]="<<mBarList[i]->value()<<" slider conversion= "<<mSliderConversion<<std::endl;
		//std::cout<<"Product: "<<mBarList[i]->value() * mSliderConversion<<std::endl;

		//fprintf(stderr,"Slider %d value is %f\n",i,t);
		if (t > mEigenGrasps->getGrasp(i)->mMax) {
			mBarList[i]->setValue( (int)(mEigenGrasps->getGrasp(i)->mMax / mSliderConversion) - 1);
		} else if (t < mEigenGrasps->getGrasp(i)->mMin) {
			mBarList[i]->setValue( (int)(mEigenGrasps->getGrasp(i)->mMin / mSliderConversion) + 1);
		}		
		amplitudes[i] = mBarList[i]->value() * mSliderConversion;
		val.setNum(amplitudes[i],'f',2);
		mValueList[i]->setText(val);
	}

	//fprintf(stderr,"Start w. amplitudes: %f %f\n",amplitudes[0], amplitudes[1]);
	double *dof = new double[mHand->getNumDOF()];
	mEigenGrasps->getDOF(amplitudes,dof);
	/*
	fprintf(stderr,"Computed DOF: ");
	for(int d=0; d<mHand->getNumDOF(); d++) {
		fprintf(stderr,"%.1f ", dof[d]);
	}
	fprintf(stderr,"\n");
	*/
	mEigenGrasps->getAmp(amplitudes, dof);
	//fprintf(stderr,"Recovered amplitudes: %f %f\n",amplitudes[0], amplitudes[1]);

#ifdef EIGENGRASP_LOOSE
	if ( !mHand->checkSetDOFVals(dof) ) {
		fprintf(stderr,"All DOF values are illegal\n");
	}
#endif

	mHand->forceDOFVals(dof);
        QObject::disconnect(mHand, SIGNAL(configurationChanged()), this, SLOT(handConfigurationChanged()) );
        mHand->emitConfigChange();
        QObject::connect(mHand, SIGNAL(configurationChanged()), this, SLOT(handConfigurationChanged()) );
	mEigenGrasps->setMinMax();

	delete [] amplitudes;
	delete [] dof;
}

/*! Called when the "fix" check box for a particular eigengrasp has been
	changed. Fixes or unfixes that particular eigengrasp. Right now, 
	individual eigen grasp fix makes sense iff the interface is RIGID, 
	so if a box has been checked, this will also set the entire 
	eigengrasp interface to "rigid" mode.
*/
void EigenGraspDlg::fixBoxChanged()
{	
	bool fixed = false;
	for (int i=0; i < mNumberGrasps; i++ ) {
		if ( mCheckList[i]->isChecked() && mBarList[i]->isEnabled() ){
			double fa = mBarList[i]->value() * mSliderConversion;
			mEigenGrasps->fixEigenGrasp(i, fa);
			mBarList[i]->setEnabled(false);
			fixed = true;
		} else if ( !mCheckList[i]->isChecked() && !mBarList[i]->isEnabled() ){
			mEigenGrasps->unfixEigenGrasp(i);
			mBarList[i]->setEnabled(true);
		}
	}
	if (fixed) {
		rigidCheckBox->setChecked(true);
		rigidCheckBox_clicked();
	}
}

void EigenGraspDlg::resetSlave()
{
	if (mSlave) {
		delete mSlave;
	}

	mNumberGrasps = mEigenGrasps->getSize(); 
	fileNameLabel->setText( QString("Filename: ") + mEigenGrasps->getName() );

	mValueList.clear();
	mBarList.clear();
	mCheckList.clear();

	mSlave = new QDialog(this);
	setSlaveLayout(mNumberGrasps);
	mSlave->show();

	mEigenGrasps->setRigid(false);
	rigidCheckBox->setChecked(false);

//	handConfigurationChanged();
//	eigenGraspChanged();
}

void EigenGraspDlg::setSlaveLayout( int nGrasps )
{
	mainLayout = new QVBoxLayout(mSlave, 5);

	QLabel *valueLabel = new QLabel(QString("Value:"), mSlave);
	QLabel *amplLabel = new QLabel(QString("Amplitude:"), mSlave);
	QLabel *fixedLabel = new QLabel(QString("Fixed"), mSlave);

	QHBoxLayout *fakeRow = new QHBoxLayout(mainLayout,-1);
	fakeRow->addSpacing(400);

	QHBoxLayout *labelRow = new QHBoxLayout(mainLayout,-1);
	labelRow->addSpacing(5);
	labelRow->addWidget(valueLabel,0);
	labelRow->addWidget(amplLabel,1,Qt::AlignHCenter);
	labelRow->addWidget(fixedLabel,0);
	labelRow->addSpacing(5);
	mainLayout->addLayout(labelRow);

	for (int i=0; i<nGrasps; i++) {
		QHBoxLayout *graspRow = new QHBoxLayout(mainLayout,10);
	
		QLabel *eigenValue = new QLabel(QString("0.0"), mSlave);
		QScrollBar *bar = new QScrollBar(Qt::Horizontal, mSlave);
		bar->setRange( -SLIDER_STEPS, SLIDER_STEPS );
		bar->setPageStep(5000);
		bar->setLineStep(1000);
		bar->setValue(0);
		QCheckBox *box = new QCheckBox(mSlave);

		graspRow->addSpacing(15);
		graspRow->addWidget(eigenValue,0);
		graspRow->addWidget(bar,1);
		graspRow->addWidget(box,0);
		graspRow->addSpacing(15);

		mValueList.push_back(eigenValue);
		mBarList.push_back(bar);
		mCheckList.push_back(box);

		connect(bar,SIGNAL(sliderMoved(int)), this, SLOT(eigenGraspChanged()) );
		//comment this one out
		//connect(bar,SIGNAL(valueChanged(int)), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(nextLine()), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(prevLine()), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(nextPage()), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(prevPage()), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(sliderReleased()), this, SLOT(eigenGraspChanged()) );
		connect(box,SIGNAL(clicked()), this, SLOT(fixBoxChanged()) );
	}	
	mainLayout->addSpacing(20);
}

void EigenGraspDlg::saveButton_clicked()
{
	QString fn = QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT"))+QString("/models/eigen"),
			"EigenGrasp Files (*.xml)") ;
    if ( !fn.isEmpty() ) {
	    if (fn.section('.',1).isEmpty())
			fn.append(".xml");
		mEigenGrasps->writeToFile( fn.latin1() );
	}
}

void EigenGraspDlg::loadButton_clicked()
{
	QString fn = QFileDialog::getOpenFileName( this, QString(), QString(getenv("GRASPIT"))+QString("/models/eigen"),
			"EigenGrasp Files (*.xml)" );
	if (fn.isEmpty()) return;
	mHand->loadEigenData(fn);
	mEigenGrasps = mHand->getEigenGrasps();
	resetSlave();
}

void EigenGraspDlg::identityButton_clicked()
{
	mHand->useIdentityEigenData();
	mEigenGrasps = mHand->getEigenGrasps();
	resetSlave();
}

void EigenGraspDlg::exitButton_clicked()
{
	mEigenGrasps->setRigid(false);
	//yes, I know, it leaks memory (but just a little).
	QDialog::accept();
}

/*! Attempts to set the slider conversion factor based on the range
	of legal values of all eigengrasps in the eigengrasp interface.
	For various reasons this has never worked well, so right not it
	is hard-coded to some conversion value that works reasonably well.
*/
void EigenGraspDlg::adjustSliders()
{
	double max;
/*
	double min;
	min = 1.0e5;
	max = -1.0e5;

	for (int i=0; i<mNumberGrasps; i++) {
		if ( mEigenGrasps->getGrasp(i)->mMin < min ) min = mEigenGrasps->getGrasp(i)->mMin;
		if ( mEigenGrasps->getGrasp(i)->mMax > max ) max = mEigenGrasps->getGrasp(i)->mMax;
	}

	//map all the sliders to the maximum range of motion (positive or negative)
	//amongst all eigengrasps. This guarantees that all origin positions are in the middle of their respective
	//sliders, and all sliders are the same length. However, sliders can not be used all the way
	if ( fabs(min) > max ) max = fabs(min);
	fprintf(stderr,"\n MAX is %f \n", max);

	//this fails because some EG have tiny contributions on some joints and this created huge numbers here
*/
 max = 4.0;
 mSliderConversion = 2 * max / SLIDER_STEPS;
}

void EigenGraspDlg::show()
{
	QDialog::show();
}

void EigenGraspDlg::setAmplitudes(double *amp)
{
	QString val;
	for (int i=0; i < mNumberGrasps; i++) {
		mBarList[i]->setValue( (int) (amp[i] / mSliderConversion) );
		val.setNum(amp[i],'f',2);
		mValueList[i]->setText(val);
	} 
}

/*! When the posture of the hand is changed by the user by direct interaction
	with a dof, this will compute the eg projection of the resulting dof
	posture, and set the slider positions accordingly. That will also trigger
	the sliders changed callback, which will set hand posture again. The result
	of that will depend on whether the interface is rigid or not, see 
	EigenGraspInterface for details.
*/
void EigenGraspDlg::handConfigurationChanged()
{
	double *amp = new double[mNumberGrasps];
	double *dof = new double[mHand->getNumDOF()];
	mHand->getDOFVals(dof);
	mEigenGrasps->getAmp(amp, dof);
	setAmplitudes(amp);
	mEigenGrasps->setMinMax();
	delete [] amp;
	delete [] dof;
}


void EigenGraspDlg::setOriginButton_clicked()
{
	double *dof = new double[mHand->getNumDOF()];
	mHand->getDOFVals(dof);
	mEigenGrasps->setOrigin(dof);
	handConfigurationChanged();
	delete [] dof;
}


void EigenGraspDlg::rigidCheckBox_clicked()
{
	if (rigidCheckBox->isChecked()) {
		mEigenGrasps->setRigid(true);
		eigenGraspChanged();
	} else {
		mEigenGrasps->setRigid(false);
		//unfix any individual eigengrasp fix check boxes
		for (int i=0; i < mNumberGrasps; i++ ) {	
			if ( mCheckList[i]->isChecked() ) mCheckList[i]->setChecked(false);
		}
		fixBoxChanged();
	}
}

void EigenGraspDlg::goToOrigin()
{
	double* amplitudes = new double[mNumberGrasps];
	QString val;

	for (int i=0; i < mNumberGrasps; i++) {
		mBarList[i]->setValue(0);
		amplitudes[i] = 0;
		val.setNum(0.0,'f',2);
		mValueList[i]->setText(val);
	}
	double *dof = new double[mHand->getNumDOF()];
	mEigenGrasps->getDOF(amplitudes,dof);
	mHand->forceDOFVals(dof);
	mEigenGrasps->setMinMax();
	adjustSliders();

	delete [] amplitudes;
	delete [] dof;
}


void EigenGraspDlg::closeHandButton_clicked()
{
	// obsolete; has been removed
}


void EigenGraspDlg::goToOriginButton_clicked()
{
	goToOrigin();
}


