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
// $Id: compliantPlannerDlg.cpp,v 1.14 2009/07/02 21:04:05 cmatei Exp $
//
//######################################################################

#include "compliantPlannerDlg.h"

#include <iostream>

#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <QIntValidator>

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(QS_TOTAL);

//for the bounding box and drawing forces
#include "graspitGUI.h"
#include "ivmgr.h"

//for debug
#include "world.h"

#include "robot.h"
#include "body.h"
#include "grasp.h"
#include "listPlanner.h"
#include "searchState.h"

void
CompliantPlannerDlg::init()
{
	mPlanner = new ListPlanner(mHand);
	energyTypeBox->insertItem("Quasistatic");
	energyTypeBox->insertItem("Dynamic");
	mPlanner->setRenderType(RENDER_ALWAYS);
	mPlanner->setStatStream(&std::cerr);
	mOut = NULL;
	mHand->getGrasp()->setObjectNoUpdate(mObject);
	mObjectRefTran = mObject->getTran();
	QObject::connect(mPlanner, SIGNAL(update()), this, SLOT(update()));
	QObject::connect(mPlanner, SIGNAL(finished()), this, SLOT(plannerFinished()));
	mNumCandidates = 0;
	currentLabel->setText("0/0");
	QIntValidator* val = new QIntValidator(2, 99, this);
	resolutionEdit->setValidator(val);
	resolutionEdit->setText("8");
	QIntValidator* val2 = new QIntValidator(this);
	testOneEdit->setValidator(val2);
	testOneEdit->setText("0");
	resultsBox->insertItem("console");
	resultsBox->insertItem("file:");
	resultsFileEdit->setText("comp_plan.txt");

	QDoubleValidator* dVal = new QDoubleValidator(0.5, 1.5, 3, this);
	tFromEdit->setValidator(dVal);
	tToEdit->setValidator(dVal);
	tStepEdit->setValidator(dVal);
	sFromEdit->setValidator(dVal);
	sToEdit->setValidator(dVal);
	sStepEdit->setValidator(dVal);

	tFromEdit->setText("0.5");
	tToEdit->setText("1.5");
	tStepEdit->setText("0.1");
	sFromEdit->setText("0.5");
	sToEdit->setText("1.5");
	sStepEdit->setText("0.1");

}

CompliantPlannerDlg::~CompliantPlannerDlg()
{
	if (mOut) {
		mOut->close();
		delete mOut;
	}
	delete mPlanner;
}

void 
CompliantPlannerDlg::addCartesianSamples(const GraspPlanningState &seed, 
										 std::list<GraspPlanningState*> *sampling, 
										 int samples, double x, double y, double z)
{
	//redundant, but easier...
	double a = seed.readPosition()->getParameter("a");
	double b = seed.readPosition()->getParameter("b");
	//double c = seed.readPosition()->getParameter("c");
	//compute angular values
	//from HandObjectStateImpl:
	//x =  a * cos(beta) * cos(gamma);
	//y =  b * cos(beta) * sin(gamma);
	//z =  c * sin(beta);
	double beta = asin(z / sqrt(x*x + y*y + z*z));
	double gamma = atan2(y/b, x/a);
	DBGP("x: " << x << "; y: " << y <<"; z: " << z);
	DBGP("gamma: " << gamma << "; beta: " << beta);
	//sample roll angle as well
	for (int m=0; m<samples; m++) {
		//only sample from 0 to almost PI, as the HH is symmetric
		double tau = M_PI * ((double)m) / samples;
		GraspPlanningState *newState = new GraspPlanningState(&seed);
		newState->getPosition()->getVariable("tau")->setValue(tau);
		newState->getPosition()->getVariable("gamma")->setValue(gamma);
		newState->getPosition()->getVariable("beta")->setValue(beta);
		sampling->push_back(newState);
	}
}

/*! Samples an ellipsoid by sampling uniformly a grid with the same aspect
	ratio and projecting the resulting points on the ellipsoid. Not ideal,
	but at least much better then sampling angular variables directly */
void 
CompliantPlannerDlg::gridEllipsoidSampling(const GraspPlanningState &seed,
										   std::list<GraspPlanningState*> *sampling, 
										   int samples)
{
	double a = seed.readPosition()->getParameter("a");
	double aRes = 2.0 * a / samples;
	double b = seed.readPosition()->getParameter("b");
	double bRes = 2.0 * b / samples;
	double c = seed.readPosition()->getParameter("c");
	double cRes = 2.0 * c / samples;
	DBGP("a: " << a << "; b: " << b <<"; c: " << c);

	for (double i=0.5; i<samples; i+=1.0) {
		for(double j=0.5; j<samples; j+=1.0) {
			addCartesianSamples(seed, sampling, samples,  a, -b+i*bRes, -c+j*cRes);
			addCartesianSamples(seed, sampling, samples, -a, -b+i*bRes, -c+j*cRes);
			addCartesianSamples(seed, sampling, samples, -a+i*aRes, b , -c+j*cRes);
			addCartesianSamples(seed, sampling, samples, -a+i*aRes,-b , -c+j*cRes);
			addCartesianSamples(seed, sampling, samples, -a+i*aRes, -b+j*bRes , c);
			addCartesianSamples(seed, sampling, samples, -a+i*aRes, -b+j*bRes ,-c);
		}
	}
}

void
CompliantPlannerDlg::generateButtonClicked()
{
	int resolution = resolutionEdit->text().toInt();
	if (resolution < 1) {
		DBGA("Resolution must be at least 1");
		return;
	}
	//get object bbox dimensions
	SoGetBoundingBoxAction *bba = 
		new SoGetBoundingBoxAction(graspItGUI->getIVmgr()->getViewer()->getViewportRegion());
	bba->apply(mObject->getIVGeomRoot());
	SbVec3f bbmin,bbmax;
	bba->getBoundingBox().getBounds(bbmin,bbmax);
	delete bba;
	double a = 0.5*(bbmax[0] - bbmin[0]);
	double b = 0.5*(bbmax[1] - bbmin[1]);
	double c = 0.5*(bbmax[2] - bbmin[2]);
	//ellipsoidSampling(a,b,c,resolution);
	boxSampling(a,b,c, resolution);

	mPlanner->resetPlanner();
	if (visualMarkersBox->isChecked()) {
		visualMarkersBoxClicked();
	}
	update();
}

void 
CompliantPlannerDlg::ellipsoidSampling(double a, double b, double c, double resolution)
{
	//generate a list of grasps by sampling an ellipsoid around the object
	GraspPlanningState seed(mHand);
	seed.setObject(mObject);
	//todo: should use bbox center as reference frame, not object origin
	//which could be anything
	seed.setRefTran(mObject->getTran(), false);
	seed.setPostureType(POSE_DOF, false);
	seed.setPositionType(SPACE_ELLIPSOID, false);
	seed.reset();

	//set ellipsoid parameters
	seed.getPosition()->setParameter("a", a);
	seed.getPosition()->setParameter("b", b);
	seed.getPosition()->setParameter("c", c);
	//we don't want to sample distance
	seed.getPosition()->getVariable("dist")->setValue(0.0);
	seed.getPosition()->getVariable("dist")->setFixed(true);
	//create a list of earch states that sample the variables of the seed
	std::list<GraspPlanningState*> sampling;
	//uniform sampling of variables. Creates a horrible sampling.
	//createPositionSpaceSampling(seed, &sampling, resolution);
	//grid based sampling. Does somewhat better.
	gridEllipsoidSampling(seed, &sampling, resolution);
	DBGA("Sampled " << sampling.size() << " states.");
	mNumCandidates = sampling.size();
	//pass the list to the planner which will take ownership of it and destroy it
	mPlanner->setInput(sampling);
}

void
CompliantPlannerDlg::sampleFace(vec3 x, vec3 y, vec3 z, 
								double sz1, double sz2, vec3 tln, double res,
								std::list<GraspPlanningState*> *sampling)
{
	mat3 R(x, y, z);
	int rotSamples=2;

	double m1 = (2.0*sz1 - floor(2.0*sz1 / res) * res)/2.0;
	while (m1 < 2*sz1){
		double m2 = (2.0*sz2 - floor(2.0*sz2 / res) * res)/2.0;
		while (m2 < 2*sz2) {
			vec3 myTln(tln);
			myTln = myTln + (m1 - sz1)* y;
			myTln = myTln + (m2 - sz2)* z;
			transf tr(R, myTln);
			for(int rot=0; rot < rotSamples; rot++) {
				double angle = M_PI * ((double)rot) / rotSamples;
				transf rotTran(Quaternion(angle, vec3(1,0,0)), vec3(0,0,0));
				tr = rotTran * tr;
				GraspPlanningState* seed = new GraspPlanningState(mHand);
				seed->setObject(mObject);
				seed->setRefTran(mObject->getTran(), false);
				seed->setPostureType(POSE_DOF, false);
				seed->setPositionType(SPACE_COMPLETE, false);
				seed->reset();
				seed->getPosition()->setTran(tr);
				sampling->push_back(seed);
			}
			m2+=res;
		}
		m1 += res;
	}
}

void 
CompliantPlannerDlg::boxSampling(double a, double b, double c, double res)
{
	std::list<GraspPlanningState*> sampling;
	res = 30;
	sampleFace( vec3(0, 1,0), vec3(-1,0,0), vec3(0,0,1) , a, c, vec3(0,-b,0), res, &sampling);
	sampleFace( vec3(0,-1,0), vec3( 1,0,0), vec3(0,0,1) , a, c, vec3(0, b,0), res, &sampling);

	sampleFace( vec3(0,0, 1), vec3(0,1,0), vec3(-1,0,0) , b, a, vec3(0,0,-c), res, &sampling);
	sampleFace( vec3(0,0,-1), vec3(0,1,0), vec3( 1,0,0) , b, a, vec3(0,0, c), res, &sampling);

	sampleFace( vec3( 1,0,0), vec3(0, 1,0), vec3(0,0,1) , b, c, vec3(-a,0,0), res, &sampling);
	sampleFace( vec3(-1,0,0), vec3(0,-1,0), vec3(0,0,1) , b, c, vec3( a,0,0), res, &sampling);

	DBGA("Sampled " << sampling.size() << " states.");
	mNumCandidates = sampling.size();
	mPlanner->setInput(sampling);
}

void
CompliantPlannerDlg::testOneButtonClicked()
{
	if (energyTypeBox->currentText() == "Quasistatic") {
		mPlanner->setEnergyType(ENERGY_COMPLIANT);
	} else if (energyTypeBox->currentText() == "Dynamic") {
		mPlanner->setEnergyType(ENERGY_DYNAMIC);
	} else {
		assert(0);
	}
	if (mPlanner->isActive()) {
		DBGA("Stop planner first!");
		return;
	}
	int num = testOneEdit->text().toInt();
	if (num < 0 || num >= mNumCandidates) {
		DBGA("Wrong test number selected");
		return;
	}
	//single tests are always printed out to console
	mPlanner->setStatStream(&std::cerr);
	DBGA("Testing pre-grasp #" << num);
	mPlanner->testState(num);
	mHand->getWorld()->updateGrasps();
	graspItGUI->getIVmgr()->drawDynamicForces();
	graspItGUI->getIVmgr()->drawUnbalancedForces();
}

void
CompliantPlannerDlg::startPlanner()
{
	mPlanner->startPlanner();
}

void
CompliantPlannerDlg::testButtonClicked()
{
	if (energyTypeBox->currentText() == "Quasistatic") {
		mPlanner->setEnergyType(ENERGY_COMPLIANT);
	} else if (energyTypeBox->currentText() == "Dynamic") {
		mPlanner->setEnergyType(ENERGY_DYNAMIC);
	} else {
		assert(0);
	}

	if (mPlanner->isActive()) {
		DBGA("Pause:");
		mPlanner->pausePlanner();
	} else {
		PROF_RESET(QS_TOTAL);
		PROF_START_TIMER(QS_TOTAL);
		mBatch = false;
		if (mOut) {
			mPlanner->setStatStream(mOut);
		} else {
			mPlanner->setStatStream(&std::cerr);
		}
		startPlanner();
	}
}

void
CompliantPlannerDlg::update()
{
	QString n1,n2;
	n1.setNum(mPlanner->getCurrentStep());
	n2.setNum(mNumCandidates);
	currentLabel->setText(n1 + "/" + n2);
}

void
CompliantPlannerDlg::showResult()
{
	int d = mPlanner->getListSize();
	int rank, size, iteration; double energy;
	bool render = true;

	if (d==0) {
		mBestGraspNum = rank = size = iteration = energy = 0; render = false;
	} else if (mBestGraspNum < 0){
		mBestGraspNum = 0;
	} else if ( mBestGraspNum >= d) {
		mBestGraspNum = d-1;
	} 
 
	if ( d!=0 ){
		const GraspPlanningState *s = mPlanner->getGrasp(mBestGraspNum);
		rank = mBestGraspNum+1;
		size = d;
		energy = s->getEnergy();
		iteration = s->getItNumber();
	}

	QString n1,n2;
	n1.setNum(rank);
	n2.setNum(size);
	rankLabel->setText("Rank: " + n1 + "/" + n2);
	n1.setNum(energy,'f',3);
	energyLabel->setText("Energy: " + n1);
	n1.setNum(iteration);
	testOneEdit->setText(n1);
	showOneButtonClicked();
	iterationLabel->setText("Iteration: " + n1);

}

void
CompliantPlannerDlg::prevButtonClicked()
{
	mBestGraspNum--;
	showResult();
}
void
CompliantPlannerDlg::nextButtonClicked()
{
	mBestGraspNum++;
	showResult();
}
void
CompliantPlannerDlg::bestButtonClicked()
{
	mBestGraspNum=0;
	showResult();
}

void 
CompliantPlannerDlg::showOneButtonClicked()
{
	if (mPlanner->isActive()) {
		DBGA("Stop planner first!");
		return;
	}
	int num = testOneEdit->text().toInt();
	if (num < 0 || num >= mNumCandidates) {
		DBGA("Wrong test number selected");
		return;
	}
	DBGA("Testing pre-grasp #" << num);
	mPlanner->showState(num);
}

void 
CompliantPlannerDlg::prepareOneButtonClicked()
{
	if (mPlanner->isActive()) {
		DBGA("Stop planner first!");
		return;
	}
	int num = testOneEdit->text().toInt();
	if (num < 0 || num >= mNumCandidates) {
		DBGA("Wrong test number selected");
		return;
	}
	DBGA("Testing pre-grasp #" << num);
	mPlanner->prepareState(num);
}

void 
CompliantPlannerDlg::visualMarkersBoxClicked()
{
	mPlanner->showVisualMarkers( visualMarkersBox->isChecked());
}

void 
CompliantPlannerDlg::resetObjectButtonClicked()
{
	mObject->setTran(mObjectRefTran);
}

void
CompliantPlannerDlg::updateOut()
{
	if (mOut) {
		mOut->close();
		delete mOut; mOut = NULL;
	}
	if (resultsBox->currentIndex()==0) {
		DBGA("Output to stderr");
		return;
	}
	QString filename("data\\" + resultsFileEdit->text());
	mOut = new std::fstream(filename.latin1(), std::fstream::out | std::fstream::app);
	if (mOut->fail()) {
		DBGA("Failed to open file " << filename.latin1());
		delete mOut; mOut = NULL;
		resultsBox->setCurrentItem(0);
	} else {
		DBGA("Output file opened: " << filename.latin1());
	}
}

void 
CompliantPlannerDlg::designTestButtonClicked()
{
	if (!mOut) {
		DBGA("Set output file first!!!!");
		return;
	}
	mTFrom = tFromEdit->text().toDouble();
	mTTo = tToEdit->text().toDouble();
	mTStep = tStepEdit->text().toDouble();
	mSFrom = sFromEdit->text().toDouble();
	mSTo = sToEdit->text().toDouble();
	mSStep = sStepEdit->text().toDouble();
	DBGA("Starting batch testing");
	mBatch = true;
	mTR = mTFrom;
	mSR = mSFrom;
	mPlanner->setEnergyType(ENERGY_COMPLIANT);
	mPlanner->setStatStream(NULL);
	startPlanner();
}

void
CompliantPlannerDlg::plannerFinished()
{
	PROF_STOP_TIMER(QS_TOTAL);
	PROF_PRINT(QS_TOTAL);
	if (!mBatch) return;
	//write results to file
	DBGA("Test done: " << mTR << " and " << mSR);
	if (mOut) {
		*mOut << mTR << " " << mSR;
		*mOut << std::endl;
	}
	mSR += mSStep;
	if (mSR > mSTo + 1.0e-2) {
		mSR = mSFrom;
		mTR += mTStep;
		if (mTR > mTTo + 1.0e-2) {
			mBatch = false;
			return;
		}
	}
	mPlanner->resetPlanner();
	startPlanner();
}
