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
// $Id: eigenGrasp.cpp,v 1.15 2009/08/14 18:09:33 cmatei Exp $
//
//######################################################################

#include "eigenGrasp.h"
#include "robot.h"

#include <math.h>
#include <stdio.h>

#include <QFile>
#include <QString>
#include <QTextStream>

#include "math/matrix.h"

#include "debug.h"

/*! The eigengrasp is initialized to all 0s*/
EigenGrasp::EigenGrasp(int size, double e)
{
	if (size <= 0 ) {
		fprintf(stderr,"Wrong size of eigen grasp\n");
		return;
	}
	mVals = new double[size];
	for (int i=0; i < size; i++)
		mVals[i] = 0;
	mSize = size;
	mEigenValue = e;
	mFixed = false;
}

EigenGrasp::EigenGrasp(const EigenGrasp *orig)
{
	mSize = orig->mSize;
	mVals = new double[mSize];
	memcpy(mVals, orig->mVals, mSize*sizeof(double));
	mEigenValue = orig->mEigenValue;
	mMin = orig->mMin; mMax = orig->mMax;
	mFixed = orig->mFixed;
	fixedAmplitude = orig->fixedAmplitude;
}

EigenGrasp::~EigenGrasp()
{
	delete [] mVals;
}

void
EigenGrasp::getEigenGrasp(double *eg) const
{
	for (int i=0; i<mSize; i++)
		eg[i] = mVals[i];
}

void
EigenGrasp::setEigenGrasp(const double *eg)
{
	for (int i=0; i<mSize; i++)
		mVals[i] = eg[i];
}

double
EigenGrasp::normalize()
{
	int i;
	double norm = 0;
	for (i=0; i<mSize; i++) {
		norm += mVals[i] * mVals[i];
	}
	norm = sqrt(norm);
	for (i=0; i<mSize; i++) {
		mVals[i] = mVals[i] / norm;
	}
	return norm;
}

void
EigenGrasp::writeToFile(FILE *fp)
{
	fprintf(fp,"%f\n",mEigenValue);
	for (int i=0; i<mSize; i++) {
		fprintf(fp,"%f ", mVals[i]);
	}
	fprintf(fp,"\n");
}

void 
EigenGrasp::readFromFile(FILE *fp)
{
	float v;
	fscanf(fp,"%f",&v);
	mEigenValue = v;
	for (int i=0; i<mSize; i++) {
		fscanf(fp,"%f",&v);
		mVals[i] = v;
	}
}

int
EigenGrasp::readFromStream(QTextStream *stream)
{
	if (stream->atEnd()) {fprintf(stderr,"Unable to read EG, end of file\n");return 0;}
	QString line = stream->readLine();
	mEigenValue = line.toDouble();
	if (stream->atEnd()) {fprintf(stderr,"Unable to read EG, end of file\n");return 0;}
	line = stream->readLine();
	for (int i=0; i<mSize; i++) {
		QString val = line.section(' ',i,i,QString::SectionSkipEmpty);
		if ( val.isNull() || val.isEmpty() ) {fprintf(stderr,"Unable to read EG value #%d\n",i);return 0;}
		mVals[i] = val.toDouble();
	}
	return 1;
}

double
EigenGrasp::dot(double *d)
{
	double dot = 0;
	for (int i=0; i<mSize; i++) {
		dot += mVals[i] * d[i];
	}
	return dot;
}

//---------------------------------------------------- EigenGraspInterface --------------------------------------

EigenGraspInterface::EigenGraspInterface(Robot *r)
{
	mRobot = r;
	dSize = mRobot->getNumDOF();
	eSize = mGrasps.size();
	mOrigin = NULL;
	mNorm = NULL;
	mRigid = false;
	mP = mPInv = NULL;
}

EigenGraspInterface::EigenGraspInterface(const EigenGraspInterface *orig)
{
	mRobot = orig->mRobot;
	dSize = mRobot->getNumDOF();
	eSize = orig->eSize;

	for (int i=0; i<eSize; i++) {
		mGrasps.push_back( new EigenGrasp(orig->mGrasps[i]) );
	}

	mOrigin = new EigenGrasp( orig->mOrigin );
	mNorm = new EigenGrasp( orig->mNorm);
	mName = orig->mName;
	mRigid = orig->mRigid;
	mP = mPInv = NULL;
	if (orig->mP) {
		mP = new Matrix(*(orig->mP));
	}
	if (orig->mPInv) {
		mPInv = new Matrix(*(orig->mPInv));
	}
}

EigenGraspInterface::~EigenGraspInterface()
{
	clear();
}

void
EigenGraspInterface::clear()
{
	for (int i=0; i < eSize; i++) {
		delete mGrasps[i];
	}
	mGrasps.clear();
	eSize = 0;
	if (mOrigin){
		delete mOrigin;
		mOrigin = NULL;
	}
	if (mNorm) {
		delete mNorm;
		mNorm = NULL;
	}
	if (mP) {delete mP; mP = NULL;}
	if (mPInv) {delete mPInv; mPInv = NULL;}
	mRigid = false;
}

/*! Writes the entire interface to a file in the following order:
	- the size of the eg space
	- the size of the dof space
	- all eigengrasps
	- the origin
	Seems not to be compatible with the readFromFile fucntion
	of this class.
*/
int
EigenGraspInterface::writeToFile(const char *filename)
{
	FILE *fp = fopen(filename,"w");
	if (!fp) return 0;

	fprintf(fp,"%d\n",eSize);
	fprintf(fp,"%d\n",mRobot->getNumDOF());
	for (int i=0; i<eSize; i++) {
		mGrasps[i]->writeToFile(fp);
	}
	mOrigin->writeToFile(fp);
	fclose(fp);
	return 1;
}

int
EigenGraspInterface::setTrivial()
{
	if (dSize != mRobot->getNumDOF() ) {
		fprintf(stderr,"ERROR setting trivial EG's\n");
		return 0;
	}
	clear();
	eSize = mRobot->getNumDOF();
	double *eg = new double[eSize];
	for (int i=0; i<eSize; i++) {
		eg[i] = 0;
	}
	for (int i=0; i<eSize; i++) {
		EigenGrasp *newGrasp = new EigenGrasp(eSize);
		eg[i]=1;
		newGrasp->setEigenGrasp(eg);
		eg[i]=0;
		mGrasps.push_back(newGrasp);
	}
	mNorm = new EigenGrasp(dSize);
	mNorm->setOnes();
	mOrigin = new EigenGrasp(dSize);
	setSimpleOrigin();
	computeProjectionMatrices();
	setName("Identity");
	delete [] eg;
	return 1;
}

/*! Reads the entire interface from a file, looking for keywords 
	inside the file. Loads the size of the dof space, all eg's
	(which define the size of the eg space), any normalization
	factors (optional) and the origin of the eg space (optional).

	The size of the dof space must be loaded before any eg's, 
	origin, etc can be loaded; is must also match the number of
	dof's in this robot.
*/
int
EigenGraspInterface::readFromFile(QString filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		DBGA("Failed to open EG file: " << filename.latin1());
		return 0;
	}
	QTextStream stream( &file );

	clear();
	int numDims = 0;
	QString line,id;
	bool error = false;
	while (!error && !stream.atEnd() ) {
		line = stream.readLine();
		line.stripWhiteSpace();
		id = line.section(' ',0,0);
		if (id == "DIMENSIONS") {
			numDims = line.section(' ',1,1).toInt();
			if (numDims != mRobot->getNumDOF()) {
				DBGA("ERROR reading eigengrasp: appears to be constructed for another robot.");
			}
		} else if ( id == "EG" ) {
			if (!numDims) {DBGA("Number of dimensions not specified!"); error = true; continue;}
			EigenGrasp *newGrasp = new EigenGrasp(numDims);
			if (!newGrasp->readFromStream(&stream)) {error = true; continue;}
			newGrasp->normalize();
			mGrasps.push_back(newGrasp);
		} else if ( id == "ORIGIN" ) {
			if (!numDims) {DBGA("Number of dimensions not specified!"); error = true; continue;}
			mOrigin = new EigenGrasp(numDims);
			if (!mOrigin->readFromStream(&stream)){error = true; continue;}
			checkOrigin();
		} else if ( id == "NORM" ) {
			if (!numDims) {DBGA("Number of dimensions not specified!"); error = true; continue;}
			mNorm = new EigenGrasp(numDims);
			if (!mNorm->readFromStream(&stream)){error = true; continue;}
			DBGA("EG Normalization data loaded from file");
		}
	}
	if (error) {
		DBGA("Error reading EG file");
		return 0;
	}

	eSize = mGrasps.size();
	DBGA("Read " << eSize << " eigengrasps from EG file");
	if (!mNorm) {
		DBGA("No normalization data found; using factors of 1.0");
		mNorm = new EigenGrasp(dSize);
		mNorm->setOnes();
	}
	if (!mOrigin) {
		DBGA("No EG origin found; using automatic origin");
		mOrigin = new EigenGrasp(dSize);
		setSimpleOrigin();
	}
	computeProjectionMatrices();
	setMinMax();
	return 1;
}

/*! If the origin of the eigengrasp subspace is not inside the legal range
	of the dof, the coordinate of the origin along the offending axis is 
	set to either the min or the max of that dof.
*/
void
EigenGraspInterface::checkOrigin()
{
	for (int d=0; d < dSize; d++) {
		if (mOrigin->getAxisValue(d) < mRobot->getDOF(d)->getMin()) {
			fprintf(stderr,"WARNING: Eigengrasp origin lower than DOF %d range\n",d);
			mOrigin->setAxisValue( d, mRobot->getDOF(d)->getMin() );
//			mOrigin->setAxisValue( d, 0.5 * (mRobot->getDOF(d)->getMin() + mRobot->getDOF(d)->getMax()) );
		}
		if (mOrigin->getAxisValue(d) > mRobot->getDOF(d)->getMax()) {
			fprintf(stderr,"WARNING: Eigengrasp origin greater than DOF %d range\n",d);
			mOrigin->setAxisValue( d, mRobot->getDOF(d)->getMax() );
//			mOrigin->setAxisValue( d, 0.5 * (mRobot->getDOF(d)->getMin() + mRobot->getDOF(d)->getMax()) );
		}
	}
}
void
EigenGraspInterface::setOrigin(const double *dof)
{
	mOrigin->setEigenGrasp(dof);
	checkOrigin();
}

/*! Sets as origin of eigengrasp subspace the point which is halfway between 
	min and max for each DOF of the robot.
*/
void
EigenGraspInterface::setSimpleOrigin()
{
	double mmin, mmax;
	double* o = new double[dSize];
	for (int d=0; d < dSize; d++) {
		mmin = mRobot->getDOF(d)->getMin();
		mmax = mRobot->getDOF(d)->getMax();
		o[d] = (mmax + mmin) / 2.0;
	}
	setOrigin(o);
	delete [] o;
}

/*! Computes the projection matrices that go between dof and eg space.
	This works in the general case; no assumptions are made about the
	basis of the eg space (the eigengrasps themselves) being orthonormal.
	In the case of orthonormal eg's, simple dot products would do as well,
	but this is more general and elegant.

	In general, if we define the projection from dof space to eg space as
	P(x) = a and its inverse as PInv(a) = x, the following rules apply:
	P(PInv(a)) = a (always)
	PInv(P(x)) != x (usually)
*/

void
EigenGraspInterface::computeProjectionMatrices()
{
	if (mP) delete mP;
	if (mPInv) delete mPInv;

	//first build the E matrix that just contains the (potentially non-orthonormal) bases
	Matrix E(eSize, dSize);
	for (int e=0; e<eSize; e++) {
		for (int d=0; d<dSize; d++) {
			E.elem(e,d) = mGrasps[e]->getAxisValue(d);
		}
	}

	//the trivial case (assumes ortho-normal E)
	//mP = new Matrix(E);
	//mPInv = new Matrix(E.transposed());

	//general case
	//remember: P(PInv(a)) = a (always)
	//		    PInv(P(x)) != x (usually)
	// P = (E*ET)'*E
	// P' = ET
	Matrix ET(E.transposed());
	Matrix EET(eSize, eSize);
	matrixMultiply(E, ET, EET);
	Matrix EETInv(eSize, eSize);
	int result = matrixInverse(EET, EETInv);
	if (result) {
		DBGA("Projection matrix is rank deficient!");
		mP = new Matrix(Matrix::ZEROES<Matrix>(eSize, dSize));
		mPInv = new Matrix(Matrix::ZEROES<Matrix>(dSize, eSize));
		return;
	}
	mP = new Matrix(eSize, dSize);
	matrixMultiply(EETInv, E, *mP);
	mPInv = new Matrix(ET);
}

/*! The boundary of the "legal" space forms a polygon in EG-subspace 
	which is not axis-aligned. It is difficult to compute analytically
	so for any new hand configuration this function re-computes the min 
	and max values. The min and max along each eigengrasp are then stored
	inside the eigengrasps themselves.

	The behavior depends on whether EIGENGRASP_LOOSE is defined. If not, min
	and max have to be set so that not a single dof is taken outside of its
	range. If it is not set, then min and max have to be set so that at least
	one dof is inside the legal range, the others can go out as they will be
	clamped later.

	For some reason, this apparently simple function has given me a ton of
	trouble, and I've never been completely happy with it.
*/
void
EigenGraspInterface::setMinMax()
{
	double m,M,mmin,mmax;
	double* eg = new double[dSize];
	double* currentDOF = new double[dSize];
	double* currentAmps  = new double[eSize];

	mRobot->getDOFVals(currentDOF);
	getAmp(currentAmps, currentDOF);

	for (int e = 0; e < eSize; e++)
	{
		int mind, maxd;
		//fprintf(stderr,"\n------\nEG %d\n",e);
#ifdef EIGENGRASP_LOOSE
		mmin = +1.0e5;
		mmax = -1.0e5;
#else
		mmin = -1.0e5;
		mmax = +1.0e5;
#endif
		mGrasps[e]->getEigenGrasp(eg);
		for (int d=0; d < dSize; d++) {
			if (eg[d]==0) continue;
			m = ( mRobot->getDOF(d)->getMin() - currentDOF[d] ) / ( eg[d] * mNorm->getAxisValue(d) );
			M = ( mRobot->getDOF(d)->getMax() - currentDOF[d] ) / ( eg[d] * mNorm->getAxisValue(d) );
			if ( m > M) {std::swap(m,M);} //swap m and M if needed
#ifdef EIGENGRASP_LOOSE
			if ( m < mmin ) {mmin = m; mind = d;}
			if ( M > mmax ) {mmax = M; maxd = d;}
#else
			if ( m > mmin ) {mmin = m; mind = d;}
			if ( M < mmax ) {mmax = M; maxd = d;}
#endif
		}
		mGrasps[e]->mMin = currentAmps[e] + mmin;
		mGrasps[e]->mMax = currentAmps[e] + mmax;

		//fprintf(stderr,"Current: %f; range: %f(%d) -- %f(%d) \n",currentAmps[e],
		//		mGrasps[e]->mMin, mind, mGrasps[e]->mMax, maxd);
	}

	delete [] eg;
	delete [] currentDOF;
	delete [] currentAmps;
}

void
EigenGraspInterface::toDOFSpace(const double *amp, double *dof, const double *origin) const
{
	Matrix a(amp, eSize, 1, true);
	Matrix x(dSize, 1);
	matrixMultiply(*mPInv, a, x);
	for(int d=0; d<dSize; d++) {
		dof[d] = x.elem(d,0) * mNorm->getAxisValue(d) + origin[d];
	}
}

void
EigenGraspInterface::toEigenSpace(double *amp, const double *dof, const double *origin) const
{
	Matrix x(dSize, 1);
	for (int d=0; d < dSize; d++) {
		x.elem(d,0) = (dof[d] - origin[d]) / mNorm->getAxisValue(d);
	}
	Matrix a(eSize, 1);
	matrixMultiply(*mP, x, a);
	for (int e=0; e<eSize; e++) {
		amp[e] = a.elem(e,0);
	}
}

/*! Given a vector of EG amplitudes (a point in EG subspace) this 
	function returns it's equivalent in DOF space. Essentially, it
	back-projects a point from eg space to dof space.

	If the interface is rigid, we add the amplitudes to the pre-specified 
	eigen space origin. This means we DISCARD whatever component was in 
	the pose that was not from eigenspace.
	
	If the interface is not rigid, we add the change in amplitudes to the 
	current position of the robot. This means we KEEP the position component 
	that was not in eigen space.
*/
void 
EigenGraspInterface::getDOF(const double *amp, double *dof) const 
{
	double *origin = new double[dSize];
	double *rigidAmp = new double[eSize];
	
	for (int e=0; e < eSize; e++) {
		if ( !mGrasps[e]->mFixed )
			rigidAmp[e] = amp[e];
		else {
			rigidAmp[e] = mGrasps[e]->fixedAmplitude;
			DBGA(e << " fixed at " << mGrasps[e]->fixedAmplitude);
		}
	}

	if (mRigid) {
		//if the interface is rigid, we add the amplitudes to the pre-specified eigen space origin
		//it means we DISCARD whatever component was in the pose that was not from eigenspace
		mOrigin->getEigenGrasp(origin);
		toDOFSpace(rigidAmp, dof, origin);
	} else {
		//if it is not, we just add the change in amplitudes to the current position of the robot
		//this means we KEEP the position component that was not in eigen space
		double *currentAmp = new double[eSize];
		double *relativeAmp = new double[eSize];
		mRobot->getDOFVals(origin);
		getAmp(currentAmp, origin);
		for (int e=0; e < eSize; e++) {
			relativeAmp[e] = rigidAmp[e] - currentAmp[e];
		}
		toDOFSpace(relativeAmp, dof, origin);
		delete [] currentAmp;
		delete [] relativeAmp;		
	}

	delete [] rigidAmp;
	delete [] origin;
}

/*! Given a set of dof values (a point in dof space), it computes its 
	eigengrasp amplitudes (its projection in eg space). Just subtracts
	the origin of the eg space from the provided point, then projects
	the result along the eg subspace.
*/
void 
EigenGraspInterface::getAmp(double *amp, const double *dof) const
{
	double *origin = new double[dSize];
	mOrigin->getEigenGrasp(origin);
	toEigenSpace(amp, dof, origin);
	delete [] origin;
}
