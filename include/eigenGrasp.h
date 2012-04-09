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
// $Id: eigenGrasp.h,v 1.14 2009/04/21 14:53:07 cmatei Exp $
//
//######################################################################
#ifndef _eigengrasp_h_
#define _eigengrasp_h_

#include <assert.h>

#include <vector>
#include <QString>
#include <QTextStream>

/*!	If this is not defined, than motion along eigengrasps is permitted only 
	if NO joint values are taken outside the legal range. If it defined, we 
	allow motion as long as at least one joint is still legal, while 
	artificially constraining the ones that would leave the legal range of motion
	This kind of constraining is unwanted because it implies movement that is not 
	in eigen space and produces artifacts.
*/
#define EIGENGRASP_LOOSE

class TiXmlElement;
class Matrix;
class Robot;
class QTextStream;
//! A single eigengrasp direction
/*!	An eigengrasp holds a generalized direction of motion in \a mSize 
	dimensions. mSize is assumed to be the number of dofs of the robot 
	that this is used for. It is essentially an msize-dimensional vector.

	mMin and mMax store the min and max amplitudes of motion along this 
	eigengrasp that will not take the robot outside the acceptable DOF 
	range. They only make sense when defined relative to an ORIGIN of 
	motion!
*/
class EigenGrasp
{
private:
	//! Dimension of this eigengrasp
	int mSize;
	//! The actual vector
	double *mVals;
	//! The eigenvalue corresponding to this EG. Not actually used anywhere, just here if ever needed
	double mEigenValue;

public:
	//! Stores the min value along this eigengrasp that is inside the legal joint range.
	double mMin;
	//! Stores the max value along this eigengrasp that is inside the legal joint range.
	double mMax;
	//! True if limits loaded from EigenGrasp File. If this is false Graspit! will attempt to compute limits at runtime.
	bool mPredefinedLimits;
	//! Remembers if the amplitude along this EG is fixed (no motion along this EG is allowed)
	bool mFixed;
	//! If this eigengrasp is fixed, this is the value that is was fixed at
	double fixedAmplitude;

	//! Initializes an empty eigengrasp of a given size
	EigenGrasp(int size, double e=0);
	//! Copy constructor
	EigenGrasp(const EigenGrasp *orig);
	~EigenGrasp();

	int getSize() const {return mSize;}

	//! Returns the mSize-dimensional vector itself
	void getEigenGrasp (double *eg) const;
	//! Sets the mSize-dimensional vector itself
	void setEigenGrasp(const double *eg);
	
	//! Gets the eigenvalue associated w. this eigengrasp; not used.
	double getEigenValue() const {return mEigenValue;}
	//! Sets the eigenvalue associated w. this eigengrasp; not used.
	void setEigenValue(double e){mEigenValue = e;}

	//! Sets this eigengrasp to the vector [1,...,1]
	void setOnes(){for (int i=0; i<mSize; i++) mVals[i]=1.0;}

	//! Returns the comonent of this eigengrasp along the i-th dimension
	double getAxisValue(int i) const {assert(i<mSize&&i>=0); return mVals[i];}
	//! Sets the component of this eigengrasp along the i-th dimension
	void setAxisValue(int i, double val){assert(i<mSize&&i>=0); mVals[i]=val;}

	//! Normalizes this eigengrasp to norm 1
	double normalize();

	//! This is the main function that is used: dot the EG against another mSize-dimensional vector
	double dot(double *d);

	void writeToFile(FILE *fp);
	void writeToFile(TiXmlElement *ep);
	void readFromFile(FILE *fp);

	int readFromStream(QTextStream *stream);
	int readFromXml(const TiXmlElement *element);
	
	//! Tells this eigengrasp that it is fixed at the given value
	void fix(double a){mFixed = true; fixedAmplitude = a;}
	//! Tells this eigengrasp that it can move freely
	void unfix(){mFixed = false;}
};

/*!	This is the complete interface for controlling a robot in an eigengrasp
	subspace. It holds a set of eigengrasps for a particular robot. It also 
	has AN ORIGIN OF MOTION which is VERY IMPORTANT: it completely defines 
	the subspace of the eigengrasps.

	The main role of this interface is to go back and forth between two spaces:
	the dof, of dimensionality \a dSize, and the eigengrasp space, of 
	dimensionality \a eSize. It simply defines the eigengrasp subspace and 
	provides a linear mapping between the two.
*/
class EigenGraspInterface
{
private:
	//! The robot that this interface refers to. Should have the right number of joints.
	Robot *mRobot;
	//! The dimensionality of the dof space
	int dSize;
	//! The dimensionality of the eigengrasp space.
	int eSize;

	//! The eigengrasps themselves; the bases of the subspace
	std::vector<EigenGrasp*> mGrasps;
	//! The origin of the EG subspace
	EigenGrasp *mOrigin;

	//! Used if each EG has been "normalized". 
	/*! Values along each axis are first multiplied by the respective
		value in here, then added back to the origin (mean). */
	EigenGrasp *mNorm;

	//! Helper that allows us to display what file the EG's where loaded from .
	QString mName;

	//!Matrix that projects from dof space to eigen space
	Matrix *mP;
	//!Matrix that projects from eigen space to dof space
	Matrix *mPInv;

	//! Shows if motion of the robot is RIGID 
	/*! In rigid motion, only configuration that are strictly inside the
		EG subspace are allowed. Otherwise, any configuration is allowed. 
		This is relevant when getAmp(...) and getDOF(...) are used. */
	mutable bool mRigid;

	void clear();

	//! Builds the projection matrices based on eigengrasp definitions.
	void computeProjectionMatrices();

	//! Goes from EG space to DOF space
	void toDOFSpace(const double *amp, double *dof, const double *origin) const;
	//! Goes from DOF space to EG space
	void toEigenSpace(double *amp, const double *dof, const double *origin) const;
public:
	EigenGraspInterface(Robot *r);
	EigenGraspInterface(const EigenGraspInterface *orig);
	~EigenGraspInterface();

	//! Returns the g-th eigengrasp
	const EigenGrasp* getGrasp(int g) const {return mGrasps[g];}
	//! Returns the size of the eigengrasp space (the number of eigengrasps)
	int getSize() const {return (int)mGrasps.size();}

	//! Returns whether the interface is rigid.
	bool isRigid() const {return mRigid;}
	//! Sets the interface to rigid mode or non-rigis mode
	/*! In rigid motion, only configuration that are strictly inside the
		EG subspace are allowed. Otherwise, any configuration is allowed. 
		This is relevant when getAmp(...) and getDOF(...) are used. */
	void setRigid(bool r) const {mRigid = r;}

	//! Saves all the eigengrasps that define the subspace to a file, then also writes the origin
	int writeToFile(const char *filename);
	//! Loads all the eigengrasps, as well as the origin, from a file
	int readFromFile(QString filename);
	//! The trivial set of EG's is the identity set, where the EG (sub)space is identical to the dof space
	int setTrivial();

	//! Sets the name of this space; used used to show what file these eigengrasps were loaded from
	void setName(QString n){mName = n;}
	//! Gets the name of this space; used only for gui and debug purposes
	const QString getName() const {return mName;}

	//! Sets the origin of the eigengrasp space, as a point in the dof space
	void setOrigin(const double *dof);
	//! Sets the origin of eigengrasp space as the point halfway between each dof's range
	void setSimpleOrigin();

	//! Re-computes the min and max allowable amplitudes along each EG direction. 
	void setMinMax();
	//! Checks if the origin of the eigengrasp space is inside the legal range of the dofs
	void checkOrigin();

	//! Sets one of the eigengrasps as "fixed", meaning no movement is allowed along it
	/*! This is relevant when a point in dof space has to be projected in eigengrasp
		space and then back to dof space; if a certain eigengrasp is fixed, the projection
		is forced to assume the fixed value for that eigengrasp.
	*/
	void fixEigenGrasp(int i, double fa){mGrasps[i]->fix(fa);}
	//! Removes the "fixed" tag from an eigengrasp, allowing free movement
	void unfixEigenGrasp(int i){mGrasps[i]->unfix();}

	// These are the main functions that interface the EG:

	//! Converts a set of eigengrasp amplitudes into dof values
	void getDOF (const double *amp, double *dof) const;
	//! Converts a set of dof values to eigengrasp amplitudes
	void getAmp(double *amp, const double *dof) const;
};

#endif
