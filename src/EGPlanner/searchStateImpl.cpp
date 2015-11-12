//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: searchStateImpl.cpp,v 1.9 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "searchStateImpl.h"
#include "robot.h"
#include "eigenGrasp.h"

void PostureStateDOF::createVariables()
{
	QString name("DOF ");
	QString num;
	for (int i=0; i<mHand->getNumDOF(); i++) {
		num.setNum(i);
		mVariables.push_back( new SearchVariable(name+num, mHand->getDOF(i)->getMin(), mHand->getDOF(i)->getMax(),
										0.5 * (mHand->getDOF(i)->getMax() - mHand->getDOF(i)->getMin()) ));
	}
}
void PostureStateDOF::getHandDOF(double *dof) const
{
	for (int i=0; i<mHand->getNumDOF(); i++) {
		dof[i] = readVariable(i);
	}
}
void PostureStateDOF::storeHandDOF(const double *dof)
{
	for (int i=0; i<mHand->getNumDOF(); i++) {
		getVariable(i)->setValue(dof[i]);
	}
}

void PostureStateEigen::createVariables()
{
	QString name("EG ");
	QString num;
	float min,max;

	//this is a horrible hack to adapt EG range to the characteristics of the hand
	//should really compute these based on DOF max / min, but for some reason
	//I've never been able to do that properly
	if ( mHand->isA("Pr2Gripper") ) {
		min = -0.6f;
		max = 0.6f;
	} else {
		min = -4.0f;
		max = 4.0f;
	}

	for (int i=0; i<mHand->getEigenGrasps()->getSize(); i++) {
		num.setNum(i);
		mVariables.push_back( new SearchVariable(name+num, min, max, (max-min) / 4.0) );
	}
}
void PostureStateEigen::getHandDOF(double *dof) const
{
	double *eg = new double[ mHand->getEigenGrasps()->getSize() ];
	for (int i=0; i < mHand->getEigenGrasps()->getSize(); i++) {
		eg[i] = readVariable(i);
	}
	bool r = mHand->getEigenGrasps()->isRigid();
	mHand->getEigenGrasps()->setRigid(true);
	mHand->getEigenGrasps()->getDOF( eg, dof );
	mHand->checkSetDOFVals(dof);
	mHand->getEigenGrasps()->setRigid(r);
	delete [] eg;
}
void PostureStateEigen::storeHandDOF(const double *dof)
{
	double *eg = new double[ mHand->getEigenGrasps()->getSize() ];
	mHand->getEigenGrasps()->getAmp(eg, dof);
	for (int i=0; i < mHand->getEigenGrasps()->getSize(); i++) {
		getVariable(i)->setValue(eg[i]);
	}
	delete [] eg;
}

// ------------------------------------ POSITION STATES -------------------------------

void PositionStateComplete::createVariables()
{
	mVariables.push_back( new SearchVariable("Tx",-250,250,100) );
	mVariables.push_back( new SearchVariable("Ty",-250,250,100) );
	mVariables.push_back( new SearchVariable("Tz",-250,250,100) );
	mVariables.push_back( new SearchVariable("Qw",-5, 5, 1) );
	mVariables.push_back( new SearchVariable("Qx",-5, 5, 1) );
	mVariables.push_back( new SearchVariable("Qy",-5, 5, 1) );
	mVariables.push_back( new SearchVariable("Qz",-5, 5, 1) );
}

/*! This is a redundant way of saving the transform (since the Q 
	doesn't need 4 variables since it's normalized. It's just the 
	simplest way to interface with a transf so it's used as an 
	interface between other types. Not really meant for performing 
	searches.
*/
transf PositionStateComplete::getCoreTran() const
{
	double tx = readVariable("Tx");
	double ty = readVariable("Ty");
	double tz = readVariable("Tz");
	double qw = readVariable("Qw");
	double qx = readVariable("Qx");
	double qy = readVariable("Qy");
	double qz = readVariable("Qz");
	return transf(Quaternion(qw,qx,qy,qz), vec3(tx,ty,tz));	
}
void PositionStateComplete::setTran(const transf &t)
{
	getVariable("Tx")->setValue( t.translation().x() );
	getVariable("Ty")->setValue( t.translation().y() );
	getVariable("Tz")->setValue( t.translation().z() );
	getVariable("Qw")->setValue( t.rotation().w );
	getVariable("Qx")->setValue( t.rotation().x );
	getVariable("Qy")->setValue( t.rotation().y );
	getVariable("Qz")->setValue( t.rotation().z );
}

void PositionStateAA::createVariables()
{
	mVariables.push_back( new SearchVariable("Tx",-250,250,150) );
	mVariables.push_back( new SearchVariable("Ty",-250,250,150) );
	mVariables.push_back( new SearchVariable("Tz",-250,250,150) );
	mVariables.push_back( new SearchVariable("theta", 0, M_PI, M_PI/5) );
	mVariables.push_back( new SearchVariable("phi", -M_PI, M_PI, M_PI/2, true) );
	mVariables.push_back( new SearchVariable("alpha",0, M_PI, M_PI/2) );
}
transf PositionStateAA::getCoreTran() const
{
	double tx = readVariable("Tx");
	double ty = readVariable("Ty");
	double tz = readVariable("Tz");
	double theta = readVariable("theta");
	double phi = readVariable("phi");
	double alpha = readVariable("alpha");
	return rotate_transf(alpha, vec3( sin(theta)*cos(phi) , sin(theta)*sin(phi) , cos(theta) )) *
		   translate_transf(vec3(tx,ty,tz));
}
void PositionStateAA::setTran(const transf &t)
{
	mat3 m_inv = t.affine().inverse();
	vec3 unrotatedTranslation = m_inv*t.translation();
	double alpha;
	vec3 axis;
	t.rotation().ToAngleAxis(alpha, axis);
	double theta = acos(axis.z());
	double phi = atan2(axis.y(), axis.z());
	
	getVariable("Tx")->setValue( unrotatedTranslation.x() );
	getVariable("Ty")->setValue( unrotatedTranslation.y() );
	getVariable("Tz")->setValue( unrotatedTranslation.z() );
	getVariable("theta")->setValue( theta);
	getVariable("phi")->setValue( phi );
	getVariable("alpha")->setValue( alpha );
}

void PositionStateEllipsoid::createVariables()
{
	mVariables.push_back( new SearchVariable("beta",-M_PI/2.0, M_PI/2.0, M_PI/2.0) );
	mVariables.push_back( new SearchVariable("gamma",-M_PI, M_PI, M_PI, true) );
	mVariables.push_back( new SearchVariable("tau",-M_PI, M_PI, M_PI, true) );
	mVariables.push_back( new SearchVariable("dist",-50, 100, 50) );

	//ellipsoid scaling parameters
	mParameters.push_back(SearchParameter("a",80));
	mParameters.push_back(SearchParameter("b",80));
	mParameters.push_back(SearchParameter("c",160));
}
transf PositionStateEllipsoid::getCoreTran() const
{
	double a = getParameter("a");
	double b = getParameter("b");
	double c = getParameter("c");
	double beta = readVariable("beta");
	double gamma = readVariable("gamma");
	double tau = readVariable("tau");
	double distance = readVariable("dist");
	double px,py,pz;
	px =  a * cos(beta) * cos(gamma);
	py =  b * cos(beta) * sin(gamma);
	pz =  c * sin(beta);
	
	//compute normal direction - for some reason this always points INSIDE the ellipsoid
	vec3 n1(  -a*sin(beta)*cos(gamma), -b*sin(beta)*sin(gamma), c*cos(beta) );
	vec3 n2(  -a*cos(beta)*sin(gamma),  b*cos(beta)*cos(gamma), 0 );
	vec3 normal = normalise(n1) * normalise(n2);

	vec3 xdir(1,0,0);
	vec3 ydir = normal * normalise(xdir);
	xdir = ydir * normal;
	mat3 r(xdir,ydir,normal);

	transf handTran = transf(r, vec3(px,py,pz) - distance * normal);
	Quaternion zrot(tau, vec3(0,0,1));
	handTran = transf(zrot, vec3(0,0,0) ) * handTran;
	return mHand->getApproachTran().inverse() * handTran;
	//So: hand tranform is: move onto the ellipsoid and rotate z axis in normal direction
	//	  --> hand approach transform inverted to get to hand origin
}
void PositionStateEllipsoid::setTran(const transf &t)
{
double a = getParameter("a");
	double b = getParameter("b");
	double c = getParameter("c");
	
	// first get handTran
	transf handTran = mHand->getApproachTran() * t;
	vec3 normal;
	/*There may be more intelligent ways of deconstructing this
	but using the nomenclature established in getCoreTran:
	normal = [nx ny nz], ct = cos(tau), st = sin(tau)
	ydir = [0 nz -ny]  
	xdir = [(nz^2 + ny^2) (-ny*nx) (-nz*nx)]
	d = distance
	handTran matrix currently is the following - R(tau) * transf(r, vec3(px,py,pz) - distance * normal)
	= 
	[ct -st 0 0
	st ct 0 0
	0  0  1 0
	0  0  0  1] *
	[(nz^2 + ny^2)		0		nx		px - d*nx]
	-ny*nx				nz		ny		py - d*ny
	-nz*nx				-ny		nz		pz - d*nz
	0					0		0		1		]
	=
	[(ct*(nz^2 + ny^2)+(st*ny*nx)		-st*nz		ct*nx-st*ny		ct*(px-d*nx) - st*(py-d*ny)
	st*((nz^2 + ny^2)-(ct*nx*ny)		ct*nz		st*nx+ct*ny		 st*(px-d*nx) + ct*(py-d*ny)
	-nz*nx								-ny			nz				pz - d*nz
	0									0			0				1]
	*/
	normal[2] = handTran.affine()[2,2];
	normal[1] = -handTran.affine()[2,1];
	normal[0] =  -handTran.affine()[2,0]/normal[2];
	double tau = atan2(-handTran.affine()[0,1],handTran.affine()[1,1]);
	normal = normalise(normal); //normal is supposed to be of unit magnitude
	//unrotate handtran by tau around z
	handTran = rotate_transf(-tau, vec3(0,0,1)) * handTran;
	/*
	px =  a * cos(beta) * cos(gamma);
	py =  b * cos(beta) * sin(gamma);
	pz =  c * sin(beta);
	

	and 
	//using matlab, one can derive the following with gamma = g:
	normal = [ -c/(pow(a,2)*cos(g)^2-pow(a,2)*cos(g)^2*cos(beta)^2+pow(b,2)-pow(b,2)*cos(g)^2-pow(b,2)*cos(beta)^2+pow(b,2)*cos(beta)^2*cos(g)^2+c^2*cos(beta)^2)^(1/2)*b*cos(g)/(-pow(a,2)*cos(g)^2+pow(b,2)*cos(g)^2+pow(a,2))^(1/2)*abs(cos(beta)),
	      -c/(pow(a,2)*cos(g)^2-pow(a,2)*cos(g)^2*cos(beta)^2+pow(b,2)-pow(b,2)*cos(g)^2-pow(b,2)*cos(beta)^2+pow(b,2)*cos(beta)^2*cos(g)^2+c^2*cos(beta)^2)^(1/2)*a*sin(g)/(-pow(a,2)*cos(g)^2+pow(b,2)*cos(g)^2+pow(a,2))^(1/2)*abs(cos(beta)),
		-a*sin(beta)*b/(pow(a,2)*cos(g)^2-pow(a,2)*cos(g)^2*cos(beta)^2+pow(b,2)-pow(b,2)*cos(g)^2-pow(b,2)*cos(beta)^2+pow(b,2)*cos(beta)^2*cos(g)^2+c^2*cos(beta)^2)^(1/2)*signum(cos(beta))/(-pow(a,2)*cos(g)^2+pow(b,2)*cos(g)^2+pow(a,2))^(1/2)]
	from this:
	(ny/a)/(nx/b) = sin(g)/cos(g)= tan(gamma)  
	gamma = atan((ny/a)/(nx/b))
	d = h[0][3]+px/nx = h1 + a*cos(beta)*cos(g)/nx -> eq1
	d = h[1][3]+py/ny = h2 + b*cos(beta)*sin(g)/ny -> eq2
	d = h[2][3]+pz/nz = h3 + c*sin(beta)/nz -> eq3
	solve for d by matlab (by solve eq1, eq2, and eq3 in terms of functions of beta and gamma, squaring both sides, adding all three equations together
	and setting the results equal to one.):
	d = nz^2*b^2*a^2*h3+ny^2*a^2*c^2*h2+nx^2*b^2*c^2*h1+(2*nz^2*b^2*a^4*h3*ny^2*c^2*h2+2*nz^2*b^4*a^2*h3*nx^2*c^2*h1+2*ny^2*a^2*c^4*h2*nx^2*b^2*h1-nx^2*b^4*c^2*nz^2*a^2*h3^2-nx^2*b^2*c^4*ny^2*a^2*h2^2+nx^2*b^4*c^4*a^2-ny^2*a^4*c^2*nz^2*b^2*h3^2-ny^2*a^2*c^4*nx^2*b^2*h1^2+ny^2*a^4*c^4*b^2-nz^2*b^2*a^4*ny^2*c^2*h2^2-nz^2*b^4*a^2*nx^2*c^2*h1^2+nz^2*b^4*a^4*c^2)^(1/2))/(nx^2*b^2*c^2+ny^2*a^2*c^2+nz^2*b^2*a^2
	there must be an easier way to do this, but I don't see it off hand
	*/
	double gamma = atan2(normal[1]/a, normal[0]/b);
	double h1 = handTran.translation()[0], h2 = handTran.translation()[1], h3 = handTran.translation()[2];
	double nx = normal[0], ny = normal[1], nz = normal[2];
	//powers
	double a_2 = pow(a,2), a_4= pow(a,4), b_2 = pow(b,2), b_4 = pow(b,4), c_2 = pow(c,2), c_4 = pow(c,4), h1_2 = pow(h1,2), h2_2 = pow(h2,2), h3_2 = pow(h3,2), nx_2 = pow(nx,2),ny_2 = pow(ny,2),nz_2 = pow(nz,2);
	double d = nz_2*b_2*a_2*h3+ny_2*a_2*c_2*h2+nx_2*b_2*c_2*h1+sqrt(2*nz_2*b_2*a_4*h3*ny_2*c_2*h2+2*nz_2*b_4*a_2*h3*nx_2*c_2*h1+2*ny_2*a_2*c_4*h2*nx_2*b_2*h1-nx_2*b_4*c_2*nz_2*a_2*h3_2-nx_2*b_2*c_4*ny_2*a_2*h2_2+nx_2*b_4*c_4*a_2-ny_2*a_4*c_2*nz_2*b_2*h3_2-ny_2*a_2*c_4*nx_2*b_2*h1_2+ny_2*a_4*c_4*b_2-nz_2*b_2*a_4*ny_2*c_2*h2_2-nz_2*b_4*a_2*nx_2*c_2*h1_2+nz_2*b_4*a_4*c_2)/(nx_2*b_2*c_2+ny_2*a_2*c_2+nz_2*b_2*a_2);
	double beta = atan2((handTran.translation()[2]+d*nz)/c, (handTran.translation()[1]+d*ny)/sin(gamma)/b);
	getVariable("beta")->setValue(beta);
	getVariable("gamma")->setValue(gamma);
	getVariable("tau")->setValue(tau);
	getVariable("dist")->setValue(d);

}

void PositionStateApproach::createVariables()
{
#ifndef SEMANTIC_PLANNER_ENABLED
	mVariables.push_back( new SearchVariable("dist",-30, 200, 100) );
	mVariables.push_back( new SearchVariable("wrist 1",-M_PI/3.0, M_PI/3.0, M_PI/6.0) );
	mVariables.push_back( new SearchVariable("wrist 2",-M_PI/3.0, M_PI/3.0, M_PI/6.0) );
#else
	mVariables.push_back( new SearchVariable("dist",-30, 300, 200) );
	mVariables.push_back( new SearchVariable("wrist 1",-M_PI/12.0, M_PI/12.0, M_PI/24.0) );
	mVariables.push_back( new SearchVariable("wrist 2",-M_PI/12.0, M_PI/12.0, M_PI/24.0) );
	mVariables.push_back( new SearchVariable("wrist 3",-M_PI/6.0, M_PI/6.0, M_PI/12.0) );
	mVariables.push_back( new SearchVariable("wrist 4",-50, 50, 20) );
	mVariables.push_back( new SearchVariable("wrist 5",-50, 50, 20) );
#endif
}
transf PositionStateApproach::getCoreTran() const
{
#ifndef SEMANTIC_PLANNER_ENABLED
	double dist = readVariable("dist");
	double rx = readVariable("wrist 1");
	double ry = readVariable("wrist 2");
	transf handTran = transf(Quaternion::IDENTITY, vec3(0,0,dist));
	handTran = handTran * rotate_transf(rx, vec3(1,0,0) ) * rotate_transf( ry, vec3(0,1,0) );
#else
	double dist = readVariable("dist");
	double rx = readVariable("wrist 1");
	double ry = readVariable("wrist 2");
	double rz = readVariable("wrist 3");
	double dx = readVariable("wrist 4");
	double dy = readVariable("wrist 5");

	transf handTran = transf(Quaternion::IDENTITY, vec3(dx, dy,dist));
	handTran = handTran * rotate_transf(rx, vec3(1,0,0) ) * rotate_transf( ry, vec3(0,1,0) ) * rotate_transf( rz, vec3(0,0,1) ) ;
#endif

	return mHand->getApproachTran().inverse() * handTran * mHand->getApproachTran();
}
void PositionStateApproach::setTran(const transf &t)
{
	//This function is totally wrong at present 5.8.2012

	//First get handTran
	transf handTran = mHand->getApproachTran() * t * mHand->getApproachTran().inverse();
	double d = handTran.translation().z();
	/*the rotation part of the transform matrix looks like this:
		[cos(ry) 0 sin(ry)
		-sin(rx)sin(ry) cos(rx) -sin(rx)cos(ry)
		cos(rx)sin(ry) sin(rx) cos(rx)cos(ry)]
	*/
	double rx = atan2(handTran.affine()[2,1],handTran.affine()[1,1]);
	double ry = atan2(handTran.affine()[0,2],handTran.affine()[0,0]);


	getVariable("dist")->setValue(d);
	getVariable("wrist 1")->setValue(rx);
	getVariable("wrist 2")->setValue(ry);

}

void PositionStateLocal::createVariables()
{
	mVariables.push_back( new SearchVariable("dist",-30, 30, 15) );
	mVariables.push_back( new SearchVariable("wrist 1",-M_PI/12.0, M_PI/12.0, M_PI/24.0) );
	mVariables.push_back( new SearchVariable("wrist 2",-M_PI/12.0, M_PI/12.0, M_PI/24.0) );
	mVariables.push_back( new SearchVariable("wrist 3",-M_PI/6.0, M_PI/6.0, M_PI/12.0) );
	mVariables.push_back( new SearchVariable("wrist 4",-30, 30, 10) );
	mVariables.push_back( new SearchVariable("wrist 5",-30, 30, 10) );

}
transf PositionStateLocal::getCoreTran() const
{
	//This function is totally wrong at present 5.8.2012

	double dist = readVariable("dist");
	double rx = readVariable("wrist 1");
	double ry = readVariable("wrist 2");
	double rz = readVariable("wrist 3");
	double dx = readVariable("wrist 4");
	double dy = readVariable("wrist 5");

	transf handTran = transf(Quaternion::IDENTITY, vec3(dx, dy,dist));
	handTran = handTran * rotate_transf(rx, vec3(1,0,0) ) * rotate_transf( ry, vec3(0,1,0) ) * rotate_transf( rz, vec3(0,0,1) ) ;

	return mHand->getApproachTran().inverse() * handTran * mHand->getApproachTran();
}
void PositionStateLocal::setTran(const transf &t)
{
	//First get handTran
	//transf handTran = mHand->getApproachTran() * t * mHand->getApproachTran().inverse();
	//double d = handTran.translation().z();
	///*the rotation part of the transform matrix looks like this:
	//	[cos(ry) 0 sin(ry)
	//	-sin(rx)sin(ry) cos(rx) -sin(rx)cos(ry)
	//	cos(rx)sin(ry) sin(rx) cos(rx)cos(ry)]
	//*/
	//double rx = atan2(handTran.affine()[2,1],handTran.affine()[1,1]);
	//double ry = atan2(handTran.affine()[0,2],handTran.affine()[0,0]);


	getVariable("dist")->setValue(0);
	getVariable("wrist 1")->setValue(0);
	getVariable("wrist 2")->setValue(0);
	getVariable("wrist 3")->setValue(0);
	getVariable("wrist 4")->setValue(0);
	getVariable("wrist 5")->setValue(0);

}
