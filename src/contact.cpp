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
// Author(s): Andrew T. Miller 
//
// $Id: contact.cpp,v 1.56 2009/08/14 18:09:43 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the contact class
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

#include "matvec3D.h"
#include "contact.h"
#include "world.h"
#include "body.h"
#include "mytools.h"
#include "matrix.h"
#include "FitParabola.h"

//#define GRASPITDBG
#include "debug.h"

const double Contact::THRESHOLD = 0.1;
const double Contact::INHERITANCE_THRESHOLD = 1;
const double Contact::INHERITANCE_ANGULAR_THRESHOLD = 0.984; //cosine of 10 degrees

/*!
  Initializes a new contact on body \a b1.  The other contacting body is \a b2.
  The contact position \a pos and the contact normal \a norm are expressed
  in local body \a b1 coordinates.
*/
Contact::Contact(Body *b1, Body *b2, position pos, vec3 norm)
{
  body1 = b1;
  body2 = b2;
  mate = NULL;
  wrench = NULL;
  body1Tran = b1->getTran();
  body2Tran = b2->getTran();
  
  updateCof();
  normal = normalise(norm);
  loc = pos;
  vec3 tangentX,tangentY;

  if (fabs(normal % vec3(1,0,0)) > 1.0 - MACHINE_ZERO) {
    tangentX = normalise(normal * vec3(0,0,1));
  } else {
    tangentX = normalise(normal * vec3(1,0,0));
  }
  tangentY = normalise(normal * tangentX);  
  frame = coordinate_transf(loc,tangentX,tangentY);			 
  coneMat = NULL;
  prevBetas = NULL;
  inheritanceInfo = false;
  for (int i=0; i<6; i++) {
	  dynamicForce[i] = 0;
  }
}

/*!
  Deletes contact basis vectors, optimal force coefficients, and
  friction cone boundary wrenches.  If the contact has an undeleted mate,
  it removes the mate's connection to this contact, and removes the mate
  contact from the other body (thus deleting it).
*/
Contact::~Contact()
{
  if (optimalCoeffs) delete [] optimalCoeffs;
  if (wrench) delete [] wrench;
  if (prevBetas) delete [] prevBetas;

  if (mate) {
    mate->setMate(NULL);
    body2->removeContact(mate);
  }
  DBGP("Contact deleted");
}

/*! Friction edges contain "normalised" friction information: just the 
	frictional component, and without reference to normal force or coeff
	of friction (c.o.f.). They only care about the relationship between 
	frictional components.
	
	To get an actual wrench that can be applied at the contact, we must add
	normal force (normalised here to 1N) and take into account the relationship
	btw normal force and c.o.f.
*/
void Contact::wrenchFromFrictionEdge(double *edge, const vec3 &radius, Wrench *wr)
{
	vec3 tangentX = frame.affine().row(0);
	vec3 tangentY = frame.affine().row(1);

	GraspableBody *object = (GraspableBody *)body1;

	//max friction is normal_force_magnitude (which is 1) * coeff_of_friction 
	//possible friction for this wrench is (friction_edge * max_friction) in the X and Y direction of the contact
	vec3 forceVec = normal + (tangentX*cof*edge[0])+ (tangentY*cof*edge[1]);
	
	//max torque is contact_radius * normal_force_magnitude (which is 1) * coeff_of_friction
	//possible torque for this wrench is (friction_edge * max_torque) in the direction of the contact normal
	//the contact_radius coefficient is already taken into account in the friction_edge
	vec3 torqueVec = ( (radius*forceVec) + normal*cof*edge[5] )/object->getMaxRadius();
	
	wr->torque = torqueVec;
	wr->force = forceVec;
}

/*! Takes the information about pure friction at this contact, contained in the
	frictionEdges, and adds normal force and the effect of coeff of friction
	to obtain actual wrenches that can be applied at this contact. Essentially
	builds the Contact Wrench Space based on the friction information and
	(normalized) contact forces. See wrenchFromFrictionEdge for details.  
*/
void Contact::computeWrenches()
{
  DBGP("COMPUTING WRENCHES");

  if (wrench) delete [] wrench;
  //one wrench for each vector
  numFCWrenches = numFrictionEdges;
  wrench = new Wrench[numFCWrenches];

  vec3 radius = loc - ((GraspableBody*)body1)->getCoG();
  for (int i=0;i<numFCWrenches;i++) {
	  wrenchFromFrictionEdge( &frictionEdges[6*i], radius, &wrench[i] );
	}
}


/*! Sets up the friction edges of this contact using an ellipsoid 
	approximation. This is convenience function, as friction edges can
	be set in many ways. However, we currently use PCWF and SFC models
	which are both cases of linearized ellipsoids, so this function
	can be used for both.
	
	Consider a 3D friction ellipsoid, where the first two dimensions are
	tangential frictional force (along X and Y) and the third is frictional
	torque (along Z). This function samples this ellipsoid at \a numLatitudes
	latitudes contained in \a phi[]; at each latitude l it takes \a numDirs[l] 
	equally spaced discrete samples. Each of those samples becomes a friction
	edge, after it is converted to the full 6D space by filling in the other
	dimensions with zeroes.
 */
int
Contact::setUpFrictionEllipsoid(int numLatitudes, int numDirs[], double phi[], double eccen[])
{
	numFrictionEdges = 0;
	for (int i=0;i<numLatitudes;i++) {
		numFrictionEdges += numDirs[i];
	}
	if (numFrictionEdges > MAX_FRICTION_EDGES) return FAILURE;
	prevBetas = new double[numFrictionEdges];

	int col = 0;
	for (int i=0;i<numLatitudes;i++) {
		double cosphi = cos(phi[i]);
		double sinphi = sin(phi[i]);
		for (int j=0;j<numDirs[i];j++) {
			double theta = j * 2*M_PI/numDirs[i];
      
			double num = cos(theta)*cosphi;
			double denom = num*num/(eccen[0]*eccen[0]);
			num = sin(theta)*cosphi;
			denom += num*num/(eccen[1]*eccen[1]);
			num = sinphi;
			denom += num*num/(eccen[2]*eccen[2]);
			denom = sqrt(denom);

			frictionEdges[col*6]   = cos(theta)*cosphi/denom;
			frictionEdges[col*6+1] = sin(theta)*cosphi/denom;
			frictionEdges[col*6+2] = 0;
			frictionEdges[col*6+3] = 0;
			frictionEdges[col*6+4] = 0;
			frictionEdges[col*6+5] = sinphi/denom;
			col++;
		}    
	}
	return SUCCESS;
}

/*! Assembles a friction constraint matrix for this contact that can be used in
	an LCP. This matrix is of the form [-mu 1 1 ... 1] with a 1 for each friction
	edge. This is used in a constraint as F * beta <= 0, saying that the sum of
	friction edge amplitudes (thus friction force) must be less than normal force 
	times friction coefficient.
*/
Matrix 
Contact::frictionConstraintsMatrix() const
{
	Matrix F(1, 1+numFrictionEdges);
	F.setAllElements(1.0);
	F.elem(0,0) = -1.0 * getCof();
	return F;
}

/*! Returns the matric that relates friction edge amplitudes to friction force. That
	matrix is of the form [n D] where n is the contact normal and D has as columns
	the friction edges. The computations are done in local contact coordinate system
	(contact normal along the z axis).
*/
Matrix 
Contact::frictionForceMatrix() const
{
	Matrix Rfi(6, numFrictionEdges + 1);
	//the column for the normal force;
	//in local contact coordinates the normal always points in the z direction
	Rfi.elem(0,0) = Rfi.elem(1,0) = 0.0; Rfi.elem(2,0) = 1.0;
	Rfi.elem(3,0) = Rfi.elem(4,0) = Rfi.elem(5,0) = 0.0;
	//the columns for the friction edges
	for(int edge=0; edge<numFrictionEdges; edge++) {
		for(int i=0; i<6; i++) {
			Rfi.elem(i, edge+1) = frictionEdges[6*edge+i];
		}
	}
	//flip the sign of Rfi to get contact pointing in the right direction
	Rfi.multiply(-1.0);
	return Rfi;
}

/*! Creates the individual force matrices for all contacts in the list using
	frictionConstraintMatrix() then assembles them in block diagonal form.
*/
Matrix
Contact::frictionForceBlockMatrix(const std::list<Contact*> &contacts)
{
	if (contacts.empty()) {
		return Matrix(0,0);
	}
	std::list<Matrix*> blocks;
	std::list<Contact*>::const_iterator it;
	for (it=contacts.begin(); it!=contacts.end(); it++) {
		blocks.push_back( new Matrix((*it)->frictionForceMatrix()) );
	}
	Matrix Rf(Matrix::BLOCKDIAG<Matrix>(&blocks));
	while (!blocks.empty()) {
		delete blocks.back();
		blocks.pop_back();
	}
	return Rf;
}

/*! Creates the friction constraint matrices of all contacts in the list using
	Contact::frictionConstraintMatrix(), then assembles all the matrices in 
	block diagonal form.
*/
Matrix 
Contact::frictionConstraintsBlockMatrix(const std::list<Contact*> &contacts)
{
	if (contacts.empty()) {
		return Matrix(0,0);
	}
	std::list<Matrix*> blocks;
	std::list<Contact*>::const_iterator it;
	for (it=contacts.begin(); it!=contacts.end(); it++) {
		blocks.push_back( new Matrix((*it)->frictionConstraintsMatrix()) );
	}
	Matrix blockF(Matrix::BLOCKDIAG<Matrix>(&blocks));
	while (!blocks.empty()) {
		delete blocks.back();
		blocks.pop_back();
	}
	return blockF;
}

/*! Creates a line vector that, when multiplied by a vector of contact wrench
	amplitudes returns the sum of the normal components. Therefore, it has 1
	in the positions corresponding to normal force amplitudes and 0 in the
	positions corresponding to friction wrench amplitudes.
*/
Matrix 
Contact::normalForceSumMatrix(const std::list<Contact*> &contacts)
{
	if (contacts.empty()) {
		return Matrix(0,0);
	}
	std::list<Matrix*> blocks;
	std::list<Contact*>::const_iterator it;
	for (it=contacts.begin(); it!=contacts.end(); it++) {
		blocks.push_back( new Matrix(Matrix::ZEROES<Matrix>(1, (*it)->numFrictionEdges+1)) );
		blocks.back()->elem(0,0) = 1.0;
	}
	Matrix blockF(Matrix::BLOCKROW<Matrix>(&blocks));
	while (!blocks.empty()) {
		delete blocks.back();
		blocks.pop_back();
	}
	return blockF;
}

/*! The matrix, when multiplied with a wrench applied at this contact will give the
	resultant wrench applied on the other body in contact (thus computed relative
	to that object's center of mass), expressed in world coordinates.

	The matrix looks like this:
	| R 0 |
	|CR R |
	Where R is the 3x3 rotation matrix between the coordinate systems and C also 
	contains the cross product matrix that depends on the translation between them.	
*/
Matrix 
Contact::localToWorldWrenchMatrix() const
{
	Matrix Ro(Matrix::ZEROES<Matrix>(6,6));
	transf contactTran = getContactFrame() * getBody1()->getTran();
	mat3 R; contactTran.rotation().ToRotationMatrix(R);
	Matrix Rot( Matrix::ROTATION(R) );
	//the force transform is simple, just the matrix that changes coord. systems
	Ro.copySubMatrix(0, 0, Rot);
	Ro.copySubMatrix(3, 3, Rot);
	//for torque we also multiply by a cross product matrix
	vec3 worldLocation = contactTran.translation();
	vec3 cog = getBody2()->getTran().translation();
	mat3 C; C.setCrossProductMatrix(worldLocation - cog); 
	Matrix CR(3,3);
	matrixMultiply(Matrix::ROTATION(C.transpose()), Rot, CR);
	//also scale by object max radius so we get same units as force
	//and optimization does not favor torque over force
	double scale = 1.0;
	if (getBody2()->isA("GraspableBody")) {
		scale = scale / static_cast<GraspableBody*>(getBody2())->getMaxRadius();
	}
	CR.multiply(scale);
	Ro.copySubMatrix(3, 0, CR);
	return Ro;
}

/*! Assembles together the localToWorldWrenchMatrix for all the contacts in the list
	in block diagonal form.
*/
Matrix 
Contact::localToWorldWrenchBlockMatrix(const std::list<Contact*> &contacts)
{
	if (contacts.empty()) {
		return Matrix(0,0);
	}
	std::list<Matrix*> blocks;
	std::list<Contact*>::const_iterator it;
	for (it=contacts.begin(); it!=contacts.end(); it++) {
		blocks.push_back( new Matrix((*it)->localToWorldWrenchMatrix()) );
	}
	Matrix Ro(Matrix::BLOCKDIAG<Matrix>(&blocks));
	while (!blocks.empty()) {
		delete blocks.back();
		blocks.pop_back();
	}
	return Ro;
}

/*!
  First computes the new location of the contact point using the motion
  transform (expressed with respect to the body coordinate frame).  If the
  dot product of the contact point motion vector and the contact normal is
  less than zero, then the contact prevents this motion.
*/
bool Contact::preventsMotion(const transf& motion) const
{  
  if ( (loc*motion - loc) % normal < -MACHINE_ZERO) return true;
  return false;
}


/*!
  Recomputes the COF for this contact.  This is called when the material of
  one of the two bodies is changed.
*/
void
Contact::updateCof()
{
  cof = body1->getWorld()->getCOF(body1->getMaterial(),body2->getMaterial());
  kcof = body1->getWorld()->getKCOF(body1->getMaterial(),body2->getMaterial());
  body1->setContactsChanged();
  body2->setContactsChanged();
}

/*!
  Returns the correct coefficient of friction for this contact.  If either
  body is dynamic, and the relative velocity between them is greater than
  1.0 mm/sec (should be made a parameter), then it returns the kinetic COF,
  otherwise it returns the static COF.
*/
double
Contact::getCof() const
{
  DynamicBody *db;
  vec3 radius,vel1(vec3::ZERO),vel2(vec3::ZERO),rotvel;

  if (body1->isDynamic()) {
    db = (DynamicBody *)body1;
    radius = db->getTran().rotation() * (loc - db->getCoG());
    vel1.set(db->getVelocity()[0],db->getVelocity()[1],db->getVelocity()[2]);
    rotvel.set(db->getVelocity()[3],db->getVelocity()[4],db->getVelocity()[5]);
    vel1 += radius * rotvel;
  }
  if (body2->isDynamic()) {
    db = (DynamicBody *)body2;
    radius = db->getTran().rotation() * (mate->loc - db->getCoG());
    vel2.set(db->getVelocity()[0],db->getVelocity()[1],db->getVelocity()[2]);
    rotvel.set(db->getVelocity()[3],db->getVelocity()[4],db->getVelocity()[5]);
    vel2 += radius * rotvel;
  }
  if ((vel1 - vel2).len() > 1.0) {
    DBGP("SLIDING!");
    return kcof;
  }
  else return cof;
}

/*!
  When the grasp force optimization completes it calls this routine to set
  this contact's optimal force.  This force is a compromise between minimizing
  internal grasp forces and how close the force is to the boundary of the
  friction cone, or starting to slip.
*/
void
Contact::setContactForce (double *optmx)
{
  for (int i=0;i<contactDim;i++)
    optimalCoeffs[i] = optmx[i];
}

/*!
  Each body has a thin layer around it that is Contact::THRESHOLD mm thick, and
  when another body is within that layer, the two bodies are in contact.
  During dynamic simulation, contacts provide constraints to prevent the two
  bodies from interpenetrating.  However, the constraint is a velocity
  constraint not a position one, so errors in position due to numerical issues
  can occur.  If the two bodies get closer than half the contact threshold,
  we correct this specifying a constraint error in the dynamics, which will
  serve to move the bodies apart.  This routine returns the distance that
  two bodies have violated that halfway constraint.
*/
double
Contact::getConstraintError()
{
	transf cf = frame * body1->getTran();
	transf cf2 = mate->getContactFrame() * body2->getTran();
	return MAX(0.0,Contact::THRESHOLD/2.0 - (cf.translation() - cf2.translation()).len());
}

/*! Attempts to save some information from a previously computed dynamics 
	time step. The contact \a c is from the previous time step, but has been 
	determined to be close enough to this one that it is probably the same
	contact, having slightly evolved over a time step.
*/
void
Contact::inherit(Contact *c)
{
	inheritanceInfo = true;
	setLCPInfo( c->getPrevCn(), c->getPrevLambda(), c->getPrevBetas() );
	/*
	if (!coneMat || !c->coneMat) {
		return;
	}
	coneR = c->coneR;
	coneG = c->coneG;
	coneB = c->coneB;
	coneMat->diffuseColor = SbColor( coneR, coneG, coneB);
	coneMat->ambientColor = SbColor( coneR, coneG, coneB);
	coneMat->emissiveColor = SbColor( coneR, coneG, coneB);
	*/
}

void
Contact::setLCPInfo(double cn, double l, double *betas)
{
	prevCn = cn;
	prevLambda = l;
	for (int i=0; i<numFrictionEdges; i++) {
		prevBetas[i] = betas[i];
	}
	//lambda is the LCP paramter that indicates contact slip
	if (l>0) mSlip = true;
	else mSlip = false;
}

///////////////////////////////////////////
// Point contact
///////////////////////////////////////////

PointContact::PointContact(Body *b1,Body *b2,position pos, vec3 norm) : 
				Contact( b1, b2, pos, norm )
{
	//we should really have another class for the FL contact
	if (cof == 0.0) {
		frictionType = FL;
		contactDim = 1;
		lmiDim = 1;
	} else {
		frictionType = PCWF;
		contactDim = 3;
		lmiDim = 3;
	}
	optimalCoeffs = new double[contactDim]; 
	setUpFrictionEdges();
}

PointContact::~PointContact()
{
}

/*! Set up friction as a linearized circle; a PCWF can only have friction
	forces (no torques) in the tangential plane of the contact. When normal
	force will be added later, the friction circle becomes the more 
	familiar contact cone.
*/
int PointContact::setUpFrictionEdges(bool dynamicsOn)
{

	if (dynamicsOn) {
		//we don't really need to update anything; friction edges for point contacts do not change
		//during dynamic simulation
		return 0;
	}
	double eccen[3] = {1,1,1};
	int numDirs[1] = {8};
	double phi[1] = {0.0};
	return setUpFrictionEllipsoid(1,numDirs,phi,eccen); 
}

/*! Return a visual indicator showing the contact cone */
SoSeparator* 
PointContact::getVisualIndicator()
{
	double height,alpha,cof;
	SoSeparator *cne;
	SoTransform *tran;
	SoIndexedFaceSet *ifs;
	SoCoordinate3 *coords;

	SbVec3f *points = new SbVec3f[numFrictionEdges+1];
	int32_t *cIndex = new int32_t[5*numFrictionEdges+1];
	int i;

	points[0].setValue(0,0,0);

	//this is now a member of the class so no need to declare it here
	coneMat = new SoMaterial;  

    coneMat->diffuseColor = SbColor(0.8f,0.0f,0.0f);
    coneMat->ambientColor = SbColor(0.2f,0.0f,0.0f);
    coneMat->emissiveColor = SbColor(0.4f,0.0f,0.0f);
/*
	coneR = ((float)rand())/RAND_MAX;
	coneG = ((float)rand())/RAND_MAX;
	coneB = ((float)rand())/RAND_MAX;

	coneMat->diffuseColor = SbColor(coneR, coneG, coneB);
	coneMat->ambientColor = SbColor(coneR, coneG, coneB);
	coneMat->emissiveColor = SbColor(coneR, coneG, coneB);
*/
    coneMat->transparency = 0.8f;

    SoMaterial* zaxisMat = new SoMaterial;  
    zaxisMat->diffuseColor = SbColor(0,0,0);
    zaxisMat->ambientColor = SbColor(0,0,0);

	cof = getCof();

	height = Body::CONE_HEIGHT;
	cne = new SoSeparator;
	coords = new SoCoordinate3;
	ifs = new SoIndexedFaceSet;
	tran = new SoTransform;
  
	alpha = 0.0;
	for (i=0;i<numFrictionEdges;i++) {
		points[i+1].setValue(cos(alpha)*cof,sin(alpha)*cof,1.0);
		points[i+1] *= height;
		cIndex[4*i] = 0;
		cIndex[4*i+1] = (i+2 <= numFrictionEdges ? i+2 : 1);
		cIndex[4*i+2] =  i+1;
		cIndex[4*i+3] = -1;
		cIndex[4*numFrictionEdges+i] = i+1;
		alpha += 2.0*M_PI/numFrictionEdges;
	}
	cIndex[5*numFrictionEdges] = -1;
  
	coords->point.setValues(0,numFrictionEdges+1,points);
	ifs->coordIndex.setValues(0,5*numFrictionEdges+1,cIndex);
	delete [] points;
	delete [] cIndex;

	getContactFrame().toSoTransform(tran);
  
	SoCylinder *zaxisCyl = new SoCylinder;
	zaxisCyl->radius = 0.05f;
	zaxisCyl->height = height;
  
	SoTransform *zaxisTran = new SoTransform;
	zaxisTran->translation.setValue(0,0,height/2.0);
	zaxisTran->rotation.setValue(SbVec3f(1,0,0),(float)M_PI/2.0f);
  
	SoSeparator *zaxisSep = new SoSeparator;
	zaxisSep->addChild(zaxisTran);
	zaxisSep->addChild(zaxisMat);
	zaxisSep->addChild(zaxisCyl);
  
	cne->addChild(tran);
	cne->addChild(zaxisSep);
	cne->addChild(coneMat);
	cne->addChild(coords);
	cne->addChild(ifs);
	return cne;
}

//////////////////////////////////////////////////////////
//SoftContact functions
//////////////////////////////////////////////////////////

/*! Does not set up friction edges yet; we need to wait until the mate of
	this contact is also defined because we will need its own analytical
	surface before we can come up with friction edges.
*/
SoftContact::SoftContact( Body *b1, Body *b2, position pos, vec3 norm,
						 Neighborhood *bn ) : Contact(b1, b2, pos, norm)
{ 
	frictionType = SFCL;
	contactDim = 4;
	lmiDim = 4;
	optimalCoeffs = new double[contactDim]; 

	bodyNghbd =  new vec3[ (int) bn->size() ];
	Neighborhood::iterator itr;
	int i = 0;
	vec3 temp;
	
	for( itr = bn->begin(); itr != bn->end(); itr++ ) {
		temp.set(itr->x(), itr->y(), itr->z());
		//places bodyNghbd in frame of contact with the z-axis pointing out
		//bodyNghbd[i] = frame.affine().inverse() * ( temp - frame.translation() );
		position posit;
		posit = position( temp.toSbVec3f() ) * frame.inverse();
		bodyNghbd[i] = vec3( posit.toSbVec3f() );
		i++;
	}
	numPts = (int) bn->size();
	majorAxis = 0.0;
	minorAxis = 0.0;
	relPhi = 0.0;
	a = 0; b = 0; c = 0;
	r1 = 0; r2 = 0;
	r1prime = 0; r2prime = 0;

	FitPoints();

	//setUpFrictionEdges();
	//wait to set these up until other contact is made
}

SoftContact::~SoftContact()
{
	delete[]bodyNghbd;
}

/*! Sets up friction edges as a 3D friction ellipsoid. All the computations for
	fitting analytical surfaces to the two bodies should already have been 
	completed.
*/
int SoftContact::setUpFrictionEdges(bool dynamicsOn )
{
	if( !getMate() ) {
		fprintf(stderr,"Trying to set up friction edges for a contact with no mate!!\n");
		return 1;
	}

	DBGP("Setting up SOFT contact friction edges");
	double eccen[3];

	// magnitude of tangential friction
	eccen[0]=1;
	eccen[1]=1;
	// magnitude of max frictional torque
	double torquedivN;

	// relative radii of curvature at the contact
	CalcRprimes();

	if (!dynamicsOn) {
		// if dynamics are not on, compute it based on some random applied force
		torquedivN = CalcContact_Mattress( 5 );
		eccen[2] = torquedivN*1000;
	} else {
		// if dynamics are on, compute it based on the applied force reported by the LCP
		double nForce = dynamicForce[2];
		//need to figure out what all the numbers in the dynamic force mean
		//I think it is a wrench, and hope it is in the contacts coordinate frame
		//and that the units are in newtons, chekc dynamics.cpp iterateDynamics where
		//it is created
		torquedivN = CalcContact_Mattress( nForce );
		eccen[2] = torquedivN*1000;
	}

	//various possible approximations for the friction ellipsoid

	/*
	int numDirs[9] = {1,3,5,7,8,7,5,3,1};
	double phi[9] = {M_PI_2, M_PI_2*0.6, M_PI_2*0.3, M_PI_2*0.15, 0.0,
                     -M_PI_2*0.15, -M_PI_2*0.3, -M_PI_2*0.6, -M_PI_2};
	return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
	*/

	/*
	int numDirs[9] = {1,8,8,8,8,8,8,8,1};
	double phi[9] = {M_PI_2, M_PI_2*0.66, M_PI_2*0.33, M_PI_2*0.165, 0.0,
				  -M_PI_2*0.165, -M_PI_2*0.33, -M_PI_2*0.66, -M_PI_2};
	return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
	*/
	
	int numDirs[5] = {1,5,8,5,1};
	double phi[5] = {M_PI_2, M_PI_2*0.50, 0.0, -M_PI_2*0.50, -M_PI_2};
	return Contact::setUpFrictionEllipsoid( 5, numDirs, phi, eccen );
}

/*! The Soft Contact version of computeWrenches can also take into
	account the fact that a soft contact can generate some moments in
	the plane of the contacts. It creates wrenches like the regular
	contact, but multiple times, moving the location of the force around
	the area of contact. Currently disabled.
 */
void SoftContact::computeWrenches()
{
	bool approxPlaneMoments = false;

	if (!approxPlaneMoments) {
		Contact::computeWrenches();
		return;
	}

	//for now we hard-code 4 possible locations for the contact
	//on 4 points around the perimeter of the ellipse

	//we also approximate the ellipse with a circle of radius sqrt(majorAxis * minorAxis) 
	//because for now we don't really compute the direction of the major axis

	//when we do, we will have to transform the tangents by:
	// - the transform that aligns it with the major radius of the paraboloid
	// - the transform that further moves that to the major axis of the contact ellipse

	if (wrench) delete [] wrench;
	numFCWrenches = 4 * numFrictionEdges;
	wrench = new Wrench[numFCWrenches];

	vec3 tangentX = frame.affine().row(0);
	vec3 tangentY = frame.affine().row(1);
	
	vec3 radius;
	vec3 baseRadius = loc - ((GraspableBody*)body1)->getCoG();

	float a,b;
	a = b = 1000 * sqrt(majorAxis * minorAxis); //also convert to milimeters
	a = b = 1000 * std::max(majorAxis, minorAxis);

	DBGA("Soft contact size: " << a);

	for (int i=0;i<numFrictionEdges;i++) {
		radius = baseRadius + ( a * tangentX );
		wrenchFromFrictionEdge( &frictionEdges[6*i], radius, &wrench[4*i+0] );

		radius = baseRadius - ( a * tangentX );
		wrenchFromFrictionEdge( &frictionEdges[6*i], radius, &wrench[4*i+1] );

		radius = baseRadius + ( b * tangentY );
		wrenchFromFrictionEdge( &frictionEdges[6*i], radius, &wrench[4*i+2] );

		radius = baseRadius - ( b * tangentY );
		wrenchFromFrictionEdge( &frictionEdges[6*i], radius, &wrench[4*i+3] );
	}
	DBGA("Soft wrenches computed");
}

/*! Computes an analytical surface of the form ax^2 + bx + c in a small patch
	around the contact on body1. The fit is in the local body1 coordinate 
	system.
*/
void SoftContact::FitPoints( )
{
	double *coeffs = new double [3];
	FitParaboloid( bodyNghbd, numPts, coeffs );

	a = coeffs[0];
	b = coeffs[1];
	c = coeffs[2];

	RotateParaboloid( coeffs, &r1, &r2, &fitRot, &fitRotAngle );
	DBGP(getBody1()->getName().latin1() << ": " << "a=" << a << " b=" << b << " c=" <<c);
	DBGP("r1=" << r1 << " r2=" <<r2);
}

/*! Calculates the angle betwen the main radius curvature on body1 and the 
	main radius of curvature on body 2.
*/
int SoftContact::CalcRelPhi( )
{
	vec3 temp, t;
	vec3 R11, R12;		//directions of relative curvatures in world frame

	temp.set( 1, 0 , 0);
	t = fitRot * temp;
	R11 = t * frame;
	R11 = R11 * body1->getTran();

	temp.set( 1, 0 , 0);
	t = ((SoftContact *)getMate())->fitRot * temp;
	R12 = t * ((SoftContact *)getMate())->frame;
	R12 = R12 * body2->getTran();

	relPhi = acos( (R11%R12)/(R11.len()*R12.len()) );
	((SoftContact *)getMate())->relPhi = relPhi ;

	//fprintf( stderr, "Rel Phi   %f", relPhi );
	return 0;	
}

/*! Calculates the relative radii of curvature of the contact, using the
	radii of curvature of both bodies and the angle between them. 
	See Johnson, Contact Mechanics Chapter 4 page 85 for calculations.
*/
int SoftContact::CalcRprimes()
{
	if( !(getMate()) ) {
		DBGA("Contact doesn't have mate, not calculating curvature...");
		return 1;
	}

	SoftContact *m = (SoftContact *)getMate();

	CalcRelPhi();

	DBGP("Body 1" << getBody1()->getName().latin1() << " Body 2 " << m->getBody1()->getName().latin1());

	//the less than zero curvature is for flat objects
	//1/r goes to zero for flat objects because the curvature
	//is infinite
	double x,y,w,z;
	if( r1 < 0.0 )
		w = 0.0;
	else
		w = 1/r1;

	if( r2 < 0.0 )
		x = 0.0;
	else
		x = 1/r2;	

	if( m->r1 < 0.0 )
		y = 0.0;
	else
		y = 1/m->r1;

	if( m->r2 < 0.0 )
		z = 0.0;
	else
		z = 1/m->r2;

	DBGP("x: " << x << " y: " << y << " w: " << w << " z: " << z);

	double ApB = 0.5*( w + x + y + z );

	DBGP("Apb: " << ApB);

	double AmB = 0.5*sqrt( (w - x)*(w - x) + (y - z)*(y - z) +
					2*cos(2*relPhi)*(w - x)*(y - z) );

	DBGP("Amb: " << AmB << " relPhi: " << relPhi);

	if( AmB > ApB ) {
		printf( "Invalid relative curvature, ending calculation...\n" );
		return 1;
	}

	//by definition r1prime >= r2prime
	if (ApB + AmB != 0)
		r1prime = 1/( ApB + AmB );
	else
		r1prime = -1.0;

	if( ApB == AmB)
		r2prime = -1.0;
	else
		r2prime = 1/(ApB - AmB );

	m->r1prime = r1prime;
	m->r2prime = r2prime;

	//fprintf(stderr,"original: %f %f and %f %f\n",r1,r2,m->r1, m->r2);
	//fprintf(stderr,"Relative radii: %f %f equiv: %f\n",r1prime, r2prime, sqrt(r1prime*r2prime) );
	return 0;
}

/*! Calculates the axes of the ellipse of contact using the mattress 
	model and a given normal force (for dynamics its the normal component 
	of the dynamic contact force). Sets major axis and minor axis.
	Returns the max torque available.
	See Contact Mechanics, KL Johnson Chapter 4
*/
double SoftContact::CalcContact_Mattress( double nForce )
{
	if (r1prime < 0) {
		DBGP("Degenerate soft contact");
		r1prime = 20;
	}
	if (r2prime < 0) {
		DBGP("Degenerate soft contact");
		r2prime = 20;
	}
		
	//hardwired height of mattress = 3.0 mm
	//r primes are in mm
	double h = 0.003;
	double delta = sqrt( nForce*h/(MAX(body1->getYoungs(), body2->getYoungs())
									* M_PI * sqrt(r1prime*0.001*r2prime*0.001)) ); //meters

	majorAxis = sqrt( 2*delta*r1prime*0.001 ); //meters
	minorAxis = sqrt( 2*delta*r2prime*0.001 ); 
	DBGP("Axes: " << majorAxis << " " << minorAxis);
	//axes are given in meters!!!!
	//rprimes and the radii of curvature are calculated in mm

	SoftContact *m = (SoftContact *)getMate();
	m->majorAxis = majorAxis;
	m->minorAxis = minorAxis;

	//returns max available torque proportional to given normal force
	//max torque=*K/h*mu*4*pi*(ab)^1.5/15 where a and b are 
	//the axis of the contact ellipse
	//this just returns the max torque divided by the normal force times mu
	//which is 8sqrt(ab)/15
	//the pressure distrib is K*delta/h/2*pi*ab

	//std::cerr<<"Ma ma "<< majorAxis << minorAxis << "\n";

	return 8*sqrt( majorAxis*minorAxis )/15;	//in meters
}

/*! Gets the visual indicator as a small patch of the fit analytical surface
	around the body. Also places a small arrow indicating the direction of
	the main radius of curvature computed for the body.
*/
SoSeparator* SoftContact::getVisualIndicator()
{
	double height,radius;
	SoSeparator *cne;
	SoTransform *tran;
	SoCoordinate3 *coords;
	cne = new SoSeparator;

	int sampling_n = 10, sampling_m = 10;
	//double sampleSize_n = 2, sampleSize_m = 1;
	//double sampleSize_n = (4.0 * majorAxis * 1000) / (2.0 * sampling_n);
	//double sampleSize_m = (4.0 * minorAxis * 1000) / (2.0 * sampling_m);

	double sampleSize_n = 0.7;
	double sampleSize_m = 0.7;


	DBGP("Major " << majorAxis << " minor " << minorAxis);
	
	//points runs left to right and then up to down in the grid
	SbVec3f *points = new SbVec3f[(2*sampling_n + 1)*(2*sampling_m + 1) ];

	coneMat = new SoMaterial;
	coneMat->diffuseColor = SbColor(0.8f,0.0f,0.0f);
	coneMat->ambientColor = SbColor(0.2f,0.0f,0.0f);
	coneMat->emissiveColor = SbColor(0.4f,0.0f,0.0f);
	coneMat->transparency = 0.8f;

	SoMaterial* zaxisMat = new SoMaterial;  
	zaxisMat->diffuseColor = SbColor(0,0,0);
	zaxisMat->ambientColor = SbColor(0,0,0);

	radius = Body::CONE_HEIGHT / 5;
	height = Body::CONE_HEIGHT;

	tran = new SoTransform;  
	getContactFrame().toSoTransform(tran);

	SoCylinder *zaxisCyl = new SoCylinder;
	zaxisCyl->radius = 0.05f;
	zaxisCyl->height = 0.2 * height;
  
	SoTransform *zaxisTran = new SoTransform;
	zaxisTran->translation.setValue(0,0,0.2 * height/2.0);
	zaxisTran->rotation.setValue(SbVec3f(1,0,0),(float)M_PI/2.0f);
  
	SoSeparator *zaxisSep = new SoSeparator;
	zaxisSep->addChild(zaxisTran);
	zaxisSep->addChild(zaxisMat);
	zaxisSep->addChild(zaxisCyl);

	int n_squares = 2 * sampling_n * 2 *sampling_m;
	int32_t *cIndex = new int32_t[5*n_squares];

	int count = 0;
	int count_sq = 0;
	for( int i = -sampling_n; i <= sampling_n; i++ )
	{
		for( int j = -sampling_m; j <= sampling_m; j++ )
		{
			double x = i*sampleSize_n;
			double y = j*sampleSize_m;
			//double z = a*x*x + b*y*y + c*x*y;
			double z = 0;
			if (r1 > 0) z+= x*x*1.0/(2*r1);
			if (r2 > 0) z+= y*y*1.0/(2*r2);
			points[ count ].setValue( x, y, z );
			if (x>0 && y == 0){
				points[count].setValue(x,y,z+0.4);
			}

			if ( i > -sampling_n && j > -sampling_m)
			{
				cIndex[5*count_sq + 0] = count - (2*sampling_m + 1);
				cIndex[5*count_sq + 1] = count - (2*sampling_m + 1) - 1;
				cIndex[5*count_sq + 2] = count - 1;
				cIndex[5*count_sq + 3] = count;
				cIndex[5*count_sq + 4] = -1;
				count_sq++;
			}
			count++;
		}
	}
	if (count != (2*sampling_n + 1)*(2*sampling_m + 1) || count_sq != n_squares)
		exit(0);

	coords = new SoCoordinate3;
	coords->point.setValues( 0, count, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 5*n_squares, cIndex);

	//neighborhood is in frame of contact, so add contact transform first
	cne->addChild(tran);
/*
	SoMaterial* sphereMat = new SoMaterial;  
	sphereMat->diffuseColor = SbColor(1,1,0);
	sphereMat->ambientColor = SbColor(1,1,0);
	cne->addChild(sphereMat);
	for (int i=0; i<numPts; i++)
	{
		SoSphere* sphere = new SoSphere;
		sphere->radius = 0.5;
		SoTransform* sphereTran = new SoTransform;
		sphereTran->translation.setValue( bodyNghbd[i].x(), bodyNghbd[i].y(), bodyNghbd[i].z() );
		SoSeparator* sep = new SoSeparator;
		sep->addChild(sphereTran);
		sep->addChild(sphere);
		cne->addChild(sep);
	}
*/

	cne->addChild(zaxisSep);

	//this angle gets us from the contact frame to the frame in which the paraboloid is defined by just
	//two parameters, r1 and r2
	SoTransform* axisAlignTran = new SoTransform;  
	axisAlignTran->rotation.setValue(SbVec3f(0,0,1), -fitRotAngle);
	cne->addChild(axisAlignTran);

	cne->addChild(coneMat);
	cne->addChild(coords);
	cne->addChild(ifs);

	//add an arrow in the direction of the major axis of the contact ellipse
	//this is the angle between the major axis of the two paraboloids. it is used to compute the
	//magnitude of the major axis of the contact ellipse
	//but for now we don't compute the direction of the contact ellipse!
	SoTransform* phiAlignTran = new SoTransform;  
	phiAlignTran->rotation.setValue(SbVec3f(0,0,1), relPhi);
	cne->addChild(phiAlignTran);

	SoCylinder *xaxisCyl = new SoCylinder;
	xaxisCyl->radius = 0.05f;
	xaxisCyl->height = height / 2.0;
  
	SoTransform *xaxisTran = new SoTransform;
	xaxisTran->translation.setValue(height/4.0, 0.0, 0.0);
	xaxisTran->rotation.setValue(SbVec3f(0,0,1),-(float)M_PI/2.0f);
  
	SoSeparator *xaxisSep = new SoSeparator;
	xaxisSep->addChild(xaxisTran);
	xaxisSep->addChild(zaxisMat);
	xaxisSep->addChild(xaxisCyl);

	//cne->addChild(xaxisSep);
	return cne;
}

///////////////////////////////////////////
// Virtual Contact
///////////////////////////////////////////

/*! Initializes an empty (and for now unusable) contact. Sets it
	as its own mate, which is the distinctive sign of a virtual
	contact.
*/
void
VirtualContact::init()
{
	mFingerNum = -2;
	mLinkNum = 0;
	mZaxisMat = NULL;
	mWorldInd = NULL;
	mate = this;
	body1 = NULL;
	body2 = NULL;
}

/*! Just calls the super and then calls init to give default values 
	to the virtual contact specific parameters.
*/
VirtualContact::VirtualContact() : Contact()
{
	init();
}

/*! Copies the location, friction edges, and coefficients of the \a original.
	This should really use a super's copy constructor, but we never wrote
	one. Does not copy the wrenches; the new contact needs to build them
	itself from the friction edges.
*/
VirtualContact::VirtualContact(const VirtualContact *original) : Contact()
{
	init();
	mFingerNum = original->mFingerNum;
	mLinkNum = original->mLinkNum;
	//copy friction edges
	//this should really be done by a Contact copy constructor
	numFrictionEdges = original->numFrictionEdges;
	memcpy( frictionEdges, original->frictionEdges, 6 * numFrictionEdges * sizeof(double) );
	//wrenches are not copied; this contact should build them itself
	loc = original->loc;
	frame = original->frame;
	normal = original->normal;
	cof = original->cof;	
	body1 = original->body1;
	body2 = original->body2;
}

/*! When copying from a regular contact, we must be careful because the	
	normal of a virtual contact points outwards, whereas the normal of a 
	trasitional contact points inwards.
*/
VirtualContact::VirtualContact(int f, int l, Contact *original) : Contact()
{
	init();
	mFingerNum = f;
	mLinkNum = l;

	numFrictionEdges = original->numFrictionEdges;
	memcpy( frictionEdges, original->frictionEdges, 6 * numFrictionEdges * sizeof(double) );
	cof = original->cof;

	loc = original->loc;
	frame = original->frame;
	//we now rotate the frame so that the normal points outwards, as the contact on the object would
	Quaternion q(3.14159, vec3(1,0,0));
	transf newRot(q, vec3(0,0,0) );
	frame = newRot * frame;
	normal = -1 * original->normal;
	body1 = original->body1;
	body2 = original->body2;
}

/*! Just sets the mate to NULL and let the super destructor 
	take care of the rest.
*/
VirtualContact::~VirtualContact()
{
	mate = NULL;
}

position
VirtualContact::getWorldLocation()
{
	return loc * body1->getTran();
}

vec3
VirtualContact::getWorldNormal()
{
	return normal * body1->getTran();
}

/*! The virtual contact, for now, does not compute its own friction
	edges, but just inherits them from the regular contact that it 
	starts from. Alternatively, if it loaded from a file, it reads
	them in from the file.
	
	In the future, this hierarchy needs to be better engineered.
*/
int
VirtualContact::setUpFrictionEdges(bool dynamicsOn)
{
	dynamicsOn = dynamicsOn;
	return 1;
}

/*!	This computes the wrenches of the virtual contact as used for grasp 
	quality computations. The important aspect to take into account is 
	wether we are using an object or not.
	
	If we are using an object, we assume that contact centroid and maxradius 
	have already been set to match those of the	object (hopefully, the grasp 
	has done this). Also, the force applied be the contact is scaled by a 
	function of the distance between the contact and the actual object.
	
	If there is no object, we assume that contact centroid and maxradius 
	have been preset dependign only on the set of virtual contacts that 
	make up the grasp (again, we hope the grasp has done this). There is no 
	scaling involved, all forces are handled normally as if the contact was 
	actually on the object.
*/
void
VirtualContact::computeWrenches(bool useObjectData, bool simplify)
{
	int i;
	//double alpha;
	vec3 radius, forceVec,torqueVec,tangentX,tangentY;

	//we need everything to be wrt world coordinates
	position worldLoc;
	vec3 worldNormal;

	if (!useObjectData) {	
		worldLoc = getWorldLocation();
		worldNormal = getWorldNormal();
		transf worldFrame = frame * body1->getTran();
		tangentX = worldFrame.affine().row(0);
		tangentY = worldFrame.affine().row(1);
	} else {
//		LOOK AT VIRTUAL CONTACT ON THE HAND
		worldLoc = getWorldLocation();
		worldNormal = getWorldNormal();
		tangentX = vec3(0,1,0) * worldNormal;
		tangentX = normalise(tangentX);
		tangentY = worldNormal * tangentX;
	}

	//mCenter needs to be already set to object cog if needed as such
	radius = worldLoc - mCenter;
	if (wrench) delete [] wrench;

	if (simplify) {
		numFCWrenches = 1; //this is hack-ish, should be fixed
		wrench = new Wrench[1];
		wrench[0].force = worldNormal;
		wrench[0].torque = (radius*worldNormal) / mMaxRadius;
		return;
	}

	numFCWrenches = numFrictionEdges;
	wrench = new Wrench[numFrictionEdges];

	//SHOULD SET UP COEFFICIENT BASED ON MATERIALS!
	for (i=0;i<numFCWrenches;i++) {
		forceVec = worldNormal + (tangentX*cof*frictionEdges[6*i])+ (tangentY*cof*frictionEdges[6*i+1]);
		//max friction is normal_force_magnitude (which is 1) * coeff_of_friction 
		//possible friction for this wrench is (friction_edge * max_friction) in the X and Y direction of the contact

		wrench[i].force = forceVec;

		wrench[i].torque = ( (radius*forceVec) + worldNormal*cof*frictionEdges[6*i+5] )/mMaxRadius;
		//max torque is contact_radius * normal_force_magnitude (which is 1) * coeff_of_friction
		//possible torque for this wrench is (friction_edge * max_torque) in the direction of the contact normal
		//the contact_radius coefficient is already taken into account in the friction_edge
	}
}

/*!	Scales the force vector based on how distant the VirtualContact 
	is from the actual object. The intuition is as follows: we do not 
	want Grasp Quality to be a step function depending wether a contact 
	is on of off the surface of the object. Ratherm we want it smooth so 
	we allow a VirtualContact to contribute to GQ even if it's a bit off the
	surface of the object.
*/
void
VirtualContact::scaleWrenches(double factor)
{
	int i;
	for (i=0; i<numFCWrenches; i++) {
		wrench[i].force = factor * wrench[i].force;
		wrench[i].torque = factor * wrench[i].torque;
	}
}

/*!	Gives us a visual indicator of what this contact looks like, 
	in WORLD COORDINATES. Since this contact has to be transformed 
	to world coordinates for the sake of grasp analysis, this allows 
	us to be sure that the transformation makes sense.
	
	It assumes WRENCHES have been computed.
*/
void
VirtualContact::getWorldIndicator(bool useObjectData)
{
	vec3 forceVec;
	position worldLoc;

	if (!useObjectData) {
		worldLoc = getWorldLocation();
	} else {
		vec3 objDist;
		getObjectDistanceAndNormal(body2, &objDist, NULL);
		worldLoc = getWorldLocation() + objDist;
	}

	SoTransform* tran = new SoTransform;  
	SbMatrix tr;
	tr.setTranslate( worldLoc.toSbVec3f() );
	tran->setMatrix( tr );
  
	SbVec3f *points = (SbVec3f*)calloc(numFCWrenches+1, sizeof(SbVec3f) );
	int32_t *cIndex = (int32_t*)calloc(4*numFCWrenches, sizeof(int32_t) );

	points[0].setValue(0,0,0);

	for (int i=0;i<numFCWrenches;i++) {
		//if ( wrench[i].torque.len() != 0 ) continue;
		forceVec = wrench[i].force;
		forceVec = Body::CONE_HEIGHT * forceVec;
		points[i+1].setValue( forceVec.x(), forceVec.y(), forceVec.z() );
		cIndex[4*i] = 0;
		cIndex[4*i+1] = i + 2;
		if ( i == numFCWrenches-1) cIndex[4*i+1] = 1;
		cIndex[4*i+2] = i + 1;
		cIndex[4*i+3] = -1;
	}

	SoCoordinate3* coords = new SoCoordinate3;
	SoIndexedFaceSet* ifs = new SoIndexedFaceSet;
	coords->point.setValues(0,numFCWrenches+1,points);
	ifs->coordIndex.setValues(0,4*numFCWrenches,cIndex);
	free(points);
	free(cIndex);

	SoMaterial *coneMat = new SoMaterial;  
    coneMat->diffuseColor = SbColor(0.0f,0.0f,0.8f);
    coneMat->ambientColor = SbColor(0.0f,0.0f,0.2f);
    coneMat->emissiveColor = SbColor(0.0f,0.0f,0.4f);

	if (mWorldInd) {
		body1->getWorld()->getIVRoot()->removeChild(mWorldInd);
	}

	mWorldInd = new SoSeparator;
	mWorldInd->addChild(tran);
	mWorldInd->addChild(coneMat);
	mWorldInd->addChild(coords);
	mWorldInd->addChild(ifs);
	body1->getWorld()->getIVRoot()->addChild(mWorldInd);

	/*
	SoSeparator* cSep = new SoSeparator;
	tr.setTranslate( mCenter.toSbVec3f() );
	tran = new SoTransform;
	tran->setMatrix( tr );
	cSep->addChild(tran);
	SoSphere* cSphere = new SoSphere();
	cSphere->radius = 5;
	cSep->addChild(cSphere);
	body1->getWorld()->getIVRoot()->addChild(cSep);
	*/
}

SoSeparator* 
VirtualContact::getVisualIndicator()
{
	SoTransform *tran;

	//this one is a member variable so we can change color if we want to "mark" the contact
	if (!mZaxisMat) {
		mZaxisMat = new SoMaterial;
		mZaxisMat->ref();
	}
	mZaxisMat->diffuseColor = SbColor(0.8f,0,0);
	mZaxisMat->ambientColor = SbColor(0.8f,0,0);

	tran = new SoTransform;  
	getContactFrame().toSoTransform(tran);
  
	SoCylinder *zaxisCyl = new SoCylinder;
	zaxisCyl->radius = 0.2f;
	zaxisCyl->height = Body::CONE_HEIGHT;

	SoSphere *zaxisSphere = new SoSphere;
	zaxisSphere->radius = 3.0f;
  
	SoTransform *zaxisTran = new SoTransform;
	zaxisTran->translation.setValue(0,0,Body::CONE_HEIGHT/2.0);
//	zaxisTran->translation.setValue(0,0,2.0);
	zaxisTran->rotation.setValue(SbVec3f(1,0,0),(float)M_PI/2.0f);
  
	SoSeparator *zaxisSep = new SoSeparator;
	zaxisSep->addChild(zaxisTran);
	zaxisSep->addChild(mZaxisMat);
	zaxisSep->addChild(zaxisCyl);
//	zaxisSep->addChild(zaxisSphere);

	SoSeparator* cne = new SoSeparator;
	cne->addChild(tran);
	cne->addChild(zaxisSep);
	return cne;
}

/*! Changes the color of the visual indicator from red to blue
	if this contact is to be "marked".
*/
void
VirtualContact::mark(bool m)
{
	if (!mZaxisMat) return;
	if (m) {
		mZaxisMat->diffuseColor = SbColor(0,0,0.8f);
		mZaxisMat->ambientColor = SbColor(0,0,0.8f);
	} else {
		mZaxisMat->diffuseColor = SbColor(0.8f,0,0);
		mZaxisMat->ambientColor = SbColor(0.8f,0,0);
	}
}

/*! Saves all the info needed for this contact (what body and link 
	it's on, location, normal, friction edges and coefficient) to
	a file.
*/
void
VirtualContact::writeToFile(FILE *fp)
{
	//finger and link number
	fprintf(fp,"%d %d\n",mFingerNum, mLinkNum);

	//numFrictionEdges
	fprintf(fp,"%d\n",numFrictionEdges);

	//frictionEdges
	for (int i=0; i<numFrictionEdges; i++) {
		for (int j=0; j<6; j++)
			fprintf(fp,"%f ",frictionEdges[6*i+j]);
		fprintf(fp,"\n");
	}

	//loc
	fprintf(fp,"%f %f %f\n",loc.x(), loc.y(), loc.z());

	//frame
	Quaternion q = frame.rotation();
	vec3 t = frame.translation();
	fprintf(fp,"%f %f %f %f\n",q.w,q.x,q.y,q.z);
	fprintf(fp,"%f %f %f\n",t.x(), t.y(), t.z());

	//normal
	fprintf(fp,"%f %f %f\n",normal.x(), normal.y(), normal.z());

	//cof
	fprintf(fp,"%f\n",cof);
}

/*! Loads all the info for this contact from a file previously written
	by VirtualContact::writeToFile(...)
*/
void
VirtualContact::readFromFile(FILE *fp)
{
	float v,x,y,z;

	//finger and link number
	if ( fscanf(fp,"%d %d",&mFingerNum, &mLinkNum) <= 0){
	  DBGA("VirtualContact::readFromFile - Failed to read fingernumber or link number");
	  return;
	}
	//numFrictionEdges
	if (fscanf(fp,"%d",&numFrictionEdges) <= 0){
	    DBGA("VirtualContact::readFromFile - Failed to read number of virtual contacts");
	    return;
	  }

	//frictionEdges
	for (int i=0; i<numFrictionEdges; i++) {
		for (int j=0; j<6; j++) {
		  if(fscanf(fp,"%f",&v) <= 0){
		    DBGA("VirtualContact::readFromFile - Failed to read number of friction edges");
		    return;
		  };
			frictionEdges[6*i+j] = v;
		}
	}

	//loc
	if(fscanf(fp,"%f %f %f",&x, &y, &z) <= 0){
	 DBGA("VirtualContact::readFromFile - Failed to read virtual contact location");
	 return;
	}
	loc = position(x,y,z);

	//frame
	Quaternion q;
	vec3 t;
	if(fscanf(fp,"%f %f %f %f",&v,&x,&y,&z) <= 0) {
	  DBGA("VirtualContact::readFromFile - Failed to read virtual contact frame orientation");
	}
	q.set(v,x,y,z);
	if(fscanf(fp,"%f %f %f",&x, &y, &z) <= 0) {
	 DBGA("VirtualContact::readFromFile - Failed to read virtual contact frame location");
	} 

	t.set(x,y,z);
	frame.set(q,t);

	//normal
	if( fscanf(fp,"%f %f %f",&x, &y, &z) <= 0){
	 DBGA("VirtualContact::readFromFile - Failed to read virtual contact normal");
	 return;
	}
	normal.set(x,y,z);

	//cof
	if( fscanf(fp,"%f",&v) <= 0){ 
	DBGA("VirtualContact::readFromFile - Failed to read virtual contact friction");
	return;
	}
	cof = v;
}

/*! Sets objDistance to be the vector from the contact to the closest
	point on the given body. Sets objNormal to be the object surface normal
	at that point.
*/
void
VirtualContact::getObjectDistanceAndNormal(Body *body, vec3 *objDistance, vec3 *objNormal)
{
	position loc = getWorldLocation();
	*objDistance = body->getWorld()->pointDistanceToBody(loc, body, objNormal);
}


VirtualContactOnObject::VirtualContactOnObject()
{
	wrench = NULL;
	body2 = NULL;
	mate = this;
	prevBetas = NULL;
}

VirtualContactOnObject::~VirtualContactOnObject()
{
}

void
VirtualContactOnObject::readFromFile(FILE *fp)
{
	float w,x,y,z;

	//numFCVectors
	if(fscanf(fp,"%d",&numFrictionEdges) <= 0) {
	  DBGA("VirtualContactOnObject::readFromFile - Failed to read number of friction vectors");
	  return; 
	}

	//frictionEdges
	for (int i=0; i<numFrictionEdges; i++) {
		for (int j=0; j<6; j++) {
		  if (fscanf(fp,"%f",&w) <= 0) {
		    DBGA("VirtualContactOnObject::readFromFile - Failed to read number of friction edges");
		    return; 
		  }
		    
			frictionEdges[6*i+j] = w;
		}
	}
	fprintf(stderr,"\n<frictionEdges scanned successfully>"); // for test

	// (w,x,y,z) is already a quaternion, if you want to do frame rotate v rad along a vector (x,y,z),
	//you can use q(v,vec(x,y,z))
	Quaternion q;
	vec3 t;
	if(fscanf(fp,"%f %f %f %f",&w,&x,&y,&z) <= 0) {
	  DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact location");
	  return;
	}
	

	q.set(w,x,y,z);
	if(fscanf(fp,"%f %f %f",&x, &y, &z) <= 0) {
	  DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact orientation");
	  return;
	}
	
	t.set(x,y,z);
	loc = position(x,y,z);
	frame.set(q,t);

	//normal
	if(fscanf(fp,"%f %f %f",&x, &y, &z) <= 0) {
	  DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact normal");
	  return;
	}

	normal.set(x,y,z);

	//cof
	if(fscanf(fp,"%f",&w) <= 0) {
	  DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact normal");
	  return;
	}
	cof = w;
}

#ifdef ARIZONA_PROJECT_ENABLED
void
VirtualContactOnObject::readFromRawData(ArizonaRawExp* are, QString file, int index, bool flipNormal)
{
	
	float v;
	FILE *fp = fopen(file.latin1(), "r");
	if (!fp) {
		fprintf(stderr,"Could not open filename %s\n",file.latin1());
		return;
	}

	//numFCVectors
	if(fscanf(fp,"%d",&numFrictionEdges) <= 0) {
	  DBGA("VirtualContactOnObject::readFromRawData - Failed to read virtual contact orientation");
	  return; 
	}

	//frictionEdges
	for (int i=0; i<numFrictionEdges; i++) {
		for (int j=0; j<6; j++) {
		  if(fscanf(fp,"%f",&v) <= 0) 
		    {
		      DBGA("VirtualContactOnObject::readFromRawData - Failed to read number of friction edges for virtual contacts");
		      return; 
		    }
		  frictionEdges[6*i+j] = v;
		}
	}

	Quaternion q;
	vec3 t;
	q = are->getQuaternion(index);
	t = are->getContact(index);
	loc = position(t.x(),t.y(),t.z());
	frame.set(q,t);

	//normal
	normal = are->getNormal(index);
	if(flipNormal){
		std::cout << "flipped normal" << std::endl;
		normal = - normal;
	}

	//cof, need further consideration
	cof = 0.5;

	fclose(fp);
	
}
#endif

void
VirtualContactOnObject::writeToFile(FILE *fp){
	//numFrictionEdges
	fprintf(fp,"%d\n",numFrictionEdges);

	//frictionEdges
	for (int i=0; i<numFrictionEdges; i++) {
		for (int j=0; j<6; j++)
			fprintf(fp,"%f ",frictionEdges[6*i+j]);
		fprintf(fp,"\n");
	}

	//frame
	Quaternion q = frame.rotation();
	vec3 t = frame.translation();
	fprintf(fp,"%f %f %f %f\n",q.w,q.x,q.y,q.z);
	fprintf(fp,"%f %f %f\n",t.x(), t.y(), t.z());

	//normal
	fprintf(fp,"%f %f %f\n",normal.x(), normal.y(), normal.z());

	//cof
	fprintf(fp,"%f\n",cof);
}
