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

#include "graspit/matvec3D.h"
#include "graspit/contact/contact.h"
#include "graspit/world.h"
#include "graspit/body.h"
#include "graspit/mytools.h"
#include "graspit/math/matrix.h"

//#define GRASPITDBG
#include "graspit/debug.h"

const double Contact::THRESHOLD = 0.2;
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
  normal = norm.normalized();
  loc = pos;
  vec3 tangentX, tangentY;

  if (fabs(normal.dot(vec3(1, 0, 0))) > 1.0 - MACHINE_ZERO) {
    tangentX = normal.cross(vec3(0, 0, 1)).normalized();
  } else {
    tangentX = normal.cross(vec3(1, 0, 0)).normalized();
  }
  tangentY = (normal.cross(tangentX)).normalized();
  frame = transf::COORDINATE(loc, tangentX, tangentY);
  coneMat = NULL;
  prevBetas = NULL;
  inheritanceInfo = false;
  for (int i = 0; i < 6; i++) {
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
  if (optimalCoeffs) { delete [] optimalCoeffs; }
  if (wrench) { delete [] wrench; }
  if (prevBetas) { delete [] prevBetas; }

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
  vec3 tangentX = frame.affine().col(0);
  vec3 tangentY = frame.affine().col(1);

  GraspableBody *object = (GraspableBody *)body1;

  //max friction is normal_force_magnitude (which is 1) * coeff_of_friction
  //possible friction for this wrench is (friction_edge * max_friction) in the X and Y direction of the contact
  vec3 forceVec = normal + (tangentX * sCof * edge[0]) + (tangentY * sCof * edge[1]);

  //max torque is contact_radius * normal_force_magnitude (which is 1) * coeff_of_friction
  //possible torque for this wrench is (friction_edge * max_torque) in the direction of the contact normal
  //the contact_radius coefficient is already taken into account in the friction_edge
  vec3 torqueVec = ((radius.cross(forceVec)) + normal * sCof * edge[5]) / object->getMaxRadius();

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

  if (wrench) { delete [] wrench; }
  //one wrench for each vector
  numFCWrenches = numFrictionEdges;
  wrench = new Wrench[numFCWrenches];

  vec3 radius = loc - ((GraspableBody *)body1)->getCoG();
  for (int i = 0; i < numFCWrenches; i++) {
    wrenchFromFrictionEdge(&frictionEdges[6 * i], radius, &wrench[i]);
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
  for (int i = 0; i < numLatitudes; i++) {
    numFrictionEdges += numDirs[i];
  }
  if (numFrictionEdges > MAX_FRICTION_EDGES) { return FAILURE; }
  prevBetas = new double[numFrictionEdges];

  int col = 0;
  for (int i = 0; i < numLatitudes; i++) {
    double cosphi = cos(phi[i]);
    double sinphi = sin(phi[i]);
    for (int j = 0; j < numDirs[i]; j++) {
      double theta = j * 2 * M_PI / numDirs[i];

      double num = cos(theta) * cosphi;
      double denom = num * num / (eccen[0] * eccen[0]);
      num = sin(theta) * cosphi;
      denom += num * num / (eccen[1] * eccen[1]);
      num = sinphi;
      denom += num * num / (eccen[2] * eccen[2]);
      denom = sqrt(denom);

      frictionEdges[col * 6]   = cos(theta) * cosphi / denom;
      frictionEdges[col * 6 + 1] = sin(theta) * cosphi / denom;
      frictionEdges[col * 6 + 2] = 0;
      frictionEdges[col * 6 + 3] = 0;
      frictionEdges[col * 6 + 4] = 0;
      frictionEdges[col * 6 + 5] = sinphi / denom;
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
  Matrix F(1, 1 + numFrictionEdges);
  F.setAllElements(1.0);
  F.elem(0, 0) = -1.0 * getCof();
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
  Rfi.elem(0, 0) = Rfi.elem(1, 0) = 0.0; Rfi.elem(2, 0) = 1.0;
  Rfi.elem(3, 0) = Rfi.elem(4, 0) = Rfi.elem(5, 0) = 0.0;
  //the columns for the friction edges
  for (int edge = 0; edge < numFrictionEdges; edge++) {
    for (int i = 0; i < 6; i++) {
      Rfi.elem(i, edge + 1) = frictionEdges[6 * edge + i];
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
Contact::frictionForceBlockMatrix(const std::list<Contact *> &contacts)
{
  if (contacts.empty()) {
    return Matrix(0, 0);
  }
  std::list<Matrix *> blocks;
  std::list<Contact *>::const_iterator it;
  for (it = contacts.begin(); it != contacts.end(); it++) {
    blocks.push_back(new Matrix((*it)->frictionForceMatrix()));
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
Contact::frictionConstraintsBlockMatrix(const std::list<Contact *> &contacts)
{
  if (contacts.empty()) {
    return Matrix(0, 0);
  }
  std::list<Matrix *> blocks;
  std::list<Contact *>::const_iterator it;
  for (it = contacts.begin(); it != contacts.end(); it++) {
    blocks.push_back(new Matrix((*it)->frictionConstraintsMatrix()));
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
Contact::normalForceSumMatrix(const std::list<Contact *> &contacts)
{
  if (contacts.empty()) {
    return Matrix(0, 0);
  }
  std::list<Matrix *> blocks;
  std::list<Contact *>::const_iterator it;
  for (it = contacts.begin(); it != contacts.end(); it++) {
    blocks.push_back(new Matrix(Matrix::ZEROES<Matrix>(1, (*it)->numFrictionEdges + 1)));
    blocks.back()->elem(0, 0) = 1.0;
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
  Matrix Ro(Matrix::ZEROES<Matrix>(6, 6));
  transf contactTran = getBody1()->getTran() % getContactFrame();
  Matrix Rot(Matrix::ROTATION(contactTran.affine()).transposed());
  //the force transform is simple, just the matrix that changes coord. systems
  Ro.copySubMatrix(0, 0, Rot);
  Ro.copySubMatrix(3, 3, Rot);
  //for torque we also multiply by a cross product matrix
  vec3 worldLocation = contactTran.translation();
  vec3 cog = getBody2()->getTran().translation();
  mat3 C; setCrossProductMatrix(C, worldLocation - cog);
  Matrix CR(3, 3);
  matrixMultiply(Matrix::ROTATION(C.transpose()), Rot, CR);
  //also scale by object max radius so we get same units as force
  //and optimization does not favor torque over force
  double scale = 1.0;
  if (getBody2()->isA("GraspableBody")) {
    scale = scale / static_cast<GraspableBody *>(getBody2())->getMaxRadius();
  }
  CR.multiply(scale);
  Ro.copySubMatrix(3, 0, CR);
  return Ro;
}

/*! Assembles together the localToWorldWrenchMatrix for all the contacts in the list
  in block diagonal form.
*/
Matrix
Contact::localToWorldWrenchBlockMatrix(const std::list<Contact *> &contacts)
{
  if (contacts.empty()) {
    return Matrix(0, 0);
  }
  std::list<Matrix *> blocks;
  std::list<Contact *>::const_iterator it;
  for (it = contacts.begin(); it != contacts.end(); it++) {
    blocks.push_back(new Matrix((*it)->localToWorldWrenchMatrix()));
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
bool Contact::preventsMotion(const transf &motion) const
{
  bool result =  ((motion * loc) - loc).dot(normal) < -MACHINE_ZERO;
  return result;
}


/*!
  Recomputes the COF for this contact.  This is called when the material of
  one of the two bodies is changed.
*/
void
Contact::updateCof()
{
  sCof = body1->getWorld()->getCOF(body1->getMaterial(), body2->getMaterial());
  kcof = body1->getWorld()->getKCOF(body1->getMaterial(), body2->getMaterial());
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
  vec3 radius, vel1(vec3::Zero()), vel2(vec3::Zero()), rotvel;

  if (body1->isDynamic()) {
    db = (DynamicBody *)body1;
    radius = db->getTran().rotation() * (loc - db->getCoG());
    vel1.x() = db->getVelocity()[0];
    vel1.y() = db->getVelocity()[1];
    vel1.z() = db->getVelocity()[2];

    rotvel.x() = db->getVelocity()[3];
    rotvel.y() = db->getVelocity()[4];
    rotvel.z() = db->getVelocity()[5];

    vel1 += radius.cross(rotvel);
  }
  if (body2->isDynamic()) {
    db = (DynamicBody *)body2;
    radius = db->getTran().rotation() * (mate->loc - db->getCoG());
    vel2.x() = db->getVelocity()[0];
    vel2.y() = db->getVelocity()[1];
    vel2.z() = db->getVelocity()[2];

    rotvel.x() = db->getVelocity()[3];
    rotvel.y() = db->getVelocity()[4];
    rotvel.z() = db->getVelocity()[5];
    vel2 += radius.cross(rotvel);
  }
  if ((vel1 - vel2).norm() > 1.0) {
    DBGP("SLIDING!");
    return kcof;
  }
  else { return sCof; }
}

/*!
  When the grasp force optimization completes it calls this routine to set
  this contact's optimal force.  This force is a compromise between minimizing
  internal grasp forces and how close the force is to the boundary of the
  friction cone, or starting to slip.
*/
void
Contact::setContactForce(double *optmx)
{
  for (int i = 0; i < contactDim; i++) {
    optimalCoeffs[i] = optmx[i];
  }
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
  transf cf = body1->getTran() % frame;
  transf cf2 = body2->getTran() % mate->getContactFrame();
  return MAX(0.0, Contact::THRESHOLD / 2.0 - (cf.translation() - cf2.translation()).norm());
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
  setLCPInfo(c->getPrevCn(), c->getPrevLambda(), c->getPrevBetas());
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
  for (int i = 0; i < numFrictionEdges; i++) {
    prevBetas[i] = betas[i];
  }
  //lambda is the LCP paramter that indicates contact slip
  if (l > 0) { mSlip = true; }
  else { mSlip = false; }
}


