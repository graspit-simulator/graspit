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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: dynJoint.cpp,v 1.9 2009/05/26 20:05:32 cmatei Exp $
//
//######################################################################

/*! \file
\brief Implements the DynJoint classes.
*/

#include <math.h>

#include "graspit/dynamics/dynJoint.h"
#include "graspit/joint.h"
#include "graspit/body.h"
#include "graspit/math/matrix.h"

//#define GRASPITDBG
#include "graspit/debug.h"

/*!
  Fills in the constraint columns in the joint constraint matrix \a Nu .  It also
  computes the generalized position error between the two links or between the
  one link and the world coordinates it should be fixed to.  The errors are
  copied into the corresponding elements of the \a eps vector.  Nu^T * v = eps.

  \a numBodies are the number of bodies in this dynamic island, and \a islandIndices
  is a vector that maps a body to the index of that body within the vector
  of bodies for this dynamic island.  This allows us to compute the starting
  row numbers for the links connected to this joint. \a ncn is a counter
  that holds the current constraint number, thus the current column to use.
*/
void
DynJoint::buildConstraints(double *Nu, double *eps, int numBodies,
                           std::map<Body *, int> &islandIndices, int &ncn)
{
  //compute the joint coordinate system as seen by both the previous and the
  //next links
  transf prevTrans = prevLink->getTran() %  getPrevTrans();
  transf nextTrans = nextLink->getTran() % getNextTrans();

  //compute error
  transf error = prevTrans % nextTrans.inverse();

  //we will use the joint coordinate system as reported by the previous link
  //as reference
  vec3 constrainedAxis[3];
  constrainedAxis[0] = prevTrans.affine().col(0);
  constrainedAxis[1] = prevTrans.affine().col(1);
  constrainedAxis[2] = prevTrans.affine().col(2);

  DBGP("Constrained axes:\n  " << constrainedAxis[0] <<
       "\n  " << constrainedAxis[1] << "\n  " << constrainedAxis[2]);

  //set up cross product matrices for translations from joint location
  //the the cog's of the links
  vec3 prevCOG = prevLink->getTran() * (prevLink->getCoG()) - position::Zero();
  vec3 nextCOG = nextLink->getTran() * (nextLink->getCoG()) - position::Zero();

  //the constrained axes transformed for each of the links involved
  //remember that prevTrans is considered the reference transform for this dyn joint
  mat3 prevCross;
  setCrossProductMatrix(prevCross, prevTrans.translation() - prevCOG);
  prevCross *= prevTrans.affine().transpose();

  mat3 nextCross;
  setCrossProductMatrix(nextCross, nextTrans.translation() - nextCOG);
  nextCross *= prevTrans.affine().transpose();

  //find out which directions we are actually constraining
  char constraints[6];
  getConstraints(constraints);

  //compute the indices into the big constraint matrix
  assert(islandIndices[prevLink] >= 0);
  assert(islandIndices[nextLink] >= 0);
  int prevLinkRow = 6 * islandIndices[prevLink];
  int nextLinkRow = 6 * islandIndices[nextLink];

  //compute translation error in world coordinate system
  vec3 errorVec = nextTrans.translation() - prevTrans.translation();
  DBGP("Translation error: " << errorVec);

  //constrain translations
  for (int c = 0; c < 3; c++) {
    if (!constraints[c]) { continue; }

    Nu[(ncn) * 6 * numBodies + prevLinkRow]   -= constrainedAxis[c](0);
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 1] -= constrainedAxis[c](1);
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 2] -= constrainedAxis[c](2);
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 3] -= prevCross(3 * c);
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 4] -= prevCross(3 * c + 1);
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 5] -= prevCross(3 * c + 2);

    Nu[(ncn) * 6 * numBodies + nextLinkRow]   += constrainedAxis[c](0);
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 1] += constrainedAxis[c](1);
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 2] += constrainedAxis[c](2);
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 3] += nextCross(3 * c);
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 4] += nextCross(3 * c + 1);
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 5] += nextCross(3 * c + 2);

    eps[ncn] = -(errorVec.dot(constrainedAxis[c]));
    ncn++;

  }

  // constrain rotations
  double alpha;

  Eigen::AngleAxisd aa(error.rotation().normalized());
  alpha = aa.angle();
  errorVec = aa.axis();

  errorVec = - errorVec * alpha;
  DBGP("Rotation error: " << errorVec);

  for (int c = 0; c < 3; c++) {
    if (!constraints[3 + c]) { continue; }

    Nu[(ncn) * 6 * numBodies + prevLinkRow + 3] -= constrainedAxis[c][0];
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 4] -= constrainedAxis[c][1];
    Nu[(ncn) * 6 * numBodies + prevLinkRow + 5] -= constrainedAxis[c][2];

    Nu[(ncn) * 6 * numBodies + nextLinkRow + 3] += constrainedAxis[c][0];
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 4] += constrainedAxis[c][1];
    Nu[(ncn) * 6 * numBodies + nextLinkRow + 5] += constrainedAxis[c][2];

    eps[ncn] = -(errorVec.dot(constrainedAxis[c]));
    ncn++;
  }
}

/*! Given a point that has world transform \a toTarget, this computes the 6x6 Jacobian
    of this joint relative to that point. The Jacobian is either expressed in global
  world coordinates or in the local coordinates of the target.
*/
void
DynJoint::jacobian(transf toTarget, Matrix *J, bool worldCoords)
{
  transf myTran = getPrevLink()->getTran() % getPrevTrans();
  transf T;
  if (worldCoords) {
    // the translation from joint coordinate system to world coordinate system
    T = myTran.inverse() % transf(Quaternion::Identity(), toTarget.translation());
  } else {
    T = myTran.inverse() % toTarget;
  }
  double M[36];
  T.jacobian(M);
  J->copyMatrix(Matrix(M, 6, 6, true));
}

/*! With a fixed joint, there are no unconstrained DOF values to update.
    This is a stub.
*/
void
FixedDynJoint::updateValues()
{
}

/*! If there is no previous link there is a simpler way for enforcing constraints
  by just working in world coordinate system without bothering with joint
  coordinate system.
*/
void
FixedDynJoint::buildConstraints(double *Nu, double *eps, int numBodies,
                                std::map<Body *, int> &islandIndices, int &ncn)
{
  if (prevLink) {
    DynJoint::buildConstraints(Nu, eps, numBodies, islandIndices, ncn);
    return;
  }

  assert(islandIndices[nextLink] >= 0);
  int nextLinkRow = 6 * islandIndices[nextLink];

  transf prevTrans = getPrevTrans();
  transf nextTrans = nextLink->getTran() % getNextTrans();
  transf error = prevTrans % nextTrans.inverse();

  vec3 errorVec = nextTrans.translation() - prevTrans.translation();

  for (int c = 0; c < 3; c++) {
    Nu[(ncn) * 6 * numBodies + nextLinkRow + c]   += 1.0;
    eps[ncn] = -errorVec[c];
    ncn++;
  }

  double alpha;
  Eigen::AngleAxisd aa(error.rotation().normalized());
  alpha = aa.angle();
  errorVec = aa.axis();
  errorVec = - errorVec * alpha;
  DBGP("Rotation error: " << errorVec);

  for (int c = 3; c < 6; c++) {
    Nu[(ncn) * 6 * numBodies + nextLinkRow + c] += 1.0;
    eps[ncn] = 2 * errorVec[c - 3];
    ncn++;
  }
}

/*!
  After a timestep has been completed, this computes the current joint angle
  and velocity from the relative positions and velocities of the connected
  links.
*/
void
RevoluteDynJoint::updateValues()
{
  transf b1JointTran = prevLink->getTran() % prevFrame;

  //the z axis of the previous link, by definition the axis of the one joint
  vec3 axis = b1JointTran.affine().col(2);
  joint->setWorldAxis(axis);
  double vel1 = vec3(prevLink->getVelocity()[3],
                     prevLink->getVelocity()[4],
                     prevLink->getVelocity()[5]).dot(axis);
  double vel2 = vec3(nextLink->getVelocity()[3],
                     nextLink->getVelocity()[4],
                     nextLink->getVelocity()[5]).dot(axis);
  joint->setVelocity(vel2 - vel1);

  transf diffTran = b1JointTran.inverse() % nextLink->getTran()  % joint->getTran(0.0).inverse();

  double val;
  Eigen::AngleAxisd aa(diffTran.rotation().normalized());
  val = aa.angle();
  axis = aa.axis();
  if (axis.z() < 0) { val = -val; }

  DBGP("link " << prevLink->getName().latin1() << " - link " << nextLink->getName().latin1());
  DBGP(" joint angle: " << val * 180.0 / M_PI << " radians: " << val << " velocity: " << vel2 - vel1);

  joint->setDynamicsVal(val);
}

transf
RevoluteDynJoint::getPrevTrans()
{
  //by definition, the origin of a link is at the location of the next joint
  //however, if the previous link is the robot base, we need to take into account
  //the transform to the start of the kinematic chain
  //this is set up by the chain itself when it initializes the joint
  return prevFrame;
}

transf
RevoluteDynJoint::getNextTrans()
{
  //only a single joint sits between the prev link and the next
  return joint->getDynamicsTran().inverse();
}

/*!
  After a timestep has been completed, this computes the current joint angles
  and velocities from the relative positions and velocities of the connected
  links.
*/
void
UniversalDynJoint::updateValues()
{
  vec3 axis, ax0, ax1, ax2;
  double val, vel1, vel2;

  transf b1JointTran = prevLink->getTran() % prevFrame;
  transf b2JointTran = nextLink->getTran() % nextFrame;

  // the z axis of the previous link - by definition, the rotation direction of the next joint
  ax0 = b1JointTran.affine().col(2);
  ax2 = b2JointTran.affine().col(2);
  ax1 = (ax2.cross(ax0)).normalized();

  DBGP("ax0: " << ax0 << " len " << ax0.norm());
  DBGP("ax1: " << ax1 << " len " << ax1.norm());
  DBGP("ax2: " << ax2 << " len " << ax2.norm());

  axis = ax1.cross(ax2);
  joint1->setWorldAxis(axis);
  vel1 = vec3(prevLink->getVelocity()[3],
              prevLink->getVelocity()[4],
              prevLink->getVelocity()[5]).dot(axis);
  vel2 = vec3(nextLink->getVelocity()[3],
              nextLink->getVelocity()[4],
              nextLink->getVelocity()[5]).dot(axis);
  joint1->setVelocity(vel2 - vel1);

  vec3 ref1 = (b1JointTran %  joint1->getTran(0.0) % joint2->getTran(0.0)).affine().col(2);
  //original GraspIt:
  val = atan2(ax2.dot(ax0.cross(ref1)), ax2.dot(ref1));

  DBGP("link " << prevLink->getName().latin1() << " - link " << nextLink->getName().latin1() << ":");
  DBGP("   joint1 angle: " << val * 180.0 / M_PI << " " << val << " (rad)");
  joint1->setDynamicsVal(val);

  //is this right here? It's different from what's done for joint1
  axis = b2JointTran.affine().col(2);
  //joint2->setWorldAxis(axis);
  joint2->setWorldAxis(ax0.cross(ax1));

  vel1 = vec3(prevLink->getVelocity()[3],
              prevLink->getVelocity()[4],
              prevLink->getVelocity()[5]).dot(axis);
  vel2 = vec3(nextLink->getVelocity()[3],
              nextLink->getVelocity()[4],
              nextLink->getVelocity()[5]).dot(axis);

  joint2->setVelocity(vel2 - vel1);

  vec3 ref2 = b2JointTran.affine() * ((joint1->getTran(0.0) % joint2->getTran(0.0)).inverse().affine().col(2));

  val = atan2(ref2.dot(ax1), ref2.dot(ax1.cross(ax2)));

  joint2->setDynamicsVal(val);
}

transf
UniversalDynJoint::getPrevTrans()
{
  return prevFrame;
}

transf
UniversalDynJoint::getNextTrans()
{
  return joint2->getDynamicsTran().inverse() % joint1->getDynamicsTran().inverse();
}

/*!
  After a timestep has been completed, this computes the current joint angles
  and velocities from the relative positions and velocities of the connected
  links.
*/
void
BallDynJoint::updateValues()
{
  transf b1JointTran, b2JointTran;
  //  transf totalTran;
  vec3 axis, ax0, ax1, ax2;
  double val, vel1, vel2;

  b1JointTran = prevLink->getTran() %  prevFrame;
  b2JointTran = nextLink->getTran() % nextFrame;

  ax0 = b1JointTran.affine().col(2);
  ax2 = b2JointTran.affine().col(2);
  ax1 = (ax2.cross(ax0)).normalized();

  DBGP("ax0: " << ax0 << " len " << ax0.norm());
  DBGP("ax1: " << ax1 << " len " << ax1.norm());
  DBGP("ax2: " << ax2 << " len " << ax2.norm());

  //  axis = b1JointTran.affine().col(2);
  axis = ax1.cross(ax2);
  joint1->setWorldAxis(axis);
  vel1 = vec3(prevLink->getVelocity()[3],
              prevLink->getVelocity()[4],
              prevLink->getVelocity()[5]).dot(axis);
  vel2 = vec3(nextLink->getVelocity()[3],
              nextLink->getVelocity()[4],
              nextLink->getVelocity()[5]).dot(axis);
  joint1->setVelocity(vel2 - vel1);

  vec3 ref1 = (b1JointTran % joint1->getTran(0.0) % joint2->getTran(0.0)).affine().col(2);
  val = atan2(ax2.dot(ax0.cross(ref1)), ax2.dot(ref1));


  //  totalTran = nextLink->getTran()*b1JointTran.inverse();

  //  val = atan2(totalTran.translation()[1],totalTran.translation()[0]);

  //diffTran = joint1->getTran(0.0).inverse() * nextLink->getTran() *
  //  b1JointTran.inverse();

  // diffTran.rotation().ToAngleAxis(val,axis);


#ifdef GRASPITDBG
  printf("link %s - link %s joint1 angle: %le   %le(rad)\n", prevLink->getName().latin1(),
         nextLink->getName().latin1(), val * 180.0 / M_PI, val);
  //    std::cout << " velocity: "<<vel2-vel1<<std::endl;
#endif
  //    printf("link %d - link %d joint angle: %lf\n",l-1,l,angle);
  //    std::cout << axis << std::endl;
  joint1->setDynamicsVal(val);

  axis = ax1;
  joint2->setWorldAxis(axis);
  vel1 = vec3(prevLink->getVelocity()[3],
              prevLink->getVelocity()[4],
              prevLink->getVelocity()[5]).dot(axis);
  vel2 = vec3(nextLink->getVelocity()[3],
              nextLink->getVelocity()[4],
              nextLink->getVelocity()[5]).dot(axis);

  joint2->setVelocity(vel2 - vel1);

  val = atan2(ax2.dot(ax0), ax2.dot(ax0.cross(ax1)));

#ifdef GRASPITDBG
  printf("link %s - link %s joint2 angle: %le   %le(rad)\n", prevLink->getName().latin1(),
         nextLink->getName().latin1(), val * 180.0 / M_PI, val);
  //    std::cout << " velocity: "<<vel2-vel1<<std::endl;
#endif
  //    printf("link %d - link %d joint angle: %lf\n",l-1,l,angle);
  //    std::cout << axis << std::endl;
  joint2->setDynamicsVal(val);

  axis = ax0.cross(ax1);
  joint3->setWorldAxis(axis);
  vel1 = vec3(prevLink->getVelocity()[3],
              prevLink->getVelocity()[4],
              prevLink->getVelocity()[5]).dot(axis);
  vel2 = vec3(nextLink->getVelocity()[3],
              nextLink->getVelocity()[4],
              nextLink->getVelocity()[5]).dot(axis);

  joint3->setVelocity(vel2 - vel1);

  vec3 ref2 = b2JointTran.affine() * ((joint1->getTran(0.0) % joint2->getTran(0.0)).inverse().affine().col(2));
  val = atan2(ref2.dot(ax1), ref2.dot(ax1.cross(ax2)));

#ifdef GRASPITDBG
  printf("link %s - link %s joint3 angle: %le   %le(rad)\n", prevLink->getName().latin1(),
         nextLink->getName().latin1(), val * 180.0 / M_PI, val);
  //    std::cout << " velocity: "<<vel2-vel1<<std::endl;
#endif
  //    printf("link %d - link %d joint angle: %lf\n",l-1,l,angle);
  //    std::cout << axis << std::endl;
  joint3->setDynamicsVal(val);
}

transf
BallDynJoint::getPrevTrans()
{
  return prevFrame;
}

transf
BallDynJoint::getNextTrans()
{
  return joint3->getDynamicsTran().inverse() % joint2->getDynamicsTran().inverse() % joint1->getDynamicsTran().inverse();
}
