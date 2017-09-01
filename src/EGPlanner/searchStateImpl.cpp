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
// $Id: searchStateImpl.cpp,v 1.12 2010/01/14 01:21:09 cmatei Exp $
//
//######################################################################

#include "graspit/EGPlanner/searchStateImpl.h"
#include "graspit/robot.h"
#include "graspit/eigenGrasp.h"

#include "graspit/debug.h"

void PostureStateDOF::createVariables()
{
  QString name("DOF ");
  QString num;
  for (int i = 0; i < mHand->getNumDOF(); i++) {
    num.setNum(i);
    mVariables.push_back(new SearchVariable(name + num,
                                            mHand->getDOF(i)->getMin(), mHand->getDOF(i)->getMax(), mHand->getDOF(i)->getMin(),
                                            0.5 * (mHand->getDOF(i)->getMax()
                                                   - mHand->getDOF(i)->getMin())));
  }
}
void PostureStateDOF::getHandDOF(double *dof) const
{
  for (int i = 0; i < mHand->getNumDOF(); i++) {
    dof[i] = readVariable(i);
  }
}
void PostureStateDOF::storeHandDOF(const double *dof)
{
  for (int i = 0; i < mHand->getNumDOF(); i++) {
    getVariable(i)->setValue(dof[i]);
  }
}

void PostureStateEigen::createVariables()
{
  for (int i = 0; i < mHand->getEigenGrasps()->getSize(); i++) {
    QString name("EG ");
    QString num;
    num.setNum(i);
    float min, max;
    if (mHand->getEigenGrasps()->getGrasp(i)->mPredefinedLimits) {
      min = mHand->getEigenGrasps()->getGrasp(i)->mMin;
      max = mHand->getEigenGrasps()->getGrasp(i)->mMax;
    } else if (mHand->isA("Pr2Gripper")) {
      min = -0.6f;
      max = 0.6f;
    } else if (mHand->isA("Pr2Gripper2010")) {
      min = -0.45f;
      max = 0.45f;
    } else {
      //if limits are not pre-defined in files, we use these hard-coded values
      //as we don't actually trust our abilities to compute them
      min = -4.0f;
      max = 4.0f;
    }
    float jump  = (max - min) / 4.0;
    mVariables.push_back(new SearchVariable(name + num, min, max, min, jump));
  }
}
void PostureStateEigen::getHandDOF(double *dof) const
{
  double *eg = new double[ mHand->getEigenGrasps()->getSize() ];
  for (int i = 0; i < mHand->getEigenGrasps()->getSize(); i++) {
    eg[i] = readVariable(i);
  }
  bool r = mHand->getEigenGrasps()->isRigid();
  mHand->getEigenGrasps()->setRigid(true);
  mHand->getEigenGrasps()->getDOF(eg, dof);
  mHand->checkSetDOFVals(dof);
  mHand->getEigenGrasps()->setRigid(r);
  delete [] eg;
}
void PostureStateEigen::storeHandDOF(const double *dof)
{
  double *eg = new double[ mHand->getEigenGrasps()->getSize() ];
  mHand->getEigenGrasps()->getAmp(eg, dof);
  for (int i = 0; i < mHand->getEigenGrasps()->getSize(); i++) {
    getVariable(i)->setValue(eg[i]);
  }
  delete [] eg;
}

// ------------------------------------ POSITION STATES -------------------------------

void PositionStateComplete::createVariables()
{
  mVariables.push_back(new SearchVariable("Tx", -250, 250, 0, 100));
  mVariables.push_back(new SearchVariable("Ty", -250, 250, 0, 100));
  mVariables.push_back(new SearchVariable("Tz", -250, 250, 250, 100));
  mVariables.push_back(new SearchVariable("Qw", -5, 5, .7071, 1));
  mVariables.push_back(new SearchVariable("Qx", -5, 5, 0, 1));
  mVariables.push_back(new SearchVariable("Qy", -5, 5, 0.7071, 1));
  mVariables.push_back(new SearchVariable("Qz", -5, 5, 0, 1));
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
  return transf(Quaternion(qw, qx, qy, qz), vec3(tx, ty, tz));
}
void PositionStateComplete::setTran(const transf &t)
{
  getVariable("Tx")->setValue(t.translation().x());
  getVariable("Ty")->setValue(t.translation().y());
  getVariable("Tz")->setValue(t.translation().z());
  getVariable("Qw")->setValue(t.rotation().w());
  getVariable("Qx")->setValue(t.rotation().x());
  getVariable("Qy")->setValue(t.rotation().y());
  getVariable("Qz")->setValue(t.rotation().z());
}

void PositionStateAA::createVariables()
{
  mVariables.push_back(new SearchVariable("Tx", -250, 250, 0, 150));
  mVariables.push_back(new SearchVariable("Ty", -250, 250, 0, 150));
  mVariables.push_back(new SearchVariable("Tz", -250, 350, 350, 150));
  mVariables.push_back(new SearchVariable("theta", 0, M_PI, 0, M_PI / 5));
  mVariables.push_back(new SearchVariable("phi", -M_PI, M_PI, 0, M_PI / 2, true));
  mVariables.push_back(new SearchVariable("alpha", 0, M_PI, M_PI / 2, M_PI / 2));
}
transf PositionStateAA::getCoreTran() const
{
  double tx = readVariable("Tx");
  double ty = readVariable("Ty");
  double tz = readVariable("Tz");
  double theta = readVariable("theta");
  double phi = readVariable("phi");
  double alpha = readVariable("alpha");
  transf coreTran = transf::TRANSLATION(vec3(tx, ty, tz)) % transf::AXIS_ANGLE_ROTATION(alpha, vec3(sin(theta) * cos(phi) , sin(theta) * sin(phi) , cos(theta)));
  //transform now returned relative to hand approach transform
  return coreTran % mHand->getApproachTran().inverse();

}
void PositionStateAA::setTran(const transf &t)
{
  transf rt = t % mHand->getApproachTran();

  //translation is direct
  getVariable("Tx")->setValue(rt.translation().x());
  getVariable("Ty")->setValue(rt.translation().y());
  getVariable("Tz")->setValue(rt.translation().z());

  vec3 axis; double angle;
  Eigen::AngleAxisd aa( rt.rotation());
  axis = aa.axis();
  angle = aa.angle();

  if (angle < 0) {
    angle = -angle;
    axis = -1.0 * axis;
  }
  if (angle > M_PI) {
    angle = 2 * M_PI - angle;
    axis = -1.0 * axis;
  }
  assert(0 <= angle);
  assert(angle <= M_PI);
  //the angle
  getVariable("alpha")->setValue(angle);
  //the axis
  if (fabs(axis.x()) > 1.0e-7 || fabs(axis.y()) > 1.0e-7) {
    getVariable("phi")->setValue(atan2(axis.y(), axis.x()));
    double xy_len = sqrt(axis.x() * axis.x() + axis.y() * axis.y());
    getVariable("theta")->setValue(atan2(xy_len, axis.z()));
  } else {
    getVariable("phi")->setValue(0.0);
    getVariable("theta")->setValue(M_PI / 2.0);
  }
}

void PositionStateEllipsoid::createVariables()
{
  mVariables.push_back(new SearchVariable("beta", -M_PI / 2.0, M_PI / 2.0, 0, M_PI / 2.0));
  mVariables.push_back(new SearchVariable("gamma", -M_PI, M_PI, 0, M_PI, true));
  mVariables.push_back(new SearchVariable("tau", -M_PI, M_PI, 0, M_PI, true));
  mVariables.push_back(new SearchVariable("dist", -50, 100, 0, 50));

  //ellipsoid scaling parameters
  mParameters.push_back(SearchParameter("a", 80));
  mParameters.push_back(SearchParameter("b", 80));
  mParameters.push_back(SearchParameter("c", 160));
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
  double px, py, pz;
  px =  a * cos(beta) * cos(gamma);
  py =  b * cos(beta) * sin(gamma);
  pz =  c * sin(beta);

  //compute normal direction - for some reason this always points INSIDE the ellipsoid
  vec3 n1(-a * sin(beta)*cos(gamma), -b * sin(beta)*sin(gamma), c * cos(beta));
  vec3 n2(-a * cos(beta)*sin(gamma),  b * cos(beta)*cos(gamma), 0);
  vec3 normal = n1.normalized().cross(n2.normalized());

  vec3 xdir(1, 0, 0);
  vec3 ydir = normal.cross(xdir.normalized());
  xdir = ydir.cross(normal);
  mat3 r;
  r.col(0) = xdir.normalized();
  r.col(1) = ydir.normalized();
  r.col(2) = normal.normalized();

  transf handTran = transf(r, vec3(px, py, pz) - distance * normal);
  Eigen::AngleAxisd aa = Eigen::AngleAxisd(tau, vec3(0, 0, 1));
  Quaternion zrot(aa);
  handTran = handTran % transf(zrot, vec3(0, 0, 0));
  return handTran % mHand->getApproachTran().inverse();
  //So: hand tranform is: move onto the ellipsoid and rotate z axis in normal direction
  //    --> hand approach transform inverted to get to hand origin
}
void PositionStateEllipsoid::setTran(const transf &t)
{
  //not yet implemented
  const_cast<transf &>(t) = t; //kill the warning
  assert(0);
}

void PositionStateApproach::createVariables()
{
  mVariables.push_back(new SearchVariable("dist", -30, 200, 0, 100));
  mVariables.push_back(new SearchVariable("wrist 1", -M_PI / 3.0, M_PI / 3.0, 0, M_PI / 6.0));
  mVariables.push_back(new SearchVariable("wrist 2", -M_PI / 3.0, M_PI / 3.0, 0, M_PI / 6.0));
}
transf PositionStateApproach::getCoreTran() const
{
  double dist = readVariable("dist");
  double rx = readVariable("wrist 1");
  double ry = readVariable("wrist 2");
  transf handTran = transf(Quaternion::Identity(), vec3(0, 0, dist));
  handTran = transf::AXIS_ANGLE_ROTATION(ry, vec3(0, 1, 0)) % transf::AXIS_ANGLE_ROTATION(rx, vec3(1, 0, 0)) %  handTran;
  return mHand->getApproachTran() % handTran % mHand->getApproachTran().inverse();
}
void PositionStateApproach::setTran(const transf &t)
{
  //not yet implemented
  const_cast<transf &>(t) = t; //kill the warning
  assert(0);
}
