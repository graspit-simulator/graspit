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
// $Id: scanSimulator.cpp,v 1.5 2010/08/11 21:35:22 cmatei Exp $
//
//######################################################################

#include <assert.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/SoPickedPoint.h>

#include "graspit/scanSimulator.h"
#include "graspit/graspitCore.h"
#include "graspit/ivmgr.h"
#include "graspit/world.h"
#include "graspit/matvec3D.h"

ScanSimulator::ScanSimulator()
{
  setPosition(position(0, 0, 0), vec3(0, 0, 1), vec3(0, -1, 0), STEREO_CAMERA);
  mHMin = -70; mHMax = 70; mHLines = 140;
  mVMin = -30; mVMax = 30; mVLines = 60;
  mType = SCANNER_COORDINATES;
}

void ScanSimulator::setPosition(transf tr, AxesConvention convention)
{
  mTran = tr;
  mTranInv = mTran.inverse();
  mPosition = tr.translation();
  switch (convention) {
    case STEREO_CAMERA:
      mDirection = tr.affine().col(2);
      mUp = -1.0 * tr.affine().col(1);
      mHorizDirection = mUp.cross(mDirection);
      break;
  }
}

void ScanSimulator::setPosition(position p, vec3 optical_axis, vec3 up_axis, AxesConvention convention)
{
  mPosition = p; mDirection = optical_axis.normalized(); mUp = up_axis.normalized();
  mHorizDirection = (mUp.cross(mDirection)).normalized();
  mUp = mDirection.cross(mHorizDirection);
  switch (convention) {
    case STEREO_CAMERA:
      mat3 m;
      m.col(0) = -1.0 * mHorizDirection;
      m.col(1) = -1.0 * mUp;
      m.col(2) = mDirection;
      mTran = transf(m, vec3(mPosition.x(), mPosition.y(), mPosition.z()));
      mTranInv = mTran.inverse();
      break;
  }
}

void ScanSimulator::scan(std::vector<position> *cloud, std::vector<RawScanPoint> *rawData)
{
  float hfov = (mHMax - mHMin) * M_PI / 180.0;
  float vfov = (mVMax - mVMin) * M_PI / 180.0;

  assert(hfov >= 0 && vfov >= 0);
  assert(mHLines > 0 && mVLines > 0);

  float hstep = hfov / mHLines;
  float vstep = vfov / mVLines;

  fprintf(stderr, "Vfov %f and vstep %f and lines %d\n", vfov, vstep, mVLines);

  float hAngle;
  float vAngle = - vfov / 2.0;

  vec3 rayDirection;
  position rayPoint;
  RawScanPoint rawPoint;

  while (vAngle < vfov / 2.0) {
    hAngle = - hfov / 2.0;
    while (hAngle < hfov / 2.0) {

      computeRayDirection(hAngle, vAngle, rayDirection);
      rawPoint.hAngle = hAngle; rawPoint.vAngle = vAngle;
      rawPoint.dx = rayDirection.x();
      rawPoint.dy = rayDirection.y();
      rawPoint.dz = rayDirection.z();

      if (shootRay(rayDirection, rayPoint)) {
        vec3 dist = rayPoint - mPosition;
        rawPoint.distance = dist.norm();

        //we get the point in world coordinates
        if (mType == SCANNER_COORDINATES) {
          rayPoint = mTranInv * (rayPoint);
        }
        cloud->push_back(rayPoint);


      } else {
        rawPoint.distance = -1;
      }

      if (rawData) {
        rawData->push_back(rawPoint);
      }

      hAngle += hstep;
    }
    vAngle += vstep;
    fprintf(stderr, "Vangle: %f\n", vAngle);
  }
}

void ScanSimulator::computeRayDirection(float hAngle, float vAngle, vec3 &rayDirection)
{
  Eigen::AngleAxisd r1aa = Eigen::AngleAxisd(vAngle, mHorizDirection);
  Quaternion r1(r1aa);
  Eigen::AngleAxisd r2aa = Eigen::AngleAxisd(hAngle, r1 * mUp);
  Quaternion r2(r2aa);
  rayDirection = r2 * r1 * mDirection;
}

bool ScanSimulator::shootRay(const vec3 &rayDirection, position &rayPoint)
{
  SoRayPickAction action(graspitCore->getIVmgr()->getViewer()->getViewportRegion());

  action.setRay(toSbVec3f(mPosition), toSbVec3f(rayDirection), 0.0, -1.0);
  action.setPickAll(false);

  action.apply((SoNode *)graspitCore->getWorld()->getIVRoot());

  SoPickedPoint *pp = action.getPickedPoint();
  if (!pp) { return false; }

  rayPoint = SbVec3fTovec3(pp->getPoint());
  return true;
}

