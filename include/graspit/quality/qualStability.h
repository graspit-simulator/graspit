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
// Author(s):  Andrew T. Miller
//
// $Id: quality.h,v 1.8 2009/03/31 15:37:06 cmatei Exp $
//
//######################################################################

#ifndef STABILITY_H

#include "graspit/quality/quality.h"

//! The experimental quality measure
/*!
  This experimental quality measure tests if a grasp can withstand a
  given wrench. 
*/
class QualStability : public QualityMeasure {

  double mWrenchMultiplier;
  std::vector<double> mWrench;

  double mPreloadMultiplier;
  std::vector<double> mPreload;

  bool mSingle_step;
  bool mIterative;
  bool mCone_movement;
  bool mRigid;
  bool mMap;

  //! The string identifying this qm type
  static const char *type;

 public:
  QualStability(Grasp *g, QString n);
  QualStability(qmDlgDataT *data);
  ~QualStability();

  /*! Returns the type of this quality measure expressed as a string */
  const char *getType() const {return type;}

  double evaluate();
  double evaluate3D();

  static void buildParamArea(qmDlgDataT *qmData);

  /*! Returns the type of this class expressed as a string. */
  static const char *getClassType() {return type;}

  static std::vector<double> getDefaultPreload(Grasp *grasp);

  static std::vector<double> getGravityWrench(Grasp *grasp);

  void setPreload(double preloadMultiplier, const std::vector<double> &preload) {
    mPreloadMultiplier = preloadMultiplier;
    mPreload = preload;
  }

  void setParameters(bool single_step, bool iterative, bool cone_movement, bool rigid, bool map) {
    mSingle_step = single_step;
    mIterative = iterative; 
    mCone_movement = cone_movement;
    mRigid = rigid;
    mMap = map;
  }

  void setWrench(double wrenchMultiplier, const std::vector<double> &wrench);
};

#define STABILITY_H
#endif
