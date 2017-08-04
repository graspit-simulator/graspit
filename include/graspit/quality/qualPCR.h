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
// Author(s):  Maximilian Haas-Heger
//
//######################################################################

#ifndef PCR_H

#include "graspit/quality/quality.h"

#include <vector>

class Matrix;

//! The Potential Contact Robustness (PCR) quality measure
/*!
  The PCR quality measure measures the robustness of the contacts of 
  a given grasp. It quantifies how far each contact force is from 
  violating the following three constraints:
    1. Non-negativity
    2. Friction cone
    3. Maximum force
  This quality metric was introduced and is further explained in 
  'Contact and Grasp Robustness Measures: Analysis and Experiment' 
  (1997) by Prattichizzo et al.
*/
class QualPCR : public QualityMeasure {

  double mWrenchMultiplier;
  std::vector<double> mWrench;
  double mMaxForce;

  //! The string identifying this qm type
  static const char *type;

 public:
  QualPCR(qmDlgDataT *data);
  ~QualPCR();

  /*! Returns the type of this quality measure expressed as a string */
  const char *getType() const {return type;}

  double evaluate();
  double evaluate3D();

  static double evaluatePCR(Grasp *g, 
                     const Matrix &wrench, 
                     double maxForce, 
                     std::vector<int> states = std::vector<int>(),
                     bool show = true);

  static void buildParamArea(qmDlgDataT *qmData);

  /*! Returns the type of this class expressed as a string. */
  static const char *getClassType() {return type;}

  static std::vector<double> getGravityWrench(Grasp *grasp);
};

#define PCR_H
#endif
