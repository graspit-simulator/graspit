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

#ifndef PGR_H

#include "graspit/quality/quality.h"

#include <vector>

class Grasp;
struct threadArgs {
  Grasp *g;
  Matrix *wrench;
  double maxForce;
  std::vector<int> states;
  std::vector<int> finalStates;
  double maxPCR;
  bool doneFlag;
  pthread_mutex_t *statesMutex_pt;
  pthread_mutex_t *resultMutex_pt;
};

//! The Potential Grasp Robustness (PGR) quality measure
/*!
  The PGR quality measure is similar to the PCR metric but considers
  contacts to be in one of three possible states:
    1. both non-negativity and friction constraints are satisfied.
       The force may lie anywhere inside the friction cone
    2. the contact is sliding and only non-negativity is satisfied.
       The force may only have a normal component
    3. the contact has broken. No force may act
  The PGR quality metric maximizes the PCR metric over all possible
  contact state combinations. This quality metric was introduced and 
  is further explained in 'Contact and Grasp Robustness Measures: 
  Analysis and Experiment' (1997) by Prattichizzo et al.
*/
class QualPGR : public QualityMeasure {

  double mWrenchMultiplier;
  std::vector<double> mWrench;
  double mMaxForce;
  int mMaxContacts;

  //! The string identifying this qm type
  static const char *type;

 public:
  QualPGR(qmDlgDataT *data);
  ~QualPGR();

  /*! Returns the type of this quality measure expressed as a string */
  const char *getType() const {return type;}

  double evaluate();
  double evaluate3D();

  double evaluatePGR(Matrix &wrench, double maxForce, int maxContacts);

  static void* evaluatePCRThread(void *argv);

  static void buildParamArea(qmDlgDataT *qmData);

  /*! Returns the type of this class expressed as a string. */
  static const char *getClassType() {return type;}
};

#define PGR_H
#endif
