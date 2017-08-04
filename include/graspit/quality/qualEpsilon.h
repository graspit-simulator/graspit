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

#ifndef EPSILON_H

#include "graspit/quality/quality.h"

//! The epsilon quality measure
/*!
  The epsilon quality measure measures the size of the largest Task Wrench
  Space (TWS_ that can fit within the unit Grasp Wrench Space (GWS).  In the
  case of the ball TWS, the measure because simply the euclidean distance from
  the wrench space origin to the closest point on the  hull bouandary.  This
  is a worst case grasp quality measure.  The parameters for this quality
  measure are:
     The GWS type
     The TWS type
*/
class QualEpsilon : public QualityMeasure {
    //! A pointer to the GWS that this qm should use for its calculation
    GWS *gws;

    //! The string identifying this qm type
    static const char *type;

  public:
    QualEpsilon(qmDlgDataT *data);
    QualEpsilon(Grasp *g, QString n, const char *gwsType);
    ~QualEpsilon();

    /*! Returns the type of this quality measure expressed as a string */
    const char *getType() const {return type;}

    double evaluate();
    double evaluate3D();

    static void buildParamArea(qmDlgDataT *qmData);

    /*! Returns the type of this class expressed as a string. */
    static const char *getClassType() {return type;}
};

#define EPSILON_H
#endif
