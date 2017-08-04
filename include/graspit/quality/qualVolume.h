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

#ifndef VOLUME_H

#include "graspit/quality/quality.h"

//! The volume quality measure
/*!
  The volume quality measure measures the volume of the unit Grasp Wrench
  Space (GWS). This is an average case grasp quality measure.  The parameter
  for this quality measure is:
     The GWS type
*/
class QualVolume : public QualityMeasure {
    //! A pointer to the GWS that this qm should use for its calculation
    GWS *gws;

    //! The string identifying this qm type
    static const char *type;

  public:
    QualVolume(qmDlgDataT *data);
    QualVolume(Grasp *g, QString n, const char *gwsType);
    ~QualVolume();

    /*! Returns the type of this quality measure expressed as a string */
    const char *getType() const {return type;}

    double evaluate();
    double evaluate3D();

    static void buildParamArea(qmDlgDataT *qmData);

    /*! Returns the type of this class expressed as a string. */
    static const char *getClassType() {return type;}
};

#define VOLUME_H
#endif
