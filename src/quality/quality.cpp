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
// $Id: quality.cpp,v 1.12 2009/03/31 15:36:58 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements base QualityMeasure class and specifc qm classes.
*/

#include "graspit/quality/quality.h"

#include "graspit/quality/qualEpsilon.h"
#include "graspit/quality/qualVolume.h"
#include "graspit/quality/qualPCR.h"
#include "graspit/quality/qualPGR.h"

#include <QGridLayout>
#include <QBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <q3groupbox.h>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

#include "graspit/grasp.h"
#include "graspit/gws.h"
#include "graspit/math/matrix.h"
#include "graspit/robot.h"

#include <limits>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//Static class variables

//! A list of the possible qm types expressed as strings
const char *QualityMeasure::TYPE_LIST[] = {"Epsilon", "Volume", "PCR", "PGR", NULL};

/*!
  Sets the name of this QM to the text contained within the name widget.
  Sets the grasp pointer for this qm to the grasp in qmData.
*/
QualityMeasure::QualityMeasure(qmDlgDataT *qmData)
{
  name = qmData->qmName->text();
  grasp = qmData->grasp;
}

QualityMeasure::QualityMeasure(Grasp *g, QString n)
{
  grasp = g;
  name = n;
}

/*!
  Stub Destructor
*/
QualityMeasure::~QualityMeasure()
{
#ifdef GRASPITDBG
  std::cout << "deleting QualityMeasure" << std::endl;
#endif
}


/*!
  Calls the appropriate buildParameterArea method from the specific subclass
  of the quality measure contained within qmData.  The type of the qm is
  determined by examining the string qmData->qmType.
*/
void
QualityMeasure::buildParamArea(qmDlgDataT *qmData)
{
  if (!strcmp(qmData->qmType, QualEpsilon::getClassType())) {
    QualEpsilon::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType, QualVolume::getClassType())) {
    QualVolume::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPCR::getClassType())) {
    QualPCR::buildParamArea(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPGR::getClassType())) {
    QualPGR::buildParamArea(qmData);
  }
}

/*!
  Creates an instance of the specific subclass that is named in the string
  pointed to by qmData->qmType.
*/
QualityMeasure *
QualityMeasure::createInstance(qmDlgDataT *qmData)
{
  if (!strcmp(qmData->qmType, QualEpsilon::getClassType())) {
    return new QualEpsilon(qmData);
  }

  else if (!strcmp(qmData->qmType, QualVolume::getClassType())) {
    return new QualVolume(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPCR::getClassType())) {
    return new QualPCR(qmData);
  }

  else if (!strcmp(qmData->qmType,QualPGR::getClassType())) {
    return new QualPGR(qmData);
  }

  return NULL;
}
