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

#include "graspit/quality/qualVolume.h"

#include <QGridLayout>
#include <QLabel>
#include <QComboBox>

#include "graspit/grasp.h"
#include "graspit/gws.h"

const char *QualVolume::type = "Volume";

//! Structure holding pointers to UI items specific to this quality measure
/*!
  Quality parameter structure that simply holds a pointer to the gws type
  combo box in the parameter area.
*/
struct QualVolumeParamT {
  QWidget *gwsTypeComboBox;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualVolume::QualVolume(qmDlgDataT *data) : QualityMeasure(data)
{
  QComboBox *gwsType = (QComboBox *)data->paramPtr;
  gws = grasp->addGWS(gwsType->currentText().latin1());
}

QualVolume::QualVolume(Grasp *g, QString n, const char *gwsType) : QualityMeasure(g, n)
{
  gws = grasp->addGWS(gwsType);
}


/*!
  Removes the GWS used for this qm from the grasp.  The grasp will delete it
  if necessary.
*/
QualVolume::~QualVolume()
{
#ifdef GRASPITDBG
  std::cout << "deleting QualVolume" << std::endl;
#endif
  grasp->removeGWS(gws);
}

/*!
  Returns the volume of the GWS.  The GWS is responsible for computing its
  volume when it is updated.
*/
double
QualVolume::evaluate()
{
  val = gws->hullVolume;
  return val;
}

double
QualVolume::evaluate3D()
{
  val = gws->hullVolume;
  return val;
}

/*!
  Builds a volume qm parameter area within the quality measure dialog
  box.  This is simply building a QT combo box for the GWS selection.
*/
void
QualVolume::buildParamArea(qmDlgDataT *qmData)
{
  int i;
  QualVolume *currQM = (QualVolume *)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building qualvolume" << std::endl;
#endif

  QBoxLayout *l = new QHBoxLayout(qmData->settingsArea);
  l->setAutoAdd(true);

  // create the GWS type menu
  new QLabel(QString("Limit unit GWS using:"), qmData->settingsArea);
  QComboBox *gwsComboBox = new QComboBox(qmData->settingsArea, "gwsComboBox");

  // count the number of possible gws types
  for (i = 0; GWS::TYPE_LIST[i]; i++) {
    gwsComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
    if (currQM && !strcmp(currQM->gws->getType(), GWS::TYPE_LIST[i])) {
      gwsComboBox->setCurrentItem(i);
    }
  }

  qmData->paramPtr = gwsComboBox;
}
