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

#include "quality.h"

#include <QGridLayout>
#include <QBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <q3groupbox.h>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

#include "grasp.h"
#include "gws.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//Static class variables

//! A list of the possible qm types expressed as strings
const char *QualityMeasure::TYPE_LIST[] = {"Epsilon","Volume",NULL};

const char *QualEpsilon::type = "Epsilon";
const char *QualVolume::type = "Volume";

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
  if (!strcmp(qmData->qmType,QualEpsilon::getClassType()))
    QualEpsilon::buildParamArea(qmData);

  else if (!strcmp(qmData->qmType,QualVolume::getClassType()))
    QualVolume::buildParamArea(qmData);
}

/*!
  Creates an instance of the specific subclass that is named in the string
  pointed to by qmData->qmType.
*/
QualityMeasure *
QualityMeasure::createInstance(qmDlgDataT *qmData)
{
  if (!strcmp(qmData->qmType,QualEpsilon::getClassType()))
    return new QualEpsilon(qmData);

  else if (!strcmp(qmData->qmType,QualVolume::getClassType()))
    return new QualVolume(qmData);

  return NULL;
}


///////////////////////////////////////////////////////////////////////////////
//
//                                QualEpsilon
//
///////////////////////////////////////////////////////////////////////////////

//! Structure holding pointers to UI items specific to this quality measure
/*!
  Quality parameter structure that simply holds a pointer to the gws type
  combo box in the parameter area.
*/
struct QualEpsilonParamT {
  QComboBox *gwsTypeComboBox,*twsTypeComboBox;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualEpsilon::QualEpsilon(qmDlgDataT *data) : QualityMeasure(data)
{
  QualEpsilonParamT *params = (QualEpsilonParamT *)data->paramPtr;
  gws = grasp->addGWS(params->gwsTypeComboBox->currentText().latin1());
}

QualEpsilon::QualEpsilon(Grasp *g, QString n, const char *gwsType) : QualityMeasure(g,n)
{
	gws = grasp->addGWS( gwsType );
}
/*!
  Removes the GWS used for this qm from the grasp.  The grasp will delete it
  if necessary.
*/
QualEpsilon::~QualEpsilon()
{
#ifdef GRASPITDBG
   std::cout << "deleting QualEpsilon" << std::endl;
#endif
  grasp->removeGWS(gws);
}

/*!
  Returns the minimum distance from the origin to any of the hyperplanes
  defining the boundary of the GWS.
*/
double
QualEpsilon::evaluate()
{
  double minOffset;

  if (!gws->hyperPlanes) {
#ifdef GRASPITDBG
    std::cout << "hyperplanes is NULL"<<std::endl;
#endif
    return -1.0;
  }

#ifdef GRASPITDBG
  std::cout << "numHyperPlanes"<<gws->numHyperPlanes<<std::endl;
#endif

  for (int i=0;i<gws->numHyperPlanes;i++) {
    if (i==0 || -(gws->hyperPlanes[i][6])<minOffset) {
      minOffset = -(gws->hyperPlanes[i][6]);
      if (minOffset < 0) return -1.0;     // not a force-closure grasp
    }
  }

  val = minOffset;
  return val;
}

double
QualEpsilon::evaluate3D()
{
  if (!gws->hyperPlanes) {
#ifdef GRASPITDBG
    std::cout << "hyperplanes is NULL"<<std::endl;
#endif
    return -1.0e3;
  }

#ifdef GRASPITDBG
  std::cout << "numHyperPlanes"<<gws->numHyperPlanes<<std::endl;
#endif

  double minOffset = -1.0e3;
  for (int i=0;i<gws->numHyperPlanes;i++) {
    if ( gws->hyperPlanes[i][6] > minOffset) {
	  minOffset = gws->hyperPlanes[i][6];
    }
  }
  val = - minOffset;
  return val;
}

/*!
  Builds an epsilon qm parameter area within the quality measure dialog
  box.  This includes building a QT combo box for the GWS selection and
  one for the TWS selection.  However, currently there is only one possible
  TWS.  In the future, there may be ways to define others.
*/
void
QualEpsilon::buildParamArea(qmDlgDataT *qmData)
{
  int i;
  static QualEpsilonParamT params;

  QualEpsilon *currQM = (QualEpsilon *)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building qualepsilon" << std::endl;
#endif

  QLayout *l = new QGridLayout(qmData->settingsArea,2,2,1);
  l->setAutoAdd(true);

  // create the GWS type menu
  new QLabel(QString("Limit unit GWS using:"),qmData->settingsArea);
  params.gwsTypeComboBox = new QComboBox(qmData->settingsArea,"gwsComboBox");  

  new QLabel(QString("Task Wrench Space (TWS):"),qmData->settingsArea);
  params.twsTypeComboBox = new QComboBox(qmData->settingsArea,"twsComboBox");  

  // count the number of possible gws types
  for (i=0;GWS::TYPE_LIST[i];i++) {
    params.gwsTypeComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
    if (currQM && !strcmp(currQM->gws->getType(),GWS::TYPE_LIST[i]))
      params.gwsTypeComboBox->setCurrentItem(i);
  }
  
  params.twsTypeComboBox->insertItem(QString("Unit Ball"));

  qmData->paramPtr = &params;
}

///////////////////////////////////////////////////////////////////////////////
//
//                                QualVolume
//
///////////////////////////////////////////////////////////////////////////////

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

QualVolume::QualVolume(Grasp *g, QString n, const char *gwsType) : QualityMeasure(g,n)
{
	gws = grasp->addGWS( gwsType );
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
  new QLabel(QString("Limit unit GWS using:"),qmData->settingsArea);
  QComboBox *gwsComboBox = new QComboBox(qmData->settingsArea,"gwsComboBox");  

  // count the number of possible gws types
  for (i=0;GWS::TYPE_LIST[i];i++) {
    gwsComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
    if (currQM && !strcmp(currQM->gws->getType(),GWS::TYPE_LIST[i]))
      gwsComboBox->setCurrentItem(i);
  }

  qmData->paramPtr = gwsComboBox;
}

///////////////////////////////////////////////////////////////////////////////
//
//                      More quality measures here
//
///////////////////////////////////////////////////////////////////////////////
