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

/*! \file
  \brief Defines the base QualityMeasure class and specific qm classes.
 */
#ifndef QUALITY_H

#include <QString>
#include <string.h>
#include <vector>

class Grasp;         // defined in grasp.h
class GWS;           // defined in gws.h
class QComboBox;
class QualityMeasure;
class QHBox;
class QLineEdit;
class QWidget;

//! A collection of pointers to exchange data with the qm dialog box
/*!
  A collection of pointers to exchange data with the qm dialog box.
*/
struct qmDlgDataT {
  //! The current grasp
  Grasp *grasp;

  //! A pointer to the settings area widget of the dlg box (qm must populate)
  QWidget *settingsArea;

  //! A pointer to the qm Type selection box in the dlg box
  QComboBox *qmTypeComboBox;

  //! The pointer to the name field in the dlg box
  QLineEdit *qmName;

  //! The string quality measure type
  const char *qmType;

  //! The current quality measure
  QualityMeasure *currQM;

  //! Pointer to another stucture containing data for this specific type of QM
  void *paramPtr;
};


//! Abstract base class for quality measures
/*!
  A quality measure is associated with a particular grasp.  Each individual
  type of quality measure must be able to build a parameters area for the
  quality measure dialog box.  It also must be able to produce a scalar real
  value that evaluates its grasp in someway.
*/
class QualityMeasure {
    //! The user chosen name of this qm instance
    QString name;

  protected:
    //! A pointer to the grasp this qm is associated with
    Grasp *grasp;

    //! The current value of the qm
    double val;

  public:
    QualityMeasure(qmDlgDataT *data);
    QualityMeasure(Grasp *g, QString n);
    virtual ~QualityMeasure();

    /*! Returns the type of this quality measure expressed as a string */
    virtual const char *getType() const = 0;

    /*! Returns the user chosen name of this qm instance */
    virtual QString getName() {return name;}

    /*! Returns the quality of the grasp associated with this qm */
    virtual double evaluate() = 0;
    virtual double evaluate3D() = 0;

    static void buildParamArea(qmDlgDataT *qmData);
    static QualityMeasure *createInstance(qmDlgDataT *qmData);
    static const char *TYPE_LIST[];

};

#define QUALITY_H
#endif
