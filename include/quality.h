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
#include <list>
#include <vector>

class Grasp;         // defined in grasp.h
class GWS;           // defined in gws.h
class QComboBox;
class QualityMeasure;
class QHBox;
class QLineEdit;
class QWidget;
class Grasp;

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

  static void buildParamArea(qmDlgDataT *qmData);

  /*! Returns the type of this class expressed as a string. */
  static const char *getClassType() {return type;}

  static std::vector<double> getGravityWrench(Grasp *grasp);
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

  static void buildParamArea(qmDlgDataT *qmData);

  /*! Returns the type of this class expressed as a string. */
  static const char *getClassType() {return type;}
};

/*
class QualWeighted : public QualityMeasure {
  QualityMeausre *qm1,*qm2;
  double weight1,weight2;

 public:
  QualWeighted(Grasp *g, char *n, QualityMeasure *q1, QualityMeasure *q2,
         double w1, double w2);
  double evalute() {return weight1*qm1->evaluate() + weight2*qm2->evaluate();}

};

*/
#define QUALITY_H
#endif
