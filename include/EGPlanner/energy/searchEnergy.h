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
// Author(s): Matei T. Ciocarlie
//
// $Id: searchEnergy.h,v 1.27 2009/09/13 19:56:51 hao Exp $
//
//######################################################################

#ifndef _searchenergy_h_
#define _searchenergy_h_

#include <ostream>
#include <list>
#include <QObject>

#include "search.h"
#include "matvec3D.h"

class Hand;
class Body;
class QualityMeasure;
class GraspPlanningState;


class SearchEnergy : public QObject
{
    Q_OBJECT

protected:

    Hand *mHand;
    Body *mObject;

    QualityMeasure *mVolQual;
    QualityMeasure *mEpsQual;

    SearchEnergyType mType;
    SearchContactType mContactType;

    /*! If this flag is set, the hand is disconnected from the scene graph while
        the calculator does energy computations */
    bool mDisableRendering;

    //! If not null, it will print its output here
    mutable std::ostream *mOut;

    //! This is where the decision is made of which type of energy value should be computed and returned
    virtual double energy() const = 0;

    //! Checks if the current state is legal or not (usually legal means no interpenetrations)
    virtual bool legal() const;

    void setHandAndObject(Hand *h, Body *o);

    void createQualityMeasures();

    void setType(SearchEnergyType _mType){mType = _mType;}

    SearchEnergy();

public:

    static SearchEnergy * getSearchEnergy(SearchEnergyType t);

    void setContactType(SearchContactType t){mContactType = t;}
    double getEpsQual();
    double getVolQual();
    SearchContactType getContactType() const {return mContactType;}
    void disableRendering(bool dr){mDisableRendering = dr;}

    bool isType(SearchEnergyType t){return t == mType;}

    //! Sets the stat file where results are to be written
    void setStatStream(std::ostream *out) const {mOut = out;}

    /*! This is the main interface to this class. It is passed a GraspPlanningState* and returns whether the HandObjectState is
        legal, and if so, it's energy. If noChange = true, it will re-set the world situation to what it was on entry.
        if not, it will leave the world in the state encapsulated in HandObjectState.
        if the HandObjectState is not legal, it will re-set the world before exiting regardless of the noChange flag.
    */
    virtual void analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange = true);

    //! Works the same way as analyzeState, but analyzes the hand as it is when the function is called.
    virtual void analyzeCurrentPosture(Hand *h, Body *o, bool &isLegal, double &stateEnergy, bool noChange = true);

    virtual ~SearchEnergy();

};

#endif
