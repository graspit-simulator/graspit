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
// $Id: simAnn.h,v 1.23 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#ifndef _simannparams_h_
#define _simannparams_h_
class SimAnnParams
{

  public:

    static SimAnnParams ANNEAL_DEFAULT();
    static SimAnnParams ANNEAL_LOOP();
    static SimAnnParams ANNEAL_MODIFIED();
    static SimAnnParams ANNEAL_STRICT();
    static SimAnnParams ANNEAL_ONLINE();

    //Annealing parameters
    //! Annealing constant for neighbor generation schedule
    double YC;
    //! Annealing constant for error acceptance schedule
    double HC;
    //! Number of dimensions for neighbor generation schedule
    double YDIMS;
    //! Number of dimensions for error acceptance schedule
    double HDIMS;
    //! Adjust factor for neighbor generation schedule
    double NBR_ADJ;
    //! Adjust raw errors reported by states to be in the relevant range of the annealing schedule
    double ERR_ADJ;
    //! Starting temperatue
    double DEF_T0;
    //! Starting step
    double DEF_K0;

    virtual ~SimAnnParams(){};

};

#endif
