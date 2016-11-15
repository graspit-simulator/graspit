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

#include <QObject>
#include "search.h"

class GraspPlanningState;
class VariableSet;
class SearchEnergy;
class Hand;
class Body;

class SoSensor;

#include <stdio.h>
#include <matvec3D.h>
#include <vector>

/*!	This class performs simulated annealing on a collection of variables 
	(a GraspPlanningState*). It has no idea of grasps, hands, etc. The cooling 
	schedule is inspired from L. Ingber, "Very Fast Simulated Re-Annealing", 
	J. Mathl. Comput. Modelling, vol. 12, no. 8, pp. 967–973, December 1989.
*/
class SimAnn : public QObject
{
	Q_OBJECT
public:
	//! Possible return codes for an annealing step
	/*! <ul>
		<li> FAIL - no legal neighbor of the current state was found
		<li> KEEP - a legal neighbor was found, but the annealing procedure
				decided to keep the current state
		<li> JUMP - a jump was performed to a newly generated neighbor
		</ul>
	*/
	enum Result{FAIL = 0, JUMP = 1, KEEP =2};
private:

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

	//! The current step index, used by annealing schedule
	long int mCurrentStep;
	//! The temperature at the beginning of the annealing process
	double mT0;

	//! Total steps since last reset (saved over re-anneals)	
	int mTotalSteps; 
	//! For writing results to file
	bool mWriteResults;
	//! For writing results to file
	FILE* mFile;

	double prob(double e_old, double e_new, double t);
	double cooling(double t0, double c, int k, int d);
	//! Computes a neighbor of a given HandObjectState
	GraspPlanningState* stateNeighbor(GraspPlanningState *s, double T, GraspPlanningState *t = NULL);
	//! Computes the neighbor of one individual variable
	void variableNeighbor(VariableSet *set, double T, VariableSet *target = NULL);
	double neighborDistribution(double T);
	double neighborInverse(double T, double y);
	//! Generates neighbors according to a desired value, or "input", along each dimension
	double biasedNeighborDistribution(double T, double in, double conf);

public:
	SimAnn();
	~SimAnn();

	//! The main interface to this class. Performs one annealing step
	Result iterate(GraspPlanningState *currentState, SearchEnergy *energyCalculator, GraspPlanningState *targetState = NULL);

	void reset();
	int getCurrentStep(){return mCurrentStep;}
	void setParameters(AnnealingType type);
	void writeResults(bool w);
};
