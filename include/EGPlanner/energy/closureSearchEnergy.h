
#ifndef _closureenergy_h_
#define _closureenergy_h_

#include "include/EGPlanner/energy/searchEnergy.h"

/*! This class is meant to be used with the GuidedPlanner that looks for force-closure. It adds one main thing:
    - can use a list of "avoid states", so that a new state is deemed illegal if it is in the vicintiy of one of these
*/

class ClosureSearchEnergy : public SearchEnergy
{
protected:
    const std::list<GraspPlanningState*> *mAvoidList;
    double mThreshold;
public:
    ClosureSearchEnergy() : SearchEnergy(), mAvoidList(NULL), mThreshold(0.3) {}

    void setThreshold(double t){mThreshold=t;}
    void setAvoidList(const std::list<GraspPlanningState*> *l){mAvoidList = l;}
    void analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange = true);
    double energy() const;

};


#endif
