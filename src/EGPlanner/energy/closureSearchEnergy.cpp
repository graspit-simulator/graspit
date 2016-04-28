#include "include/EGPlanner/energy/closureSearchEnergy.h"

#include "debug.h"
#include "EGPlanner/searchState.h"

void
ClosureSearchEnergy::analyzeState(bool &isLegal, double &stateEnergy, const GraspPlanningState *state, bool noChange)
{
    //first we check if we are too close to any state in the avoid list
    if (mAvoidList) {
        std::list<GraspPlanningState*>::const_iterator it; int i=0;
        for (it = mAvoidList->begin(); it!=mAvoidList->end(); it++){
            if ( (*it)->distance(state) < mThreshold ) {
                isLegal = false;
                stateEnergy = 0.0;
                DBGP("State rejected; close to state " << i);
                return;
            }
            i++;
        }
    }
    //if not, we compute everything like we usually do
    SearchEnergy::analyzeState(isLegal, stateEnergy, state, noChange);
}



double ClosureSearchEnergy::energy() const
{
    return 20;
}
