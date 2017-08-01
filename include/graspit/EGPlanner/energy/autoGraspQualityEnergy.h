
#ifndef _autograspqualityenergy_h_
#define _autograspqualityenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class AutoGraspQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;
};


#endif
