
#ifndef _contactenergy_h_
#define _contactenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class ContactEnergy: public SearchEnergy
{
  public:
    double energy() const;
};


#endif
