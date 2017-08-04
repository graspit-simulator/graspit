

#ifndef _potentialenergy_h_
#define _potentialenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class PotentialQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;

  protected:
    double potentialQualityScalingFunction(double dist, double cosTheta)const ;

};


#endif
