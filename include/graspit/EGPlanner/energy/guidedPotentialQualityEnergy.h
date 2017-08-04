#ifndef _guidedpotentialenergy_h_
#define _guidedpotentialenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class GuidedPotentialQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;

  protected:
    double potentialQualityEnergy() const;
    double contactEnergy() const;
    double potentialQualityScalingFunction(double dist, double cosTheta)const ;


};


#endif
