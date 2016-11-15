#ifndef _guidedautograspenergy_h_
#define _guidedautograspenergy_h_

#include "EGPlanner/energy/searchEnergy.h"

class GuidedAutoGraspQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;

};


#endif
