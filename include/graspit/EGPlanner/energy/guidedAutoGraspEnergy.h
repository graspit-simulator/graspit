#ifndef _guidedautograspenergy_h_
#define _guidedautograspenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class GuidedAutoGraspQualityEnergy: public SearchEnergy
{
  public:
    double energy() const;

};


#endif
