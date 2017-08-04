
#ifndef _strictautograspenergy_h_
#define _strictautograspenergy_h_

#include "EGPlanner/energy/searchEnergy.h"

class StrictAutoGraspEnergy: public SearchEnergy
{
  public:
    double energy() const;

  protected:
    double approachAutograspQualityEnergy() const;
    double autograspQualityEnergy() const;
};


#endif
