#ifndef _dynamicautograspenergy_h_
#define _dynamicautograspenergy_h_

#include "EGPlanner/energy/searchEnergy.h"

class DynamicAutoGraspEnergy: public SearchEnergy
{
  public:
    double energy() const;
  protected:
    mutable bool mDynamicsError;

    //! returns true if any contacts are slipping
    bool contactSlip() const;

    bool dynamicAutograspComplete() const;
};


#endif
