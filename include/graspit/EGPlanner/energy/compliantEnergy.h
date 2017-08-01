
#ifndef _complientenergy_h_
#define _complientenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"

class CompliantEnergy: public SearchEnergy
{
  public:
    double energy() const;

  protected:
    mutable bool mCompUnbalanced;
    mutable vec3 mMaxUnbalancedForce;

  private Q_SLOTS:
    //! Called to compute compliant force balances during autograsp
    void autoGraspStep(int numCols, bool &stopRequest) const;
};


#endif
