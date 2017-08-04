
#include "graspit/EGPlanner/energy/autoGraspQualityEnergy.h"
#include "graspit/quality/quality.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"

double AutoGraspQualityEnergy::energy() const
{
  DBGP("Autograsp quality computation");
  mHand->autoGrasp(false, 1.0);
  mHand->getGrasp()->collectContacts();
  mHand->getGrasp()->updateWrenchSpaces();
  double volQual = mVolQual->evaluate();
  double epsQual = mEpsQual->evaluate();
  if (epsQual < 0) { epsQual = 0; } //returns -1 for non-FC grasps
  DBGP("Autograsp quality: " << volQual << " volume and " << epsQual << " epsilon.");
  return - (30 * volQual) - (100 * epsQual);
}
