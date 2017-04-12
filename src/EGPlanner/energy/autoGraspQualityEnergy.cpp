
#include "EGPlanner/energy/autoGraspQualityEnergy.h"
#include "quality.h"
#include "robot.h"
#include "grasp.h"
#include "debug.h"
#include "world.h"

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
