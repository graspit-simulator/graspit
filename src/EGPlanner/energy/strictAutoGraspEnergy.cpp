#include "graspit/EGPlanner/energy/strictAutoGraspEnergy.h"

#include "graspit/robot.h"
#include "graspit/debug.h"
#include "graspit/grasp.h"
#include "graspit/quality/quality.h"

double StrictAutoGraspEnergy::energy() const
{
  //  double gq = autograspQualityEnergy();
  double gq = approachAutograspQualityEnergy();
  if (gq == 0) { return 1.0e8; }
  else { return gq; }
}


/*! This version moves the palm in the direction of the object, attempting to establish contact on the palm
    before closing the fingers and establishing contacts on the finger.
*/
double
StrictAutoGraspEnergy::approachAutograspQualityEnergy() const
{
  transf initialTran = mHand->getTran();
  bool contact = mHand->approachToContact(30);
  if (contact) {
    if (mHand->getPalm()->getNumContacts() == 0) { contact = false; }
  }
  if (!contact) {
    //if moving the hand in does not result in a palm contact, move out and grasp from the initial position
    //this allows us to obtain fingertip grasps if we want those
    DBGP("Approach found no contacts");
    mHand->setTran(initialTran);
  } else {
    DBGP("Approach results in contact");
  }
  return autograspQualityEnergy();
}


/*! This function simply closes the hand and computes the real grasp quality that results
*/
double
StrictAutoGraspEnergy::autograspQualityEnergy() const
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
