#include "graspit/EGPlanner/energy/dynamicAutoGraspEnergy.h"

#include "graspit/robot.h"
#include "graspit/world.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/quality/quality.h"

double
DynamicAutoGraspEnergy::energy() const
{
  //approach the object until contact; go really far if needed
  mHand->findInitialContact(200);
  //check if we've actually touched the object
  if (!mHand->getNumContacts(mObject)) { return 1; }

  //this is more of a hack to cause the hand to autograsp in dynamics way
  //the world's callback for dynamics should never get called as everything
  //gets done from inside here
  mHand->getWorld()->resetDynamics();
  //I think this happens in each dynamics step anyway
  mHand->getWorld()->resetDynamicWrenches();
  mHand->getWorld()->turnOnDynamics();
  //this should set the desired values of the dof's
  //also close slowly, so we are closer to pseudo-static conditions
  mHand->autoGrasp(false, 0.5);

  QObject::connect(mHand->getWorld(), SIGNAL(dynamicsError(const char *)),
                   this, SLOT(dynamicsError(const char *)));
  //loop until dynamics is done
  mDynamicsError = false;
  int steps = 0; int stepFailsafe = 1500;
  int autoGraspDone = 0; int afterSteps = 100;
  //first close fingers to contact with object
  while (1) {
    if (!(steps % 100)) {DBGA("Step " << steps);}
    mHand->getWorld()->stepDynamics();
    if (mDynamicsError) { break; }
    //see if autograsp is done
    if (!autoGraspDone && dynamicAutograspComplete()) {
      autoGraspDone = steps;
    }
    //do some more steps after sutograsp
    if (autoGraspDone && steps - autoGraspDone > afterSteps) {
      break;
    }
    //and finally check stepfailsafe
    if (++steps > stepFailsafe) { break; }
  }

  if (mDynamicsError) {
    PRINT_STAT(mOut, "Dynamics error");
    return 2.0;
  } else if (steps > stepFailsafe) {
    PRINT_STAT(mOut, "Time failsafe");
    return 2.0;
  }
  PRINT_STAT(mOut, "Autograsp done");

  //disable contacts on pedestal
  Body *obstacle = NULL;
  for (int b = 0; b < mHand->getWorld()->getNumBodies(); b++) {
    Body *bod = mHand->getWorld()->getBody(b);
    if (bod->isDynamic()) { continue; }
    if (bod->getOwner() != bod) { continue; }
    obstacle = bod;
    break;
  }
  if (!obstacle) {
    PRINT_STAT(mOut, "Obstacle not found!");
    return 2.0;
  }
  mHand->getWorld()->toggleCollisions(false, obstacle);
  //and do some more steps
  steps = 0; afterSteps = 400;
  while (1) {
    if (!(steps % 100)) {DBGA("After step " << steps);}
    mHand->getWorld()->stepDynamics();
    /*
    if (mHand->getDOF(0)->getForce() + 1.0e3 > mHand->getDOF(0)->getMaxForce()) {
        DBGA("Max force applied");
        break;
    }*/
    if (mDynamicsError) { break; }
    if (++steps > afterSteps) { break; }
  }
  mHand->getWorld()->toggleCollisions(true, obstacle);
  QObject::disconnect(mHand->getWorld(), SIGNAL(dynamicsError(const char *)),
                      this, SLOT(dynamicsError(const char *)));
  //turn off dynamics; world dynamics on shouldn't have done anything anyway
  mHand->getWorld()->turnOffDynamics();

  if (mDynamicsError) {
    PRINT_STAT(mOut, "Dynamics error");
    return 2.0;
  }

  //if the object has been ejected output error
  if (mHand->getNumContacts(mObject) < 2) {
    PRINT_STAT(mOut, "Ejected");
    return 0.0;
  }

  //perform traditional f-c check
  mHand->getGrasp()->collectContacts();
  mHand->getGrasp()->updateWrenchSpaces();
  double epsQual = mEpsQual->evaluate();
  PRINT_STAT(mOut, "eps: " << epsQual);

  if (epsQual < 0.05) { return -0.5; }

  //grasp has finished in contact with the object
  PRINT_STAT(mOut, "Success 1");
  return -1.0;
}

/*! Implements the following heuristic: for each chain, either the last link
    must have a contact, or the last joint must be maxed out. If this is true
    the dynamic autograsp *might* have been completed
*/
bool DynamicAutoGraspEnergy::dynamicAutograspComplete() const
{
  return mHand->dynamicAutograspComplete();
}

bool DynamicAutoGraspEnergy::contactSlip() const
{
  return mHand->contactSlip();
}

