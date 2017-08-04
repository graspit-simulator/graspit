#include "graspit/EGPlanner/energy/compliantEnergy.h"
#include "graspit/profiling.h"
#include "graspit/debug.h"
#include "graspit/robot.h"
#include "graspit/world.h"
#include "graspit/math/matrix.h"
#include "graspit/grasp.h"
#include "graspit/quality/quality.h"

const double unbalancedForceThreshold = 1.0e10;

double CompliantEnergy::energy() const
{
  PROF_RESET(QS);
  PROF_TIMER_FUNC(QS);
  //approach the object until contact; go really far if needed
  mHand->findInitialContact(200);
  //check if we've actually touched the object
  if (!mHand->getNumContacts(mObject)) { return 1; }

  //close the hand, but do additional processing when each new contact happens
  mCompUnbalanced = false;
  mMaxUnbalancedForce.x() = 0.0;
  mMaxUnbalancedForce.y() = 0.0;
  mMaxUnbalancedForce.z() = 0.0;

  QObject::connect(mHand, SIGNAL(moveDOFStepTaken(int, bool &)),
                   this, SLOT(autoGraspStep(int, bool &)));
  mHand->autoGrasp(!mDisableRendering, 1.0, false);
  QObject::disconnect(mHand, SIGNAL(moveDOFStepTaken(int, bool &)),
                      this, SLOT(autoGraspStep(int, bool &)));

  if (mCompUnbalanced || mMaxUnbalancedForce.norm() > unbalancedForceThreshold) {
    //the equivalent of an unstable grasp
  }

  //check if we've actually grasped the object
  if (mHand->getNumContacts(mObject) < 2) { return 1; }

  PRINT_STAT(mOut, "unbal: " << mMaxUnbalancedForce);

  //compute unbalanced force again. Is it zero?
  //but compute it for all the force that the dofs will apply
  //a big hack for now. It is questionable if the hand should even allow
  //this kind of intrusion into its dofs.
  for (int d = 0; d < mHand->getNumDOF(); d++) {
    mHand->getDOF(d)->setForce(mHand->getDOF(d)->getMaxForce());
  }
  mHand->getWorld()->resetDynamicWrenches();
  //passing true means the set dof force will be used in computations
  Matrix tau(mHand->staticJointTorques(true));
  int result = mHand->getGrasp()->computeQuasistaticForces(tau);
  if (result) {
    if (result > 0) {
      PRINT_STAT(mOut, "Final_unbalanced");
    } else {
      PRINT_STAT(mOut, "Final_ERROR");
    }
    return 1.0;
  }
  double *extWrench = static_cast<DynamicBody *>(mObject)->getExtWrenchAcc();
  vec3 force(extWrench[0], extWrench[1], extWrench[2]);
  vec3 torque(extWrench[3], extWrench[4], extWrench[5]);

  //perform traditional f-c check
  mHand->getGrasp()->collectContacts();
  mHand->getGrasp()->updateWrenchSpaces();
  double epsQual = mEpsQual->evaluate();
  PRINT_STAT(mOut, "eps: " << epsQual);

  if (epsQual < 0.05) { return 1.0; }

  PROF_PRINT(QS);
  PRINT_STAT(mOut, "torque: " << torque << " " << torque.norm());
  PRINT_STAT(mOut, "force: " << force << " " << force.norm());

  return -200.0 + force.norm();// + torque.norm();
}


void
CompliantEnergy::autoGraspStep(int numCols, bool &stopRequest) const
{
  //if no new contacts have been established, nothing new to compute
  stopRequest = false;
  if (!numCols) {
    return;
  }
  //if any of the kinematic chains is not balanced, early exit
  mHand->getWorld()->resetDynamicWrenches();
  //compute min forces to balance the system (pass it a "false")
  Matrix tau(mHand->staticJointTorques(false));
  int result = mHand->getGrasp()->computeQuasistaticForces(tau);
  if (result) {
    if (result > 0) {
      PRINT_STAT(mOut, "Unbalanced");
    } else {
      PRINT_STAT(mOut, "ERROR");
    }
    mCompUnbalanced = true;
    stopRequest = true;
    return;
  }
  assert(mObject->isDynamic());
  double *extWrench = static_cast<DynamicBody *>(mObject)->getExtWrenchAcc();
  vec3 force(extWrench[0], extWrench[1], extWrench[2]);
  if (force.norm() > mMaxUnbalancedForce.norm()) {
    mMaxUnbalancedForce = force;
  }
  //we could do an early exit here as well
}

