#include "graspit/EGPlanner/energy/guidedAutoGraspEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/quality/quality.h"
#include "graspit/contact/virtualContact.h"


/*! This formulation combines virtual contact energy with autograsp energy. Virtual contact energy is used to "guide"
    initial stages of the search and to see if we should even bother computing autograsp quality. Autograsp is a couple
    of orders of magnitude higher and so should work very well with later stages of the sim ann search
*/
double
GuidedAutoGraspQualityEnergy::energy() const
{
  //first compute regular contact energy; also count how many links are "close" to the object
  VirtualContact *contact;
  vec3 p, n, cn;
  double virtualError = 0; int closeContacts = 0;

  //collect virtual contacts first
  mHand->getGrasp()->collectVirtualContacts();
  for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++) {

    contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
    contact->getObjectDistanceAndNormal(mObject, &p, &n);

    double dist = p.norm();
    if ((-1.0 * p).dot(n)  < 0) { dist = -dist; }

    //BEST WORKING VERSION, strangely enough
    virtualError += fabs(dist);
    cn = -1.0 * contact->getWorldNormal();
    double d = 1 - cn.dot(n);
    virtualError += d * 100.0 / 2.0;

    if (fabs(dist) < 20 && d < 0.3) { closeContacts++; }
  }

  virtualError /= mHand->getGrasp()->getNumContacts();

  //if more than 2 links are "close" go ahead and compute the true quality
  double volQuality = 0, epsQuality = 0;
  if (closeContacts >= 2) {
    mHand->autoGrasp(false, 1.0);
    //now collect the true contacts;
    mHand->getGrasp()->collectContacts();
    if (mHand->getGrasp()->getNumContacts() >= 4) {
      mHand->getGrasp()->updateWrenchSpaces();
      volQuality = mVolQual->evaluate();
      epsQuality = mEpsQual->evaluate();
      if (epsQuality < 0) { epsQuality = 0; } //QM returns -1 for non-FC grasps
    }

    DBGP("Virtual error " << virtualError << " and " << closeContacts << " close contacts.");
    DBGP("Volume quality: " << volQuality << " Epsilon quality: " << epsQuality);
  }

  //now add the two such that the true quality is a couple of orders of magn. bigger than virtual quality
  double q;
  if (volQuality == 0) { q = virtualError; }
  else { q = virtualError - volQuality * 1.0e3; }
  if (volQuality || epsQuality) {DBGP("Final quality: " << q);}

  //DBGP("Final value: " << q << std::endl);
  return q;
}

