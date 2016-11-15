
#include "EGPlanner/energy/contactEnergy.h"
#include "robot.h"
#include "grasp.h"
#include "debug.h"
#include "world.h"
#include "contact/virtualContact.h"

double ContactEnergy::energy() const
{
  /*
  this is if we don't want to use pre-specified contacts, but rather the closest points between each link
  and the object all iterations. Not necessarily needed for contact energy, but useful for GQ energy
  */
  if (mContactType == CONTACT_LIVE && mType != ENERGY_AUTOGRASP_QUALITY && mType != ENERGY_STRICT_AUTOGRASP)
  {
    mHand->getWorld()->findVirtualContacts(mHand, mObject);
    DBGP("Live contacts computation");
  }

  mHand->getGrasp()->collectVirtualContacts();

  //DBGP("Contact energy computation")
  //average error per contact
  VirtualContact *contact;
  vec3 p, n, cn;
  double totalError = 0;
  for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++)
  {
    contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
    contact->getObjectDistanceAndNormal(mObject, &p, NULL);
    double dist = p.len();

    //this should never happen anymore since we're never inside the object
    //if ( (-1.0 * p) % n < 0) dist = -dist;

    //BEST WORKING VERSION, strangely enough
    totalError += fabs(dist);

    //let's try this some more
    //totalError += distanceFunction(dist);
    //cn = -1.0 * contact->getWorldNormal();

    //new version
    cn = contact->getWorldNormal();
    n = normalise(p);
    double d = 1 - cn % n;
    totalError += d * 100.0 / 2.0;
  }

  totalError /= mHand->getGrasp()->getNumContacts();

  //DBGP("Contact energy: " << totalError);
  return totalError;
}
