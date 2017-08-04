
#include "graspit/EGPlanner/energy/contactEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/contact/virtualContact.h"

double ContactEnergy::energy() const
{
  /*
  this is if we don't want to use pre-specified contacts, but rather the closest points between each link
  and the object all iterations. Not necessarily needed for contact energy, but useful for GQ energy
  */
  if (mContactType == CONTACT_LIVE && mType != "AUTO_GRASP_QUALITY_ENERGY" && mType != "STRICT_AUTO_GRASP_ENERGY")
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
    double dist = p.norm();

    //this should never happen anymore since we're never inside the object
    //if ( (-1.0 * p) % n < 0) dist = -dist;

    //BEST WORKING VERSION, strangely enough
    totalError += fabs(dist);

    //let's try this some more
    //totalError += distanceFunction(dist);
    //cn = -1.0 * contact->getWorldNormal();

    //new version
    cn = contact->getWorldNormal();
    n = p.normalized();
    double d = 1 - cn.dot(n);
    totalError += d * 100.0 / 2.0;
  }

  totalError /= mHand->getGrasp()->getNumContacts();

  //DBGP("Contact energy: " << totalError);
  return totalError;
}
