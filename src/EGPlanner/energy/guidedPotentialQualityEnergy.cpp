#include "graspit/EGPlanner/energy/guidedPotentialQualityEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/quality/quality.h"
#include "graspit/contact/virtualContact.h"


double
GuidedPotentialQualityEnergy::energy() const
{
  mHand->getGrasp()->collectVirtualContacts();

  double potentialEnergy = potentialQualityEnergy();

  if (potentialEnergy > 0.0)
  {
    return contactEnergy();
  }

  return potentialEnergy;
}



double GuidedPotentialQualityEnergy::contactEnergy() const
{

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

double GuidedPotentialQualityEnergy::potentialQualityEnergy() const
{
  bool verbose = false;
  VirtualContact *contact;
  vec3 p, n, cn;
  int count = 0;
  //DBGP("Potential quality energy computation")
  for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++)
  {
    contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
    contact->computeWrenches(true, false);
    contact->getObjectDistanceAndNormal(mObject, &p, NULL);
    n = contact->getWorldNormal();
    double dist = p.norm();
    p = p.normalized();
    double cosTheta = n.dot(p);
    double factor = potentialQualityScalingFunction(dist, cosTheta);
    if (verbose)
    {
      fprintf(stderr, "VC %d on finger %d link %d\n", i, contact->getFingerNum(), contact->getLinkNum());
      fprintf(stderr, "Distance %f cosTheta %f\n", dist, cosTheta);
      fprintf(stderr, "Scaling factor %f\n\n", factor);
    }
    contact->scaleWrenches(factor);
    if (factor > 0.25)
    {
      count++;
      contact->mark(true);
    } else { contact->mark(false); }
  }
  double gq = -1;
  //to make computations more efficient, we only use a 3D approximation
  //of the 6D wrench space
  std::vector<int> forceDimensions(6, 0);
  forceDimensions[0] = forceDimensions[1] = forceDimensions[2] = 1;
  if (count >= 3) {
    mHand->getGrasp()->updateWrenchSpaces(forceDimensions);
    gq = mEpsQual->evaluate();
  }
  if (verbose) {
    fprintf(stderr, "Quality: %f\n\n", gq);
  }
  if (count) {
    DBGP("Count: " << count << "; Gq: " << gq << ";");
  }
  return -gq;
  return 10;
}

double
GuidedPotentialQualityEnergy::potentialQualityScalingFunction(double dist, double cosTheta) const
{
  double sf = 0;
  /*
  (void*)&cosTheta;
  if (dist<0) return 0;
  sf += 100.0 / ( pow(2.0,dist/15.0) );
  //comment this out if you don't care about normals
  //sf += 100 - (1 - cosTheta) * 100.0 / 2.0;
  */
  /*
  if (cosTheta < 0.8) return 0; //about 35 degrees
  sf = 10.0 / ( pow((double)3.0,dist/25.0) );
  if (sf < 0.25) sf = 0; //cut down on computation for tiny values. more than 50mm away
  return sf;
  */
  /*
  if (cosTheta < 0.8) return 0; //about 35 degrees
  if (dist < 20) sf = 1;
  else {
  sf = 1.5 - 1.5 / ( pow((double)3.0,(dist-20)/25.0) );
  sf = cos( sf*sf ) + 1;
  }
  sf = sf*10;
  if (sf < 0.25) sf = 0; //cut down on computation for tiny values. more than 50mm away
  return sf;
  */
  if (cosTheta < 0.7) { return 0; }
  if (dist > 50) { return 0; }
  sf = cos(3.14 * dist / 50.0) + 1;
  return sf;
}

