#include "graspit/EGPlanner/energy/potentialQualityEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/quality/quality.h"
#include "graspit/contact/virtualContact.h"


double
PotentialQualityEnergy::energy() const
{
  mHand->getGrasp()->collectVirtualContacts();

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
PotentialQualityEnergy::potentialQualityScalingFunction(double dist, double cosTheta) const
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

