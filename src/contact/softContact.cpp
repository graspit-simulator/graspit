#include "graspit/contact/softContact.h"
#include "graspit/contact/contact.h"
#include "graspit/debug.h"
#include "graspit/body.h"
#include "graspit/FitParabola.h"
#include "graspit/mytools.h"

#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

/*! Does not set up friction edges yet; we need to wait until the mate of
    this contact is also defined because we will need its own analytical
    surface before we can come up with friction edges.
*/
SoftContact::SoftContact(Body *b1, Body *b2, position pos, vec3 norm,
                         Neighborhood *bn) : Contact(b1, b2, pos, norm)
{
  frictionType = SFCL;
  contactDim = 4;
  lmiDim = 4;
  optimalCoeffs = new double[contactDim];

  bodyNghbd =  new vec3[(int) bn->size() ];
  Neighborhood::iterator itr;
  int i = 0;
  vec3 temp;

  for (itr = bn->begin(); itr != bn->end(); itr++) {
    temp.x() = itr->x();
    temp.y() = itr->y();
    temp.z() = itr->z();
    //places bodyNghbd in frame of contact with the z-axis pointing out
    //bodyNghbd[i] = frame.affine().inverse() * ( temp - frame.translation() );
    position posit;
    posit = frame.inverse().affine() * (temp) ;
    bodyNghbd[i] = posit;
    i++;
  }
  numPts = (int) bn->size();
  majorAxis = 0.0;
  minorAxis = 0.0;
  relPhi = 0.0;
  a = 0; b = 0; c = 0;
  r1 = 0; r2 = 0;
  r1prime = 0; r2prime = 0;

  FitPoints();

  //setUpFrictionEdges();
  //wait to set these up until other contact is made
}

SoftContact::~SoftContact()
{
  delete[]bodyNghbd;
}

/*! Sets up friction edges as a 3D friction ellipsoid. All the computations for
    fitting analytical surfaces to the two bodies should already have been
    completed.
*/
int SoftContact::setUpFrictionEdges(bool dynamicsOn)
{
  if (!getMate()) {
    fprintf(stderr, "Trying to set up friction edges for a contact with no mate!!\n");
    return 1;
  }

  DBGP("Setting up SOFT contact friction edges");
  double eccen[3];

  // magnitude of tangential friction
  eccen[0] = 1;
  eccen[1] = 1;
  // magnitude of max frictional torque
  double torquedivN;

  // relative radii of curvature at the contact
  CalcRprimes();

  if (!dynamicsOn) {
    // if dynamics are not on, compute it based on some random applied force
    torquedivN = CalcContact_Mattress(5);
    eccen[2] = torquedivN * 1000;
  } else {
    // if dynamics are on, compute it based on the applied force reported by the LCP
    double nForce = dynamicForce[2];
    //need to figure out what all the numbers in the dynamic force mean
    //I think it is a wrench, and hope it is in the contacts coordinate frame
    //and that the units are in newtons, chekc dynamics.cpp iterateDynamics where
    //it is created
    torquedivN = CalcContact_Mattress(nForce);
    eccen[2] = torquedivN * 1000;
  }

  //various possible approximations for the friction ellipsoid

  /*
  int numDirs[9] = {1,3,5,7,8,7,5,3,1};
  double phi[9] = {M_PI_2, M_PI_2*0.6, M_PI_2*0.3, M_PI_2*0.15, 0.0,
                   -M_PI_2*0.15, -M_PI_2*0.3, -M_PI_2*0.6, -M_PI_2};
  return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
  */

  /*
  int numDirs[9] = {1,8,8,8,8,8,8,8,1};
  double phi[9] = {M_PI_2, M_PI_2*0.66, M_PI_2*0.33, M_PI_2*0.165, 0.0,
                -M_PI_2*0.165, -M_PI_2*0.33, -M_PI_2*0.66, -M_PI_2};
  return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
  */

  int numDirs[5] = {1, 5, 8, 5, 1};
  double phi[5] = {M_PI_2, M_PI_2 * 0.50, 0.0, -M_PI_2 * 0.50, -M_PI_2};
  return Contact::setUpFrictionEllipsoid(5, numDirs, phi, eccen);
}

/*! The Soft Contact version of computeWrenches can also take into
    account the fact that a soft contact can generate some moments in
    the plane of the contacts. It creates wrenches like the regular
    contact, but multiple times, moving the location of the force around
    the area of contact. Currently disabled.
 */
void SoftContact::computeWrenches()
{
  bool approxPlaneMoments = false;

  if (!approxPlaneMoments) {
    Contact::computeWrenches();
    return;
  }

  //for now we hard-code 4 possible locations for the contact
  //on 4 points around the perimeter of the ellipse

  //we also approximate the ellipse with a circle of radius sqrt(majorAxis * minorAxis)
  //because for now we don't really compute the direction of the major axis

  //when we do, we will have to transform the tangents by:
  // - the transform that aligns it with the major radius of the paraboloid
  // - the transform that further moves that to the major axis of the contact ellipse

  if (wrench) { delete [] wrench; }
  numFCWrenches = 4 * numFrictionEdges;
  wrench = new Wrench[numFCWrenches];

  vec3 tangentX = frame.affine().col(0);
  vec3 tangentY = frame.affine().col(1);

  vec3 radius;
  vec3 baseRadius = loc - ((GraspableBody *)body1)->getCoG();

  float a, b;
  a = b = 1000 * sqrt(majorAxis * minorAxis); //also convert to milimeters
  a = b = 1000 * std::max(majorAxis, minorAxis);

  DBGA("Soft contact size: " << a);

  for (int i = 0; i < numFrictionEdges; i++) {
    radius = baseRadius + (a * tangentX);
    wrenchFromFrictionEdge(&frictionEdges[6 * i], radius, &wrench[4 * i + 0]);

    radius = baseRadius - (a * tangentX);
    wrenchFromFrictionEdge(&frictionEdges[6 * i], radius, &wrench[4 * i + 1]);

    radius = baseRadius + (b * tangentY);
    wrenchFromFrictionEdge(&frictionEdges[6 * i], radius, &wrench[4 * i + 2]);

    radius = baseRadius - (b * tangentY);
    wrenchFromFrictionEdge(&frictionEdges[6 * i], radius, &wrench[4 * i + 3]);
  }
  DBGA("Soft wrenches computed");
}

/*! Computes an analytical surface of the form ax^2 + bx + c in a small patch
    around the contact on body1. The fit is in the local body1 coordinate
    system.
*/
void SoftContact::FitPoints()
{
  double *coeffs = new double [3];
  FitParaboloid(bodyNghbd, numPts, coeffs);

  a = coeffs[0];
  b = coeffs[1];
  c = coeffs[2];

  RotateParaboloid(coeffs, &r1, &r2, &fitRot, &fitRotAngle);
  DBGP(getBody1()->getName().latin1() << ": " << "a=" << a << " b=" << b << " c=" << c);
  DBGP("r1=" << r1 << " r2=" << r2);
}

/*! Calculates the angle betwen the main radius curvature on body1 and the
    main radius of curvature on body 2.
*/
int SoftContact::CalcRelPhi()
{
  vec3 temp, t;
  vec3 R11, R12;    //directions of relative curvatures in world frame

  temp.x() = 1;
  temp.y() = 0.0;
  temp.z() = 0.0;
  t = fitRot * temp;
  R11 = frame.affine() * (t);
  R11 = body1->getTran().affine() * (R11);

  temp.x() = 1;
  temp.y() = 0.0;
  temp.z() = 0.0;
  t = ((SoftContact *)getMate())->fitRot * temp;
  R12 = ((SoftContact *)getMate())->frame.affine() * (t);
  R12 = body2->getTran().affine() * (R12);

  relPhi = acos((R11.dot(R12)) / (R11.norm() * R12.norm()));
  ((SoftContact *)getMate())->relPhi = relPhi ;

  //fprintf( stderr, "Rel Phi   %f", relPhi );
  return 0;
}

/*! Calculates the relative radii of curvature of the contact, using the
    radii of curvature of both bodies and the angle between them.
    See Johnson, Contact Mechanics Chapter 4 page 85 for calculations.
*/
int SoftContact::CalcRprimes()
{
  if (!(getMate())) {
    DBGA("Contact doesn't have mate, not calculating curvature...");
    return 1;
  }

  SoftContact *m = (SoftContact *)getMate();

  CalcRelPhi();

  DBGP("Body 1" << getBody1()->getName().latin1() << " Body 2 " << m->getBody1()->getName().latin1());

  //the less than zero curvature is for flat objects
  //1/r goes to zero for flat objects because the curvature
  //is infinite
  double x, y, w, z;
  if (r1 < 0.0) {
    w = 0.0;
  }
  else {
    w = 1 / r1;
  }

  if (r2 < 0.0) {
    x = 0.0;
  }
  else {
    x = 1 / r2;
  }

  if (m->r1 < 0.0) {
    y = 0.0;
  }
  else {
    y = 1 / m->r1;
  }

  if (m->r2 < 0.0) {
    z = 0.0;
  }
  else {
    z = 1 / m->r2;
  }

  DBGP("x: " << x << " y: " << y << " w: " << w << " z: " << z);

  double ApB = 0.5 * (w + x + y + z);

  DBGP("Apb: " << ApB);

  double AmB = 0.5 * sqrt((w - x) * (w - x) + (y - z) * (y - z) +
                          2 * cos(2 * relPhi) * (w - x) * (y - z));

  DBGP("Amb: " << AmB << " relPhi: " << relPhi);

  if (AmB > ApB) {
    printf("Invalid relative curvature, ending calculation...\n");
    return 1;
  }

  //by definition r1prime >= r2prime
  if (ApB + AmB != 0) {
    r1prime = 1 / (ApB + AmB);
  }
  else {
    r1prime = -1.0;
  }

  if (ApB == AmB) {
    r2prime = -1.0;
  }
  else {
    r2prime = 1 / (ApB - AmB);
  }

  m->r1prime = r1prime;
  m->r2prime = r2prime;

  //fprintf(stderr,"original: %f %f and %f %f\n",r1,r2,m->r1, m->r2);
  //fprintf(stderr,"Relative radii: %f %f equiv: %f\n",r1prime, r2prime, sqrt(r1prime*r2prime) );
  return 0;
}

/*! Calculates the axes of the ellipse of contact using the mattress
    model and a given normal force (for dynamics its the normal component
    of the dynamic contact force). Sets major axis and minor axis.
    Returns the max torque available.
    See Contact Mechanics, KL Johnson Chapter 4
*/
double SoftContact::CalcContact_Mattress(double nForce)
{
  if (r1prime < 0) {
    DBGP("Degenerate soft contact");
    r1prime = 20;
  }
  if (r2prime < 0) {
    DBGP("Degenerate soft contact");
    r2prime = 20;
  }

  //hardwired height of mattress = 3.0 mm
  //r primes are in mm
  double h = 0.003;
  double delta = sqrt(nForce * h / (MAX(body1->getYoungs(), body2->getYoungs())
                                    * M_PI * sqrt(r1prime * 0.001 * r2prime * 0.001))); //meters

  majorAxis = sqrt(2 * delta * r1prime * 0.001); //meters
  minorAxis = sqrt(2 * delta * r2prime * 0.001);
  DBGP("Axes: " << majorAxis << " " << minorAxis);
  //axes are given in meters!!!!
  //rprimes and the radii of curvature are calculated in mm

  SoftContact *m = (SoftContact *)getMate();
  m->majorAxis = majorAxis;
  m->minorAxis = minorAxis;

  //returns max available torque proportional to given normal force
  //max torque=*K/h*mu*4*pi*(ab)^1.5/15 where a and b are
  //the axis of the contact ellipse
  //this just returns the max torque divided by the normal force times mu
  //which is 8sqrt(ab)/15
  //the pressure distrib is K*delta/h/2*pi*ab

  //std::cerr<<"Ma ma "<< majorAxis << minorAxis << "\n";

  return 8 * sqrt(majorAxis * minorAxis) / 15; //in meters
}

/*! Gets the visual indicator as a small patch of the fit analytical surface
    around the body. Also places a small arrow indicating the direction of
    the main radius of curvature computed for the body.
*/
SoSeparator *SoftContact::getVisualIndicator()
{
  double height;
  SoSeparator *cne;
  SoTransform *tran;
  SoCoordinate3 *coords;
  cne = new SoSeparator;

  int sampling_n = 10, sampling_m = 10;
  //double sampleSize_n = 2, sampleSize_m = 1;
  //double sampleSize_n = (4.0 * majorAxis * 1000) / (2.0 * sampling_n);
  //double sampleSize_m = (4.0 * minorAxis * 1000) / (2.0 * sampling_m);

  double sampleSize_n = 0.7;
  double sampleSize_m = 0.7;


  DBGP("Major " << majorAxis << " minor " << minorAxis);

  //points runs left to right and then up to down in the grid
  SbVec3f *points = new SbVec3f[(2 * sampling_n + 1) * (2 * sampling_m + 1) ];

  coneMat = new SoMaterial;
  coneMat->diffuseColor = SbColor(0.8f, 0.0f, 0.0f);
  coneMat->ambientColor = SbColor(0.2f, 0.0f, 0.0f);
  coneMat->emissiveColor = SbColor(0.4f, 0.0f, 0.0f);
  coneMat->transparency = 0.8f;

  SoMaterial *zaxisMat = new SoMaterial;
  zaxisMat->diffuseColor = SbColor(0, 0, 0);
  zaxisMat->ambientColor = SbColor(0, 0, 0);

  height = Body::CONE_HEIGHT;

  tran = new SoTransform;
  getContactFrame().toSoTransform(tran);

  SoCylinder *zaxisCyl = new SoCylinder;
  zaxisCyl->radius = 0.05f;
  zaxisCyl->height = 0.2 * height;

  SoTransform *zaxisTran = new SoTransform;
  zaxisTran->translation.setValue(0, 0, 0.2 * height / 2.0);
  zaxisTran->rotation.setValue(SbVec3f(1, 0, 0), (float)M_PI / 2.0f);

  SoSeparator *zaxisSep = new SoSeparator;
  zaxisSep->addChild(zaxisTran);
  zaxisSep->addChild(zaxisMat);
  zaxisSep->addChild(zaxisCyl);

  int n_squares = 2 * sampling_n * 2 * sampling_m;
  int32_t *cIndex = new int32_t[5 * n_squares];

  int count = 0;
  int count_sq = 0;
  for (int i = -sampling_n; i <= sampling_n; i++)
  {
    for (int j = -sampling_m; j <= sampling_m; j++)
    {
      double x = i * sampleSize_n;
      double y = j * sampleSize_m;
      //double z = a*x*x + b*y*y + c*x*y;
      double z = 0;
      if (r1 > 0) { z += x * x * 1.0 / (2 * r1); }
      if (r2 > 0) { z += y * y * 1.0 / (2 * r2); }
      points[ count ].setValue(x, y, z);
      if (x > 0 && y == 0) {
        points[count].setValue(x, y, z + 0.4);
      }

      if (i > -sampling_n && j > -sampling_m)
      {
        cIndex[5 * count_sq + 0] = count - (2 * sampling_m + 1);
        cIndex[5 * count_sq + 1] = count - (2 * sampling_m + 1) - 1;
        cIndex[5 * count_sq + 2] = count - 1;
        cIndex[5 * count_sq + 3] = count;
        cIndex[5 * count_sq + 4] = -1;
        count_sq++;
      }
      count++;
    }
  }
  if (count != (2 * sampling_n + 1) * (2 * sampling_m + 1) || count_sq != n_squares) {
    exit(0);
  }

  coords = new SoCoordinate3;
  coords->point.setValues(0, count, points);

  SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
  ifs->coordIndex.setValues(0, 5 * n_squares, cIndex);

  //neighborhood is in frame of contact, so add contact transform first
  cne->addChild(tran);
  /*
      SoMaterial* sphereMat = new SoMaterial;
      sphereMat->diffuseColor = SbColor(1,1,0);
      sphereMat->ambientColor = SbColor(1,1,0);
      cne->addChild(sphereMat);
      for (int i=0; i<numPts; i++)
      {
          SoSphere* sphere = new SoSphere;
          sphere->radius = 0.5;
          SoTransform* sphereTran = new SoTransform;
          sphereTran->translation.setValue( bodyNghbd[i].x(), bodyNghbd[i].y(), bodyNghbd[i].z() );
          SoSeparator* sep = new SoSeparator;
          sep->addChild(sphereTran);
          sep->addChild(sphere);
          cne->addChild(sep);
      }
  */

  cne->addChild(zaxisSep);

  //this angle gets us from the contact frame to the frame in which the paraboloid is defined by just
  //two parameters, r1 and r2
  SoTransform *axisAlignTran = new SoTransform;
  axisAlignTran->rotation.setValue(SbVec3f(0, 0, 1), -fitRotAngle);
  cne->addChild(axisAlignTran);

  cne->addChild(coneMat);
  cne->addChild(coords);
  cne->addChild(ifs);

  //add an arrow in the direction of the major axis of the contact ellipse
  //this is the angle between the major axis of the two paraboloids. it is used to compute the
  //magnitude of the major axis of the contact ellipse
  //but for now we don't compute the direction of the contact ellipse!
  SoTransform *phiAlignTran = new SoTransform;
  phiAlignTran->rotation.setValue(SbVec3f(0, 0, 1), relPhi);
  cne->addChild(phiAlignTran);

  SoCylinder *xaxisCyl = new SoCylinder;
  xaxisCyl->radius = 0.05f;
  xaxisCyl->height = height / 2.0;

  SoTransform *xaxisTran = new SoTransform;
  xaxisTran->translation.setValue(height / 4.0, 0.0, 0.0);
  xaxisTran->rotation.setValue(SbVec3f(0, 0, 1), -(float)M_PI / 2.0f);

  SoSeparator *xaxisSep = new SoSeparator;
  xaxisSep->addChild(xaxisTran);
  xaxisSep->addChild(zaxisMat);
  xaxisSep->addChild(xaxisCyl);

  //cne->addChild(xaxisSep);
  return cne;
}
