
#include "graspit/contact/pointContact.h"
#include "graspit/contact/contact.h"
#include "graspit/body.h"

#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

PointContact::PointContact(Body *b1, Body *b2, position pos, vec3 norm) :
  Contact(b1, b2, pos, norm)
{
  //we should really have another class for the FL contact
  if (sCof == 0.0) {
    frictionType = FL;
    contactDim = 1;
    lmiDim = 1;
  } else {
    frictionType = PCWF;
    contactDim = 3;
    lmiDim = 3;
  }
  optimalCoeffs = new double[contactDim];
  setUpFrictionEdges();
}

PointContact::~PointContact()
{
}

/*! Set up friction as a linearized circle; a PCWF can only have friction
    forces (no torques) in the tangential plane of the contact. When normal
    force will be added later, the friction circle becomes the more
    familiar contact cone.
*/
int PointContact::setUpFrictionEdges(bool dynamicsOn)
{

  if (dynamicsOn) {
    //we don't really need to update anything; friction edges for point contacts do not change
    //during dynamic simulation
    return 0;
  }
  double eccen[3] = {1, 1, 1};
  int numDirs[1] = {8};
  double phi[1] = {0.0};
  return setUpFrictionEllipsoid(1, numDirs, phi, eccen);
}

/*! Return a visual indicator showing the contact cone */
SoSeparator *
PointContact::getVisualIndicator()
{
  double height, alpha, cof;
  SoSeparator *cne;
  SoTransform *tran;
  SoIndexedFaceSet *ifs;
  SoCoordinate3 *coords;

  SbVec3f *points = new SbVec3f[numFrictionEdges + 1];
  int32_t *cIndex = new int32_t[5 * numFrictionEdges + 1];
  int i;

  points[0].setValue(0, 0, 0);

  //this is now a member of the class so no need to declare it here
  coneMat = new SoMaterial;

  coneMat->diffuseColor = SbColor(0.8f, 0.0f, 0.0f);
  coneMat->ambientColor = SbColor(0.2f, 0.0f, 0.0f);
  coneMat->emissiveColor = SbColor(0.4f, 0.0f, 0.0f);
  /*
      coneR = ((float)rand())/RAND_MAX;
      coneG = ((float)rand())/RAND_MAX;
      coneB = ((float)rand())/RAND_MAX;

      coneMat->diffuseColor = SbColor(coneR, coneG, coneB);
      coneMat->ambientColor = SbColor(coneR, coneG, coneB);
      coneMat->emissiveColor = SbColor(coneR, coneG, coneB);
  */
  coneMat->transparency = 0.8f;

  SoMaterial *zaxisMat = new SoMaterial;
  zaxisMat->diffuseColor = SbColor(0, 0, 0);
  zaxisMat->ambientColor = SbColor(0, 0, 0);

  cof = getCof();

  height = Body::CONE_HEIGHT;
  cne = new SoSeparator;
  coords = new SoCoordinate3;
  ifs = new SoIndexedFaceSet;
  tran = new SoTransform;

  alpha = 0.0;
  for (i = 0; i < numFrictionEdges; i++) {
    points[i + 1].setValue(cos(alpha)*cof, sin(alpha)*cof, 1.0);
    points[i + 1] *= height;
    cIndex[4 * i] = 0;
    cIndex[4 * i + 1] = (i + 2 <= numFrictionEdges ? i + 2 : 1);
    cIndex[4 * i + 2] =  i + 1;
    cIndex[4 * i + 3] = -1;
    cIndex[4 * numFrictionEdges + i] = i + 1;
    alpha += 2.0 * M_PI / numFrictionEdges;
  }
  cIndex[5 * numFrictionEdges] = -1;

  coords->point.setValues(0, numFrictionEdges + 1, points);
  ifs->coordIndex.setValues(0, 5 * numFrictionEdges + 1, cIndex);
  delete [] points;
  delete [] cIndex;

  getContactFrame().toSoTransform(tran);

  SoCylinder *zaxisCyl = new SoCylinder;
  zaxisCyl->radius = 0.05f;
  zaxisCyl->height = height;

  SoTransform *zaxisTran = new SoTransform;
  zaxisTran->translation.setValue(0, 0, height / 2.0);
  zaxisTran->rotation.setValue(SbVec3f(1, 0, 0), (float)M_PI / 2.0f);

  SoSeparator *zaxisSep = new SoSeparator;
  zaxisSep->addChild(zaxisTran);
  zaxisSep->addChild(zaxisMat);
  zaxisSep->addChild(zaxisCyl);

  cne->addChild(tran);
  cne->addChild(zaxisSep);
  cne->addChild(coneMat);
  cne->addChild(coords);
  cne->addChild(ifs);
  return cne;
}
