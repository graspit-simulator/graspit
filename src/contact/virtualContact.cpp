
#include "graspit/contact/virtualContact.h"
#include "graspit/contact/contact.h"
#include "graspit/debug.h"
#include "graspit/body.h"
#include "graspit/world.h"

#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

#include "tinyxml.h"
#include "graspit/mytools.h"

#include <iostream>
#include <fstream>

/*! Initializes an empty (and for now unusable) contact. Sets it
    as its own mate, which is the distinctive sign of a virtual
    contact.
*/
void
VirtualContact::init()
{
  mFingerNum = -2;
  mLinkNum = 0;
  mZaxisMat = NULL;
  mWorldInd = NULL;
  mate = this;
  body1 = NULL;
  body2 = NULL;
}

/*! Just calls the super and then calls init to give default values
    to the virtual contact specific parameters.
*/
VirtualContact::VirtualContact() : Contact()
{
  init();
}

/*! Copies the location, friction edges, and coefficients of the \a original.
    This should really use a super's copy constructor, but we never wrote
    one. Does not copy the wrenches; the new contact needs to build them
    itself from the friction edges.
*/
VirtualContact::VirtualContact(const VirtualContact *original) : Contact()
{
  init();
  mFingerNum = original->mFingerNum;
  mLinkNum = original->mLinkNum;
  //copy friction edges
  //this should really be done by a Contact copy constructor
  numFrictionEdges = original->numFrictionEdges;
  memcpy(frictionEdges, original->frictionEdges, 6 * numFrictionEdges * sizeof(double));
  //wrenches are not copied; this contact should build them itself
  loc = original->loc;
  frame = original->frame;
  normal = original->normal;
  sCof = original->sCof;
  body1 = original->body1;
  body2 = original->body2;
}

/*! When copying from a regular contact, we must be careful because the
    normal of a virtual contact points outwards, whereas the normal of a
    trasitional contact points inwards.
*/
VirtualContact::VirtualContact(int f, int l, Contact *original) : Contact()
{
  init();
  mFingerNum = f;
  mLinkNum = l;

  numFrictionEdges = original->numFrictionEdges;
  memcpy(frictionEdges, original->frictionEdges, 6 * numFrictionEdges * sizeof(double));
  sCof = original->sCof;

  loc = original->loc;
  frame = original->frame;
  //we now rotate the frame so that the normal points outwards, as the contact on the object would
  Eigen::AngleAxisd aa = Eigen::AngleAxisd(3.14159, vec3(1, 0, 0));
  Quaternion q(aa);
  transf newRot(q, vec3(0, 0, 0));
  frame = frame % newRot;
  normal = -1 * original->normal;
  body1 = original->body1;
  body2 = original->body2;
}

/*! Just sets the mate to NULL and let the super destructor
    take care of the rest.
*/
VirtualContact::~VirtualContact()
{
  mate = NULL;
}

/*! Sets the frame, location and normal, but does not do anything with the friction edges.
 */
void VirtualContact::changeFrame(transf tr)
{
  frame = tr;
  loc = tr.translation();
  normal = tr.affine() * vec3(0, 0, 1);
}

position
VirtualContact::getWorldLocation()
{
  return body1->getTran() * loc;
}

vec3
VirtualContact::getWorldNormal()
{
  return body1->getTran().affine() * normal;
}

/*! The virtual contact, for now, does not compute its own friction
    edges, but just inherits them from the regular contact that it
    starts from. Alternatively, if it loaded from a file, it reads
    them in from the file.

    In the future, this hierarchy needs to be better engineered.
*/
int
VirtualContact::setUpFrictionEdges(bool dynamicsOn)
{
  dynamicsOn = dynamicsOn;
  return 1;
}

/*! This computes the wrenches of the virtual contact as used for grasp
    quality computations. The important aspect to take into account is
    wether we are using an object or not.

    If we are using an object, we assume that contact centroid and maxradius
    have already been set to match those of the object (hopefully, the grasp
    has done this). Also, the force applied be the contact is scaled by a
    function of the distance between the contact and the actual object.

    If there is no object, we assume that contact centroid and maxradius
    have been preset dependign only on the set of virtual contacts that
    make up the grasp (again, we hope the grasp has done this). There is no
    scaling involved, all forces are handled normally as if the contact was
    actually on the object.
*/
void
VirtualContact::computeWrenches(bool useObjectData, bool simplify)
{
  int i;
  //double alpha;
  vec3 radius, forceVec, torqueVec, tangentX, tangentY;

  //we need everything to be wrt world coordinates
  position worldLoc;
  vec3 worldNormal;

  if (!useObjectData) {
    worldLoc = getWorldLocation();
    worldNormal = getWorldNormal();
    transf worldFrame = body1->getTran() % frame;
    tangentX = worldFrame.affine().col(0);
    tangentY = worldFrame.affine().col(1);
  } else {
    //    LOOK AT VIRTUAL CONTACT ON THE HAND
    worldLoc = getWorldLocation();
    worldNormal = getWorldNormal();
    tangentX = vec3(0, 1, 0).cross(worldNormal);
    tangentX = tangentX.normalized();
    tangentY = worldNormal.cross(tangentX);
  }

  //mCenter needs to be already set to object cog if needed as such
  radius = worldLoc - mCenter;
  if (wrench) { delete [] wrench; }

  if (simplify) {
    numFCWrenches = 1; //this is hack-ish, should be fixed
    wrench = new Wrench[1];
    wrench[0].force = worldNormal;
    wrench[0].torque = (radius.cross(worldNormal)) / mMaxRadius;
    return;
  }

  numFCWrenches = numFrictionEdges;
  wrench = new Wrench[numFrictionEdges];

  //SHOULD SET UP COEFFICIENT BASED ON MATERIALS!
  for (i = 0; i < numFCWrenches; i++) {
    forceVec = worldNormal + (tangentX * sCof * frictionEdges[6 * i]) + (tangentY * sCof * frictionEdges[6 * i + 1]);
    //max friction is normal_force_magnitude (which is 1) * coeff_of_friction
    //possible friction for this wrench is (friction_edge * max_friction) in the X and Y direction of the contact

    wrench[i].force = forceVec;

    wrench[i].torque = ((radius.cross(forceVec)) + worldNormal * sCof * frictionEdges[6 * i + 5]) / mMaxRadius;
    //max torque is contact_radius * normal_force_magnitude (which is 1) * coeff_of_friction
    //possible torque for this wrench is (friction_edge * max_torque) in the direction of the contact normal
    //the contact_radius coefficient is already taken into account in the friction_edge
  }
}

/*! Scales the force vector based on how distant the VirtualContact
    is from the actual object. The intuition is as follows: we do not
    want Grasp Quality to be a step function depending wether a contact
    is on of off the surface of the object. Ratherm we want it smooth so
    we allow a VirtualContact to contribute to GQ even if it's a bit off the
    surface of the object.
*/
void
VirtualContact::scaleWrenches(double factor)
{
  int i;
  for (i = 0; i < numFCWrenches; i++) {
    wrench[i].force = factor * wrench[i].force;
    wrench[i].torque = factor * wrench[i].torque;
  }
}

/*! Gives us a visual indicator of what this contact looks like,
    in WORLD COORDINATES. Since this contact has to be transformed
    to world coordinates for the sake of grasp analysis, this allows
    us to be sure that the transformation makes sense.

    It assumes WRENCHES have been computed.
*/
void
VirtualContact::getWorldIndicator(bool useObjectData)
{
  vec3 forceVec;
  position worldLoc;

  if (!useObjectData) {
    worldLoc = getWorldLocation();
  } else {
    vec3 objDist;
    getObjectDistanceAndNormal(body2, &objDist, NULL);
    worldLoc = getWorldLocation() + objDist;
  }

  SoTransform *tran = new SoTransform;
  SbMatrix tr;
  tr.setTranslate(toSbVec3f(worldLoc));
  tran->setMatrix(tr);

  SbVec3f *points = (SbVec3f *)calloc(numFCWrenches + 1, sizeof(SbVec3f));
  int32_t *cIndex = (int32_t *)calloc(4 * numFCWrenches, sizeof(int32_t));

  points[0].setValue(0, 0, 0);

  for (int i = 0; i < numFCWrenches; i++) {
    //if ( wrench[i].torque.norm() != 0 ) continue;
    forceVec = wrench[i].force;
    forceVec = Body::CONE_HEIGHT * forceVec;
    points[i + 1].setValue(forceVec.x(), forceVec.y(), forceVec.z());
    cIndex[4 * i] = 0;
    cIndex[4 * i + 1] = i + 2;
    if (i == numFCWrenches - 1) { cIndex[4 * i + 1] = 1; }
    cIndex[4 * i + 2] = i + 1;
    cIndex[4 * i + 3] = -1;
  }

  SoCoordinate3 *coords = new SoCoordinate3;
  SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
  coords->point.setValues(0, numFCWrenches + 1, points);
  ifs->coordIndex.setValues(0, 4 * numFCWrenches, cIndex);
  free(points);
  free(cIndex);

  SoMaterial *coneMat = new SoMaterial;
  coneMat->diffuseColor = SbColor(0.0f, 0.0f, 0.8f);
  coneMat->ambientColor = SbColor(0.0f, 0.0f, 0.2f);
  coneMat->emissiveColor = SbColor(0.0f, 0.0f, 0.4f);

  if (mWorldInd) {
    body1->getWorld()->getIVRoot()->removeChild(mWorldInd);
  }

  mWorldInd = new SoSeparator;
  mWorldInd->addChild(tran);
  mWorldInd->addChild(coneMat);
  mWorldInd->addChild(coords);
  mWorldInd->addChild(ifs);
  body1->getWorld()->getIVRoot()->addChild(mWorldInd);

  /*
  SoSeparator* cSep = new SoSeparator;
  tr.setTranslate( mCenter.toSbVec3f() );
  tran = new SoTransform;
  tran->setMatrix( tr );
  cSep->addChild(tran);
  SoSphere* cSphere = new SoSphere();
  cSphere->radius = 5;
  cSep->addChild(cSphere);
  body1->getWorld()->getIVRoot()->addChild(cSep);
  */
}

SoSeparator *
VirtualContact::getVisualIndicator()
{
  SoTransform *tran;

  //this one is a member variable so we can change color if we want to "mark" the contact
  if (!mZaxisMat) {
    mZaxisMat = new SoMaterial;
    mZaxisMat->ref();
  }
  mZaxisMat->diffuseColor = SbColor(0.8f, 0, 0);
  mZaxisMat->ambientColor = SbColor(0.8f, 0, 0);

  tran = new SoTransform;
  getContactFrame().toSoTransform(tran);

  SoCylinder *zaxisCyl = new SoCylinder;
  zaxisCyl->radius = 0.2f;
  zaxisCyl->height = Body::CONE_HEIGHT;

  SoSphere *zaxisSphere = new SoSphere;
  zaxisSphere->radius = 3.0f;

  SoTransform *zaxisTran = new SoTransform;
  zaxisTran->translation.setValue(0, 0, Body::CONE_HEIGHT / 2.0);
  //  zaxisTran->translation.setValue(0,0,2.0);
  zaxisTran->rotation.setValue(SbVec3f(1, 0, 0), (float)M_PI / 2.0f);

  SoSeparator *zaxisSep = new SoSeparator;
  zaxisSep->addChild(zaxisTran);
  zaxisSep->addChild(mZaxisMat);
  zaxisSep->addChild(zaxisCyl);
  //  zaxisSep->addChild(zaxisSphere);

  SoSeparator *cne = new SoSeparator;
  cne->addChild(tran);
  cne->addChild(zaxisSep);
  return cne;
}

/*! Changes the color of the visual indicator from red to blue
    if this contact is to be "marked".
*/
void
VirtualContact::mark(bool m)
{
  if (!mZaxisMat) { return; }
  if (m) {
    mZaxisMat->diffuseColor = SbColor(0, 0, 0.8f);
    mZaxisMat->ambientColor = SbColor(0, 0, 0.8f);
  } else {
    mZaxisMat->diffuseColor = SbColor(0.8f, 0, 0);
    mZaxisMat->ambientColor = SbColor(0.8f, 0, 0);
  }
}

/*! Saves all the info needed for this contact (what body and link
    it's on, location, normal, friction edges and coefficient) to
    a file.
*/
void
VirtualContact::writeToFile(std::ofstream &outFile)
{
  outFile << "<virtual_contact>\n";
  outFile << "\t<finger_number>" << mFingerNum << "</finger_number>\n";
  outFile << "\t<link_number>" << mLinkNum << "</link_number>\n";
  outFile <<  "\t<num_friction_edges>" << numFrictionEdges << "</num_friction_edges>\n";
  outFile << "\t<friction_edges>\n";

  for (int i=0; i<numFrictionEdges; i++)
  {
    outFile << "\t\t<friction_edge>";
      for (int j=0; j<6; j++)
    {
      outFile << frictionEdges[6*i+j] << " ";
    }

    outFile << "</friction_edge>\n";
  }
  outFile << "\t</friction_edges>\n";
  outFile << "\t<location>" <<  loc.x() << " " <<  loc.y()  << " " << loc.z() << "</location>\n";
  Quaternion q = frame.rotation();
  outFile << "\t<!--w, x, y, z -->\n";
  outFile << "\t<rotation>" << q.w() << " " << q.x() << " " <<  q.y() << " " << q.z()  << "</rotation>\n";
  vec3 t = frame.translation();
  outFile << "\t<translation>" << t.x() << " " <<  t.y() << " " <<  t.z()  << "</translation>\n";
  outFile << "\t<normal>" << normal.x() << " " <<  normal.y() << " " <<  normal.z()  << "</normal>\n";
  outFile << "\t<sCof>" << sCof << "</sCof>\n";
  outFile << "</virtual_contact>\n";
}


int
VirtualContact::loadFromXml(const TiXmlElement * root)
{
  const TiXmlElement * xmlElement;
  const TiXmlElement * frictionEdgesElement;
  QString elementType;
  bool ok, ok1, ok2, ok3, ok4;
  elementType = root->Value();

  if(elementType.isNull()){
     DBGA("Empty Element Type");
  }

  if(elementType == "virtual_contact")
  {
    xmlElement = findXmlElement(root,"finger_number");
    mFingerNum = QString(xmlElement->GetText()).toDouble(&ok);

    xmlElement = findXmlElement(root,"link_number");
    mLinkNum = QString(xmlElement->GetText()).toDouble(&ok);

    xmlElement = findXmlElement(root,"num_friction_edges");
    numFrictionEdges = QString(xmlElement->GetText()).toDouble(&ok);

    frictionEdgesElement = findXmlElement(root, "friction_edges");
    xmlElement = findXmlElement(frictionEdgesElement, "friction_edge");
    for (int i=0; i<numFrictionEdges; i++)
    {

     QString friction_edge = xmlElement->GetText();
     QStringList l = QStringList::split(' ', friction_edge);

     for (int j=0; j<6; j++)
     {
         frictionEdges[6*i+j] = l[j].toDouble(&ok1);
     }
     xmlElement = xmlElement->NextSiblingElement();
    }

     xmlElement = findXmlElement(root, "location");
     QString location = xmlElement->GetText();
     location = location.simplifyWhiteSpace().stripWhiteSpace();
     QStringList l = QStringList::split(' ', location);
     if(l.count()!=3){
       DBGA("Invalid Location");
       return FAILURE;
     }

     loc = position(l[0].toDouble(&ok1),l[1].toDouble(&ok2),l[2].toDouble(&ok3));

     xmlElement = findXmlElement(root, "rotation");
     QString rotation = xmlElement->GetText();
     rotation = rotation.simplifyWhiteSpace().stripWhiteSpace();
     l = QStringList::split(' ', rotation);
     if(l.count()!=4){
       DBGA("Invalid Rotation");
       return FAILURE;
     }
     Quaternion q;
     q.w() = l[0].toDouble(&ok1);
      q.x() = l[1].toDouble(&ok2);
      q.y() = l[2].toDouble(&ok3);
      q.z() = l[3].toDouble(&ok4);

     xmlElement = findXmlElement(root, "translation");
     QString translation = xmlElement->GetText();
     translation = translation.simplifyWhiteSpace().stripWhiteSpace();
     l = QStringList::split(' ', translation);
     if(l.count()!=3){
       DBGA("Invalid Translation");
       return FAILURE;
     }
     vec3 t;
     t.x() = l[0].toDouble(&ok1);
     t.y() = l[1].toDouble(&ok2);
     t.z() = l[2].toDouble(&ok3);
     frame.set(q,t);

     xmlElement = findXmlElement(root, "normal");
     QString normal_str = xmlElement->GetText();
     normal_str = normal_str.simplifyWhiteSpace().stripWhiteSpace();
     l = QStringList::split(' ', normal_str);
     if(l.count()!=3){
       QTWARNING("Invalid Normal");
       return FAILURE;
     }

     normal.x() = l[0].toDouble(&ok1);
     normal.y() = l[1].toDouble(&ok2);
     normal.z() = l[2].toDouble(&ok3);

    xmlElement = findXmlElement(root,"sCof");
    sCof = QString(xmlElement->GetText()).toDouble(&ok);

  }
  return SUCCESS;
}

/*! Sets objDistance to be the vector from the contact to the closest
    point on the given body. Sets objNormal to be the object surface normal
    at that point.
*/
void
VirtualContact::getObjectDistanceAndNormal(Body *body, vec3 *objDistance, vec3 *objNormal)
{
  position loc = getWorldLocation();
  *objDistance = body->getWorld()->pointDistanceToBody(loc, body, objNormal);
}
