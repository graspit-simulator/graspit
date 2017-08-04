#include "graspit/contact/virtualContactOnObject.h"

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

#include <iostream>
#include <fstream>

VirtualContactOnObject::VirtualContactOnObject()
{
  wrench = NULL;
  body2 = NULL;
  mate = this;
  prevBetas = NULL;
}

VirtualContactOnObject::~VirtualContactOnObject()
{
}

bool
VirtualContactOnObject::readFromFile(std::ifstream &inFile)
{
  if (!inFile.is_open())
  {
    DBGA("VirtualContact::readFromFile - Failed to read from file");
    return false;
  }

  float w, x, y, z;

  //numFCVectors
  inFile >> numFrictionEdges;
  if (inFile.fail()) {
    DBGA("VirtualContactOnObject::readFromFile - Failed to read number of friction vectors");
    return false;
  }

  //frictionEdges
  for (int i = 0; i < numFrictionEdges; i++) {
    for (int j = 0; j < 6; j++) {
      inFile >> w;
      if (inFile.fail()) {
        DBGA("VirtualContactOnObject::readFromFile - Failed to read number of friction edges");
        return false;
      }
      frictionEdges[6 * i + j] = w;
    }
  }
  fprintf(stderr, "\n<frictionEdges scanned successfully>"); // for test

  // (w,x,y,z) is already a quaternion, if you want to do frame rotate v rad along a vector (x,y,z),
  //you can use q(v,vec(x,y,z))
  vec3 t;
  inFile >> w >> x >> y >> z;
  if (inFile.fail()) {
    DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact location");
    return false;
  }

  Quaternion q(w, x, y, z);

  inFile >> x >> y >> z;
  if (inFile.fail()) {
    DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact orientation");
    return false;
  }

  t.x() = x;
  t.y() = y;
  t.z() = z;
  loc = position(x, y, z);
  frame.set(q, t);

  //normal
  inFile >> x >> y >> z;
  if (inFile.fail()) {
    DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact normal");
    return false;
  }

  normal.x() = x;
  normal.y() = y;
  normal.z() = z;

  //sCof
  inFile >> w;
  if (inFile.fail()) {
    DBGA("VirtualContactOnObject::readFromFile - Failed to read virtual contact normal");
    return false;
  }
  sCof = w;
  return true;
}


void
VirtualContactOnObject::writeToFile(std::ofstream &outFile) {

  if (!outFile.is_open())
  {
    DBGA("VirtualContactOnObject::writeToFile: failed to open file");
    return;
  }

  outFile << numFrictionEdges << std::endl;

  //frictionEdges
  for (int i = 0; i < numFrictionEdges; i++) {
    for (int j = 0; j < 6; j++) {
      outFile << frictionEdges[6 * i + j] << " ";
    }
    outFile << std::endl;
  }

  //frame
  Quaternion q = frame.rotation();
  vec3 t = frame.translation();
  outFile << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

  //normal
  outFile << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;

  //sCof
  outFile << sCof << std::endl;
}
