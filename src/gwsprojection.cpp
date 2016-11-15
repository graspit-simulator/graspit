//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Andrew T. Miller 
//
// $Id: gwsprojection.cpp,v 1.9 2009/06/16 22:53:03 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the grasp wrench space projection class.
 */

#include "pointers.dat"
#include "gwsprojection.h"
#include "body.h"
#include "grasp.h"
#include "gws.h"
#include "matvec3D.h"
#include <set>
#include <qwidget.h>

#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/Qt/SoQtRenderArea.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

#define HULLAXES_SCALE 50.0

/*!
  A GWS projection must be initialized with the a pointer to the mainWindow,
  a pointer to the GWS being projected, a 6x1 projection coordinates vector,
  \a c, and set specifiying which of these coordinates are fixed. 
*/
GWSprojection::GWSprojection(SoQtExaminerViewer *mainViewer,GWS *g,double *c,
			     std::set<int> whichFixed)
{
  gws = g;
  GraspableBody *object = gws->getGrasp()->getObject();

  memcpy(projCoords,c,6*sizeof(double));

  fixedCoordIndex = whichFixed;

  SoMaterial *mat = new SoMaterial;
  SoShapeHints *myHints = new SoShapeHints;
  myHints->shapeType = SoShapeHints::SOLID;
  myHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
  
  mat->diffuseColor = SbColor(0,0.8f,0);
  mat->ambientColor = SbColor(0,0.2f,0);
  mat->transparency = 0.4f;

  hullCoords = new SoCoordinate3;
  hullIFS = new SoIndexedFaceSet;
  hullSep = new SoSeparator;
  hullSep->addChild(myHints);
  hullSep->addChild(mat);
  hullSep->addChild(hullCoords);
  hullSep->addChild(hullIFS);
  
  SoInput in;
  in.setBuffer((void *) pointersData, (size_t) sizeof(pointersData));
  SoSeparator *pointers = SoDB::readAll(&in);
  SoSeparator *hullaxes = (SoSeparator *)pointers->getChild(3);
  SoScale *hasf = new SoScale;
  double scale = gws->getGrasp()->getMaxRadius();
  hasf->scaleFactor=SbVec3f(scale/HULLAXES_SCALE, scale/HULLAXES_SCALE, scale/HULLAXES_SCALE);
  hullaxes->insertChild(hasf,0);

  SoRotation *lightDir = new SoRotation;
  lightDir->rotation.connectFrom(&mainViewer->getCamera()->orientation);
  SoTransformSeparator *lightSep = new SoTransformSeparator;
  lightSep->addChild(lightDir);
  lightSep->addChild(mainViewer->getHeadlight());

  SoTransform *hullTran = new SoTransform;
  if (!hullTran) printf("NULL hullTran!\n");

  if (object!=NULL) {
	hullTran->translation.connectFrom(&object->getIVTran()->translation);
	hullTran->rotation.connectFrom(&object->getIVTran()->rotation);
  } else {
	  hullTran->translation = gws->getGrasp()->getCoG().toSbVec3f();
  }

  sg = new SoSeparator;
  // create our own camera so it has better clipping planes
  SoPerspectiveCamera *camera = new SoPerspectiveCamera();
  if (!camera->position.connectFrom( &mainViewer->getCamera()->position )) 
	  fprintf(stderr,"Projection camera connection 1 failed!\n");
  if (!camera->orientation.connectFrom( &mainViewer->getCamera()->orientation ))
	  fprintf(stderr,"Projection camera connection 2 failed!\n");
  camera->nearDistance = 5;
  camera->farDistance = 1000;
  sg->addChild(camera);
  // original code just re-used main camera
  //sg->addChild( mainViewer->getCamera() );

  sg->addChild(lightSep);   
  sg->addChild(hullTran);
  sg->addChild(hullaxes);
  sg->addChild(hullSep); 

  pointers->ref();
  pointers->unref();


  projectionViewer = new SoQtRenderArea();
  projectionViewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
  projectionViewer->setBackgroundColor(SbColor(1,1,1));
  projectionViewer->setSceneGraph(sg);
  if (projectionViewer->isTopLevelShell()) printf("TOP LEVEL SHELL\n");
  else printf("NOT TOP LEVEL SHELL\n");
  projectionViewer->setWindowCloseCallback(Grasp::destroyProjection,this);

  projectionViewer->show();
  setWinTitle();
  projWin = projectionViewer->getParentWidget();

  update();

}

/*!
  Removes 1 reference to a GWS (if refcount goes to 0 it will be deleted).
  If the window still exists, delete it.  Delete the projection viewer.
*/
GWSprojection::~GWSprojection()
{
  gws->getGrasp()->removeGWS(gws);
  if (projectionViewer->getShellWidget()) {
	projectionViewer->setWindowCloseCallback(NULL);
	delete projectionViewer->getShellWidget();
  }
  delete projectionViewer;
}

/*!
  Sets the window title of the projection window after the viewer is created.
  The title is specifies the type of GWS and the coordinate values of
  fixed coordinates.
  (i.e. "L1 GWS projection (0, 0, 0, *, *, *)" is a projection into torque-
  space)
*/
void
GWSprojection::setWinTitle()
{
  int i;
  char titleStr[100],element[6];

  sprintf(titleStr,"%s GWS projection (",gws->getType());
  for (i=0;i<6;i++) {
    if (fixedCoordIndex.find(i) == fixedCoordIndex.end())
      strcat(titleStr," * ");
    else {
      sprintf(element,"%4.1f",projCoords[i]);
      strcat(titleStr,element);
    }

    if (i<5)
      strcat(titleStr,",");
  }
  strcat(titleStr,")");

  projectionViewer->setTitle(titleStr);
}

/*!
  Updates the 3D hull by calling the projection routine of the associated
  GWS.  The new hull geometry is then created.
*/
void
GWSprojection::update()
{
  int i;
  std::vector<position> coords;
  std::vector<int> indices;

  if (gws->isForceClosure() || gws->hasPositiveVolume() )
    gws->projectTo3D(projCoords,fixedCoordIndex,coords,indices);

  hullCoords->point.deleteValues(0);
  hullIFS->coordIndex.deleteValues(0);

  int numCoords = coords.size();
  for (i=0;i<numCoords;i++) {
    hullCoords->point.set1Value(i,(float)coords[i].x(),(float)coords[i].y(),
				(float)coords[i].z());
  }

  int numIndices = indices.size();
  for (i=0;i<numIndices;i++) {
    hullIFS->coordIndex.set1Value(i,indices[i]);
  }
  
  coords.clear();
  indices.clear();
}

/*!
  Replaces 3D hull geometry with an empty node.
*/
void
GWSprojection::deleteHull()
{
  hullSep->removeChild(2);
  hullSep->removeChild(2);
  hullCoords = new SoCoordinate3;
  hullIFS = new SoIndexedFaceSet;
  hullSep->addChild(hullCoords);
  hullSep->addChild(hullIFS);
}
