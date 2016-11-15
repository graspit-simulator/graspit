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
// Authors: Steffen Knoop
//          Andrew T. Miller 
//
// $Id: grasp_visualization.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_visualization.cc                                   */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements a grasp_representation (part of grasp planner)
*/

#include <iostream>

/* inventor includes */
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SoLists.h>
#include "SoArrow.h"
#include "SoTorquePointer.h"
#include "SoComplexShape.h"
#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/SbColor.h>
#include <Inventor/nodes/SoTransform.h>

#include "grasp_visualization.h"

/*!
  Creates a visual representation of a candidate grasp.  The representations
  consists of a sphere with a long arrow leaving the center of the sphere and
  a short arrow at a right angle to the longer one.  The long arrow indicates
  the palm approach vector, and the short arrow indicates the thumb direction.
  The rotation of the palm, z-approach, vector is specified with \a mat ,
  the rotation of the thumb vector is specified with \a thMat .  The Inventor
  sub graph containing these elements is added to the provided \a glroot node .
*/    
grasp_representation::grasp_representation(SbMatrix mat, SbMatrix thMat, SoSeparator* glroot){

    parentSep = glroot;
    
    top = new SoSeparator();
    sep = new SoSeparator();
    thSep = new SoSeparator();

    material = new SoMaterial();

    tranArrowSphere = new SoTransform();
    arrow = new SoArrow();
    arrow->height.setValue(50.);
    sphere = new SoSphere();
    sphere->radius.setValue(5.0);
    
    thRot = new SoTransform();
    thArrow = new SoArrow();
    thArrow->height.setValue(20.);

    top->addChild(sep);
    top->addChild(thSep);

    sep->addChild(tranArrowSphere);
    sep->addChild(material);
    sep->addChild(arrow);
    sep->addChild(sphere);

    thSep->addChild(material);
    thSep->addChild(thRot);
    thSep->addChild(thArrow);

    tranArrowSphere->setMatrix(mat);

    thRot->setMatrix(thMat);

    parentSep->addChild(top);

    visOn = true;
}

/*!
  Removes and deletes the Inventor elements for this represenntation.
*/
grasp_representation::~grasp_representation(){

    if (visOn)
	parentSep->removeChild(top);
    /*
    thSep->removeChild(thArrow);
    thSep->removeChild(thRot);
    thSep->removeChild(material);

    sep->removeChild(sphere);
    sep->removeChild(arrow);
    sep->removeChild(material);
    sep->removeChild(tranArrowSphere);

    top->removeChild(thSep);
    top->removeChild(sep);

    thArrow->unref();
    thRot->unref();
    sphere->unref();
    arrow->unref();
    material->unref();
    
    thSep->unref();
    sep->unref();
    top->unref();
    */
}

/*!
  Changes the diffuse color of the representation.
*/
void 
grasp_representation::changeColor(double r, double g, double b){
    material->diffuseColor.setValue((float)r,(float)g,(float)b);
    material->transparency.setValue(0.5);
}

/*!
  Resets the material properties of the representation elements to their
  original values.
*/
void 
grasp_representation::resetColor(){
    material->setToDefaults();
}

/*!
  Alters the radius of the center sphere of the representation to \a rad
  * GRASP_SPHERE_SIZE_FACTOR.
*/
void
grasp_representation::changeRadius(double rad){
    sphere->radius.setValue((float)(rad * GRASP_SPHERE_SIZE_FACTOR));
}


/******************
   Local Variables:
   mode:c++
   End:
******************/









