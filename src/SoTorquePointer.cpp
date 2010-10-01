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
// $Id: SoTorquePointer.cpp,v 1.3 2009/03/25 22:53:49 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements a torquePointer node for Inventor/Coin.
*/

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/engines/SoCalculator.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoTranslation.h>

#include <assert.h>
#include <math.h>

#include "SoTorquePointer.h"
#include "curvedArrow.dat"

SO_NODE_SOURCE(SoTorquePointer);

//! A pointer to the curved arrow geometry data.
SoSeparator *SoTorquePointer::curvedArrow;

/*!
  Initializes the SoArrow class. This is a one-time
  thing that is done after database initialization and before
  any instance of this class is constructed.  It reads the binary geometry
  data in curvedArrow.dat and stores a pointer to it.
*/
void
SoTorquePointer::initClass()
{
   // Initialize type id variables
   SO_NODE_INIT_CLASS(SoTorquePointer, SoComplexShape, "SoComplexShape");

   SoInput in;
   in.setBuffer((void *) curvedArrowData, (size_t) sizeof(curvedArrowData));
   curvedArrow = SoDB::readAll(&in);

   curvedArrow->ref();
}

/*!
  Sets up default field values.
*/
SoTorquePointer::SoTorquePointer()
{
  children = new SoChildList(this);

   SO_NODE_CONSTRUCTOR(SoTorquePointer);
   SO_NODE_ADD_FIELD(cylRadius, (0.5));
   SO_NODE_ADD_FIELD(height,    (4.0));
}

/*!
  Deletes the child nodes that make up this node.
*/
SoTorquePointer::~SoTorquePointer()
{
  delete children;
}

/*! 
  This is called once to generate the child nodes that make up this
  complex shape.  The child nodes consist of a cylinder, transforms, and
  cones.  A calculator node caluculates the height of the cylinder from
  the total height field and the height of any present arrowheads.
*/
void
SoTorquePointer::generateChildren()
{
  // This should be called once, that means children
  // doesn not have any children yet.
  assert (children->getLength() == 0); 
  
  // Move the curvedArrow close to the end of the shaft
  // Also scale it so that it fits around the shaft
  SoCalculator *calEngine = new SoCalculator;
  calEngine->a.connectFrom(&height);
  calEngine->b.connectFrom(&cylRadius);

  // Compute the translation of the shaft
  calEngine->expression.set1Value(0, "oA = vec3f(0,a/2.0,0)");

  // Compute the translation of the curved arrow
  calEngine->expression.set1Value(1, "oB = vec3f(0.0, 0.95*a, 0.0)");

  // Compute the scale of the curved arrow
  calEngine->expression.set1Value(2, "oC = vec3f(b/0.5,b/0.5,b/0.5)");
  
  SoCylinder *shaft = new SoCylinder;
  shaft->radius.connectFrom(&cylRadius);
  shaft->height.connectFrom(&height);

  SoTranslation *shaftTran = new SoTranslation;
  shaftTran->translation.connectFrom(&calEngine->oA);

  SoTranslation *arrowTran = new SoTranslation;
  arrowTran->translation.connectFrom(&calEngine->oB);

  SoScale *arrowScale = new SoScale;
  arrowScale->scaleFactor.connectFrom(&calEngine->oC);

  SoSeparator *arrowSep = new SoSeparator;
  arrowSep->addChild(arrowTran);
  arrowSep->addChild(arrowScale);
  arrowSep->addChild(curvedArrow);

  SoSeparator *root = new SoSeparator;
  root->addChild(arrowSep);
  root->addChild(shaftTran);
  root->addChild(shaft);

  children->append(root);
}

  

   
  


