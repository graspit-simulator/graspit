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
// $Id: SoArrow.cpp,v 1.2 2009/03/25 22:10:03 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements an arrow node for Inventor/Coin.
*/

#include <Inventor/misc/SoChildList.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>

#include <assert.h>
#include <math.h>

#include "SoArrow.h"

// Shorthand macro for testing whether the current parts field
// value (parts) includes a given part (part)
#define HAS_PART(arrowHeads, part) (((arrowHeads) & (part)) != 0)

SO_NODE_SOURCE(SoArrow);

/*!
  Initializes the SoArrow class. This is a one-time
  thing that is done after database initialization and before
  any instance of this class is constructed.
*/
void
SoArrow::initClass()
{
   // Initialize type id variables
   SO_NODE_INIT_CLASS(SoArrow, SoComplexShape, "SoComplexShape");
}

/*!
  Sets up default field values.
*/
SoArrow::SoArrow()
{

  children = new SoChildList(this);

   SO_NODE_CONSTRUCTOR(SoArrow);
   SO_NODE_ADD_FIELD(arrowHeads,(END));
   SO_NODE_ADD_FIELD(cylRadius, (0.5));
   SO_NODE_ADD_FIELD(height,    (4.0));
   SO_NODE_ADD_FIELD(coneHeight,(2.0));
   SO_NODE_ADD_FIELD(coneRadius,(1.0));

   // Set up static values and strings for the "arrowHeads"
   // enumerated type field.
   SO_NODE_DEFINE_ENUM_VALUE(Part, NONE);
   SO_NODE_DEFINE_ENUM_VALUE(Part, BEGIN);
   SO_NODE_DEFINE_ENUM_VALUE(Part, END);
   SO_NODE_DEFINE_ENUM_VALUE(Part, BOTH);

   // Copy static information for "arrowHeads" enumerated type field
   // into this instance. 
   SO_NODE_SET_SF_ENUM_TYPE(arrowHeads, Part);
}

/*!
  Deletes the child nodes that make up this node.
*/

SoArrow::~SoArrow()
{
  delete children;
}

/*!
  Turns on a part of the arrow.
*/
void
SoArrow::addPart(Part part)
{
   arrowHeads.setValue(arrowHeads.getValue() | part);

  if (HAS_PART(arrowHeads.getValue(),SoArrow::BEGIN)) {
    calEngine->c.setValue(1);
    beginSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->c.setValue(0);
    beginSw->whichChild.setValue(SO_SWITCH_NONE);
  }

  if (HAS_PART(arrowHeads.getValue(),SoArrow::END)) {
    calEngine->d.setValue(1);
    endSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->d.setValue(0);
    endSw->whichChild.setValue(SO_SWITCH_NONE);
  }

}

/*!
  Turns off a part of the arrow.
*/
void
SoArrow::removePart(Part part)
{
   arrowHeads.setValue(arrowHeads.getValue() & ~part);

  if (HAS_PART(arrowHeads.getValue(),SoArrow::BEGIN)) {
    calEngine->c.setValue(1);
    beginSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->c.setValue(0);
    beginSw->whichChild.setValue(SO_SWITCH_NONE);
  }

  if (HAS_PART(arrowHeads.getValue(),SoArrow::END)) {
    calEngine->d.setValue(1);
    endSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->d.setValue(0);
    endSw->whichChild.setValue(SO_SWITCH_NONE);
  }

}

/*!
  Returns whether a given part is on or off.
*/
SbBool
SoArrow::hasPart(Part part) const
{
   return HAS_PART(arrowHeads.getValue(), part);
}


/*! 
  This is called once to generate the child nodes that make up this
  complex shape.  The child nodes consist of a cylinder, transforms, and
  cones.  A calculator node caluculates the height of the cylinder from
  the total height field and the height of any present arrowheads.
*/
void
SoArrow::generateChildren()
{
  // This should be called once, that means children
  // doesn not have any children yet.
  assert (children->getLength() == 0); 

  // Construct the begin arrowhead
  SoCone *cne = new SoCone;
  cne->height.connectFrom(&coneHeight);
  cne->bottomRadius.connectFrom(&coneRadius);

  SoTransform *beginTran = new SoTransform;
  beginTran->rotation.setValue(SbVec3f(1,0,0),(float)M_PI);
  
  SoSeparator *beginSep = new SoSeparator;
  beginSep->addChild(beginTran);
  beginSep->addChild(cne);
  beginSw = new SoSwitch;
  beginSw->addChild(beginSep);

  // Construct the end arrowhead
  SoTranslation *endTran = new SoTranslation;

  SoSeparator *endSep = new SoSeparator;
  endSep->addChild(endTran);
  endSep->addChild(cne);
  endSw = new SoSwitch;
  endSw->addChild(endSep);

  // Move the arrowheads to the ends of the shaft
  calEngine = new SoCalculator;
  calEngine->a.connectFrom(&height);
  calEngine->b.connectFrom(&coneHeight);

  if (HAS_PART(arrowHeads.getValue(),SoArrow::BEGIN)) {
    calEngine->c.setValue(1);
    beginSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->c.setValue(0);
    beginSw->whichChild.setValue(SO_SWITCH_NONE);
  }

  if (HAS_PART(arrowHeads.getValue(),SoArrow::END)) {
    calEngine->d.setValue(1);
    endSw->whichChild.setValue(SO_SWITCH_ALL);
  }
  else {
    calEngine->d.setValue(0);
    endSw->whichChild.setValue(SO_SWITCH_NONE);
  }

  // Calculate the height of the shaft
  calEngine->expression.set1Value(0, "oa = a - c*b - d*b");

  // Compute the position where the begin cone needs to be moved to
  calEngine->expression.set1Value(1, "oA = vec3f(0.0, b/2.0, 0.0)");
  // Compute the position where the end cone needs to be moved to
  calEngine->expression.set1Value(2, "oB = vec3f(0.0, a - b/2.0, 0.0)");
  // Compute the position where the shaft needs to be moved to
  calEngine->expression.set1Value(3, "oC = vec3f(0.0, oa/2.0 + c*b, 0.0)");

  beginTran->translation.connectFrom(&calEngine->oA);
  endTran->translation.connectFrom(&calEngine->oB);
  
  SoCylinder *shaft = new SoCylinder;
  shaft->radius.connectFrom(&cylRadius);
  shaft->height.connectFrom(&calEngine->oa);

  SoTranslation *shaftTran = new SoTranslation;
  shaftTran->translation.connectFrom(&calEngine->oC);

  SoSeparator *root = new SoSeparator;
  root->addChild(beginSw);
  root->addChild(endSw);
  root->addChild(shaftTran);
  root->addChild(shaft);

  children->append(root);
}

  

   
  


