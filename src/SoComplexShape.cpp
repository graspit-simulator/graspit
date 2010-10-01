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
// $Id: SoComplexShape.cpp,v 1.2 2009/03/25 22:10:03 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements a new abstract shape class for Inventor/Coin
*/

#include <Inventor/misc/SoChildList.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodekits/SoShapeKit.h>

#include "SoComplexShape.h"

#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoPickAction.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoHandleEventAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>

SO_NODE_ABSTRACT_SOURCE(SoComplexShape);


/*!
  Initializes the SoComplexShape class. This is a one-time
  thing that is done after database initialization and before
  any instance of this class is constructed.
*/
void
SoComplexShape::initClass()
{
   // Initialize the run-time type variables
   SO_NODE_INIT_ABSTRACT_CLASS(SoComplexShape, SoNode, "SoNode");

}

/*!
  Generic constructor
*/
SoComplexShape::SoComplexShape()
{
   SO_NODE_CONSTRUCTOR(SoComplexShape);
}


/*!
  Empty stub.
*/
SoComplexShape::~SoComplexShape()
{
}

/*!
 Returns the children list.
*/
SoChildList *SoComplexShape::getChildren() const
{
  return children;
}


/*!
 Implements the SoGLRenderAction for the SoCoordinateAxis node.
*/
void
SoComplexShape::GLRender(SoGLRenderAction *action)
{
  SoComplexShape::doAction(action);
}

/*!
 Generates triangles representing a SoComplexShape.
*/
void
SoComplexShape::callback(SoCallbackAction *action)
{
  SoComplexShape::doAction(action);
}

/*!
  Enlarges the current bounding box by adding the bounding boxes of each
  of this shape's children.
*/
void
SoComplexShape::getBoundingBox(SoGetBoundingBoxAction *action)
{
   SoComplexShape::doAction(action);
}

/*!
  Passes the event on to each of the children of the complex shape.
*/
void
SoComplexShape::handleEvent(SoHandleEventAction *action)
{
   SoComplexShape::doAction(action);
}

/*!
  Passes the pick action on to each of the children of the complex shape.
*/
void
SoComplexShape::pick(SoPickAction *action)
{
   SoComplexShape::doAction(action);
}

/*!
 This implements the traversal for the SoGetMatrixAction,
 which is handled a little differently: it does not traverse
 below the root node or tail of the path it is applied to.
 Therefore, we need to compute the matrix only if this group
 is in the middle of the current path chain or is off the path
 chain (since the only way this could be true is if the group
 is under a group that affects the chain).
*/
void
SoComplexShape::getMatrix(SoGetMatrixAction *action)
{
   int         numIndices;
   const int   *indices;

   // Use SoAction::getPathCode() to determine where this group
   // is in relation to the path being applied to (if any). (see
   // the comments in doAction() for details.)
   switch (action->getPathCode(numIndices, indices)) {

     case SoAction::NO_PATH:
     case SoAction::BELOW_PATH:
      // If there's no path, or we're off the end, do nothing
      break;

     case SoAction::OFF_PATH:
     case SoAction::IN_PATH:
      // If we are in the path chain or we affect nodes in the
      // path chain, traverse the children
      SoComplexShape::doAction(action);
      break;
   }
}


/*!
 This implements typical action traversal for an SoComplexShape node
*/
void
SoComplexShape::doAction(SoAction *action)
{
  // Make sure all the children exist
  if (children->getLength() == 0) generateChildren();

   // SoAction has a method called "getPathCode()" that returns
   // a code indicating how this node is related to the path(s)
   // the action is being applied to. This code is one of the
   // following:
   //
   // NO_PATH    = Not traversing a path (action was applied
   //                to a node) 
   // IN_PATH    = This node is in the path chain, but is not
   //                the tail node
   // BELOW_PATH = This node is the tail of the path chain or
   //                is below the tail
   // OFF_PATH   = This node is off to the left of some node in
   //                the path chain
   //
   // If getPathCode() returns IN_PATH, it returns (in its two
   // arguments) the indices of the next nodes in the paths.
   // (Remember that an action can be applied to a list of
   // paths.)

   // For the IN_PATH case, these will be set by getPathCode()
   // to contain the number of child nodes that are in paths and
   // the indices of those children, respectively. In the other
   // cases, they are not meaningful.
   int         numIndices;
   const int   *indices;

   // This will be set to the index of the last (rightmost)
   // child to traverse
   int         lastChildIndex;

   // If this node is in a path, see which of our children are
   // in paths, and traverse up to and including the rightmost
   // of these nodes (the last one in the "indices" array).
   if (action->getPathCode(numIndices, indices) ==
       SoAction::IN_PATH)
      lastChildIndex = indices[numIndices - 1];

   // Otherwise, consider all of the children
   else
      lastChildIndex = children->getLength() - 1;

   // Now we are ready to traverse the children, skipping every
   // other one. For the SoGetBoundingBoxAction, however, we
   // have to do some extra work in between each pair of
   // children - we have to make sure the center points get
   // averaged correctly.
   if (action->isOfType(
            SoGetBoundingBoxAction::getClassTypeId())) {
      SoGetBoundingBoxAction *bba =
         (SoGetBoundingBoxAction *) action;
      SbVec3f  totalCenter(0.0, 0.0, 0.0);
      int      numCenters = 0;

      for (int i = 0; i <= lastChildIndex; i++) {
         children->traverse(bba, i);

         // If the traversal set a center point in the action,
         // add it to the total and reset for the next child.
         if (bba->isCenterSet()) {
            totalCenter += bba->getCenter();
            numCenters++;
            bba->resetCenter();
         }
      }
      // Now, set the center to be the average. Since the
      // centers were already transformed, there's no need to
      // transform the average.
      if (numCenters != 0)
         bba->setCenter(totalCenter / (float)numCenters, FALSE);
   }

   // For all other actions, just traverse every child
   else
      for (int i = 0; i <= lastChildIndex; i++)
         children->traverse(action, i);
}


/*!
 Copy function
*/
void SoComplexShape::copyContents(const SoFieldContainer *FC, 
				  SbBool copyConnection)
{
  SoNode::copyContents(FC, copyConnection);
  children = ((SoComplexShape *)FC)->children;
}


