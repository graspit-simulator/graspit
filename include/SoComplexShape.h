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
// Author(s):  Andrew T. Miller 
//
// $Id: SoComplexShape.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines a new abstract shape class for Inventor/Coin.
*/

#ifndef _SO_COMPLEX_SHAPE_H
#define _SO_COMPLEX_SHAPE_H

#include <Inventor/nodes/SoSubNode.h>

class SoChildList;

//! Abstract shape class for Inventor (Coin)
/*! SoComplexShape is a new abstract shape class that we have added to the
    Inventor/Coin library.  Since deriving subclasses from SoShape requires
    the precise bounding box computation, we represent complex shapes
    separately by deriving them from SoComplexShape, which is
    a subclass from SoNode directly.
*/
class SoComplexShape : public SoNode {

   SO_NODE_ABSTRACT_HEADER(SoComplexShape);

 public:
  
   static void    initClass();

   virtual SoChildList *getChildren() const;

   virtual void copyContents(const SoFieldContainer *FC, 
			     SbBool copyConnection);

 protected:

   // These are the methods that are used to apply the action
   // to various node classes. The third method is registered
   // for all relevant non-shape nodes. The calling sequence for
   // these methods is that used for all methods in the global
   // action table.

   SoComplexShape();

   virtual ~SoComplexShape();

   /*! Empty stub. */
   virtual void generateChildren() { }


   virtual void  doAction(SoAction *action);
   virtual void  getBoundingBox(SoGetBoundingBoxAction *action);
   virtual void  GLRender(SoGLRenderAction *action);
   virtual void  handleEvent(SoHandleEventAction *action);
   virtual void  pick(SoPickAction *action);
   virtual void  callback(SoCallbackAction *action);
   virtual void  getMatrix(SoGetMatrixAction *action);

   //! A pointer to a list of the nodes that make up this complex shape.
   SoChildList *children;


};

#endif // End of _SO_COMPLEX_SHAPE_H

