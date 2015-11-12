//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
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

