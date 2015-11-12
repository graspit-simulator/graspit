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
// $Id: SoTorquePointer.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines a torque pointer node for Inventor/Coin.
*/

#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/nodes/SoSeparator.h>
#include "SoComplexShape.h"

//! A ComplexShape node for Inventor/Coin that defines an torque pointer.
/*!
  This class combines a cylinder with a circular arrow that points in a
  counter clockwise direction at the end of the cylinder.  This indicates
  a torque axis and the direction of the torque.  The length of the axis
  indiciates the magnitude of the torque.  This node makes it easy to add
  these pointers to an Inventor scene without defining the pieces separately.
  The geometry for the circular arrow has been precomputed and the binary
  Inventor data is included directly as a header file.
*/
class SoTorquePointer : public SoComplexShape {

   SO_NODE_HEADER(SoTorquePointer);

 public:

   // Fields
   //! Defines the width of the shaft
   SoSFFloat     cylRadius;

   //! Defines the height of the shaft
   SoSFFloat     height;

   static void   initClass();
   SoTorquePointer();

 private:
   //! A pointer to the geometry of the curved arrow
   static SoSeparator *curvedArrow;
   
   virtual ~SoTorquePointer();

   void generateChildren();
};
