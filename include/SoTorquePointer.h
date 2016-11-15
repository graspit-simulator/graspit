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
