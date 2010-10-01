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
// $Id: SoArrow.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines an arrow node for Inventor/Coin.
*/

#include <Inventor/engines/SoCalculator.h>
#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/fields/SoSFBitMask.h>
#include <Inventor/nodes/SoSwitch.h>
#include "SoComplexShape.h"

//! A ComplexShape node for Inventor/Coin that defines an arrow or pointer.
/*!
  This class combines a cylinder with 0,1, or 2 cones that serve as arrow
  heads.  This makes it easy to add arrows to an Inventor scene without
  defining the pieces separately.  The lengths and radii of the various
  pieces can all be customized.
*/
class SoArrow : public SoComplexShape {

   SO_NODE_HEADER(SoArrow);

 public:

   //! Bitflags controlling which arrowheads are visible (NONE, BEGIN, END, or BOTH)
   enum Part {
     NONE   = 0x00, 
     BEGIN  = 0x01,             // Arrow at the beginning of the cylinder
     END    = 0x02,             // at the end
     BOTH   = 0x03              // at both ends
   };

   // Fields
   //! Defines which arrow heads should be shown.
   SoSFBitMask   arrowHeads;

   //! Width of arrow shaft
   SoSFFloat     cylRadius;     

   //! Height of the entire arrow
   SoSFFloat     height;

   //! Height of the arrow head
   SoSFFloat     coneHeight;
    
   //! Radius of the arrow head
   SoSFFloat     coneRadius;    

   static void   initClass();
   SoArrow();

   void          addPart(Part part);
   void          removePart(Part part);
   SbBool        hasPart(Part part) const;

 private:

   //! A pointer to the calculator engine that computes the cylinder height
   SoCalculator *calEngine;

   //! Pointer to switch node that controls the visibility of an arrowhead
   SoSwitch     *beginSw,*endSw;

   virtual ~SoArrow();

   void generateChildren();

};
