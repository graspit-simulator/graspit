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
// $Id: grasp_visualization.h,v 1.2 2009/03/25 22:10:24 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_visualization.h                                    */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines a grasp_representation class (part of grasp planner)
*/


#ifndef GRASP_VISUALIZATION_H
#define GRASP_VISUALIZATION_H

/* this defines only the size of the spheres that represent
   the quality measure */
#define GRASP_SPHERE_SIZE_FACTOR 50.0

//! This class is used to create a visual representation of a candidate grasp.
/*!
  After the user generates a set of candidate grasps either automatically with
  the planner, or by reading them in from a text file,  a renderArea showing
  the shape primitves of the grasped object is opened.  For each candidate
  grasp, a grasp representation, consisting of a sphere and two arrows at right
  angles, is built in that same window.  The sphere indicates the center
  position of the palm relative to the object.  The long arrow indicates
  the palm approach direction, and the short arrow indicates the thumb
  direction.  During the testing process, the sphere representing the
  current grasp being tested is colored red.  The resulting quality of the
  grasp is used to set the relative radiius of the grasp sphere.  The higher
  the quality, the larger the sphere.
*/
class grasp_representation
{
  //! Inventor root node where this representation will be added to
  SoSeparator *parentSep;

  //! Root node of this representation
  SoSeparator *top;

  //! Root of sub-graph containing sphere and palm arrow
  SoSeparator *sep;

  //! Root of sub-graph containing thumb arrow
  SoSeparator *thSep;
  
  //! Material for the representation
  SoMaterial  *material;
  
  //! The position and orientation of the sphere and approach vecotr
  SoTransform *tranArrowSphere;
  
  //! A pointer to the long approach arrow
  SoArrow     *arrow;

  //! A pointer to the palm position/quality indicator sphere
  SoSphere    *sphere;
    
  //! The rotation of the thumb arrow
  SoTransform *thRot;

  //! A pointer to the short thumb arrow
  SoArrow     *thArrow;

  //! Not used right now.
  bool visOn;

 public:
    
    grasp_representation(SbMatrix, SbMatrix, SoSeparator*);
    ~grasp_representation();

    void changeColor(double, double, double);
    void resetColor();

    void changeRadius(double);
};



#endif

/******************
   Local Variables:
   mode:c++
   End:
******************/

