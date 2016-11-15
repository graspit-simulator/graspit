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
// $Id: grasp_directions.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_directions.h                                       */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the class GraspDirection , the palm position and approach vector for a candidate grasp (used in the grasp planner)
*/

#ifndef __GRASP_DIRECTIONS_H__
#define __GRASP_DIRECTIONS_H__

enum graspDirectionType{GDT_CYL_SIDES, GDT_CYL_TOP_BOTTOM, 
			GDT_CUBE_WIDTH, 
			GDT_CUBE_HEIGHT, 
			GDT_CUBE_DEPTH, 
			GDT_SPH, 
			GDT_CONE_TOP, GDT_CONE_BOTTOM, GDT_CONE_SIDE_PLANE, GDT_CONE_EDGE};

#include "grasp_coordinates.h"

//!  This is the parent class for the different types of grasp directions
/*!
  A grasp directions consists of a palm position and palm approach vector.
  These can be expressed in any coordinate system (cartesian, cylindrical,
  or spherical.  A grasp direction is created by the planner by using
  different rules for different types of shape primitives.
*/
class GraspDirection
{
protected:
  //! Palm position
  coordinates*       point;

  //! Palm approach vector
  coordinates*       dir;

  //! Not used.  (Not sure what Steffen intended this for)
  bool               empty;

  //! The rule type that resulted in this grasp direction
  graspDirectionType gdType;
  
public:
  GraspDirection();
  virtual ~GraspDirection();
  
  /*! Sets the value of the palm position to \a in . */
  virtual void        set_point(coordinates in) = 0;
  coordinates         get_point() const;
  
  /*! Sets the value of the palm approach vector to \a in .*/
  virtual void        set_dir(coordinates in) = 0;
  coordinates         get_dir() const;
  
  void                set_empty(bool in);
  bool                get_empty() const;
  
  void                set_gdType(graspDirectionType);
  graspDirectionType  get_gdType() const; 
  
  bool                operator==(const GraspDirection&);

};

//! This class is used when the palm position and direction are expressed in cartesian coordinates
class cartesianGraspDirection :
public GraspDirection
{
public:
  cartesianGraspDirection();
  cartesianGraspDirection(GraspDirection*);
  cartesianGraspDirection(cartesianGraspDirection*);
  cartesianGraspDirection(const cartesianGraspDirection&);
  
  ~cartesianGraspDirection();
  
  void        set_point(coordinates in);
  void        set_dir(coordinates in);
  
  double distanceTo(cartesianGraspDirection) const;
};

//! This class is used when the palm position and direction are expressed in cylindrical coordinates
class cylindricalGraspDirection :
public GraspDirection
{
public:
  cylindricalGraspDirection();
  cylindricalGraspDirection(GraspDirection*);
  cylindricalGraspDirection(cylindricalGraspDirection*);
  cylindricalGraspDirection(const cylindricalGraspDirection&);
  
  ~cylindricalGraspDirection();
  
  void        set_point(coordinates in);
  void        set_dir(coordinates in);
};


//! This class is used when the palm position and direction are expressed in spherical coordinates
class sphericalGraspDirection :
public GraspDirection
{
public:
  sphericalGraspDirection();
  sphericalGraspDirection(GraspDirection*);
  sphericalGraspDirection(sphericalGraspDirection*);
  sphericalGraspDirection(const sphericalGraspDirection&);
  
  ~sphericalGraspDirection();
  
  void        set_point(coordinates in);
  void        set_dir(coordinates in);
};

#endif


/******************
   Local Variables:
   mode:c++
   End:
******************/















