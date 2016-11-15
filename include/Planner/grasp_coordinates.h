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
// $Id: grasp_coordinates.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_coordinates.h                                      */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
  \brief Defines several types of coordinates classes (used in the grasp planner)
*/

#ifndef __GRASP_COORDINATES_H__
#define __GRASP_COORDINATES_H__

#include "matvec3D.h"
class coordinates;
class cartesian_coordinates;
class cylindrical_coordinates;
class spherical_coordinates;

//! This is the base class for various coordinate representations.
/*! 
  It can convert between the various types of coordinate systems.
*/
class coordinates :
public vec3
{
private:
public:
  //! The three different types of coordinate systems
  enum coord_system_type{
    cartesian,   //!< coordinates are x, y, z
    cylindrical, //!< coordinates are r, phi, z
    spherical    //!< coordinates are r, theta, phi
  };

  //! Holds the type of coordinate representation for this instance
  coord_system_type coord_type;
  
  coordinates(double a, double b, double c);
  coordinates(coordinates* c);
  coordinates(vec3 v);
  coordinates();
  
  virtual coord_system_type    get_coord_system_type();
  virtual void                 set_coord_system_type(coord_system_type);
  
  /* transform coordinates as a point. Beginning of vector is attached to (0,0,0) */
  virtual cartesian_coordinates   get_pos_cartesian()   const;
  virtual cylindrical_coordinates get_pos_cylindrical() const;
  virtual spherical_coordinates   get_pos_spherical()   const;
  
  /* transform coordinates as a vector. Beginnning of vector is attached to "from" */
  virtual cartesian_coordinates   get_vec_cartesian(coordinates from)   const;
  virtual cylindrical_coordinates get_vec_cylindrical(coordinates from) const;
  virtual spherical_coordinates   get_vec_spherical(coordinates from)   const;
  
  /* distance normed on the max of both. 0 <= dist <= 1 */
  virtual double distanceTo(coordinates) const;
  
  bool operator==(coordinates);
};


//! Coordinate elements are values along 3 perpendicular axes
/*!
  This class allows conversion to other coordinate systems
*/
class cartesian_coordinates :
public coordinates
{
public:
    /* contructors */
    cartesian_coordinates(double x, double y, double z);
    cartesian_coordinates(cartesian_coordinates *cc);
    cartesian_coordinates(const cartesian_coordinates& cc);
    cartesian_coordinates(vec3 v);
    cartesian_coordinates();

    /* conversion */
    cartesian_coordinates   get_pos_cartesian()   const;
    cylindrical_coordinates get_pos_cylindrical() const;
    spherical_coordinates   get_pos_spherical()   const;

    cartesian_coordinates   get_vec_cartesian(cartesian_coordinates from)   const;
    cylindrical_coordinates get_vec_cylindrical(cartesian_coordinates from) const;
    spherical_coordinates   get_vec_spherical(cartesian_coordinates from)   const;
	    
    inline cartesian_coordinates operator+(cartesian_coordinates);
    inline cartesian_coordinates operator-(cartesian_coordinates);

    /* distance normed on the max of both. 0 <= dist <= 1 */
    virtual double distanceTo(coordinates) const;
};

//! Coordinates are expressed as radius, azimuth angle, and height
/*!
  This class allows conversion to other coordinate systems
*/
class cylindrical_coordinates :
public coordinates
{
public:
    /* contructors */
    cylindrical_coordinates(double R, double phi, double z);
    cylindrical_coordinates(cylindrical_coordinates *cc);
    cylindrical_coordinates(const cylindrical_coordinates& cc);
    cylindrical_coordinates(vec3 v);
    cylindrical_coordinates();

    /* conversion */
    cartesian_coordinates   get_pos_cartesian()   const;
    cylindrical_coordinates get_pos_cylindrical() const;
    spherical_coordinates   get_pos_spherical()   const;

    cartesian_coordinates   get_vec_cartesian(cylindrical_coordinates from)   const;
    cylindrical_coordinates get_vec_cylindrical(cylindrical_coordinates from) const;
    spherical_coordinates   get_vec_spherical(cylindrical_coordinates from)   const;

/* Not in class because of inconsistencies between position and
   vector convertion. Adding a vector field to a pose vector
   is different to adding a vector field to a vector field. */
/*
    inline cylindrical_coordinates operator+(cylindrical_coordinates);
    inline cylindrical_coordinates operator-(cylindrical_coordinates);
*/
    /* distance normed on the max of both. 0 <= dist <= 1 */
    double distanceTo(cylindrical_coordinates) const;
};

//! Coordinates are expressed as radius, azimuth angle, and elevation angle.
/*!
  This class allows conversion to other coordinate systems.
*/
class spherical_coordinates :
    public coordinates
{
public:
    /* contructors */
    spherical_coordinates(double r, double teta, double phi);
    spherical_coordinates(spherical_coordinates *cc);
    spherical_coordinates(const spherical_coordinates& cc);
    spherical_coordinates(vec3 v);
    spherical_coordinates();

    /* conversion */
    cartesian_coordinates   get_pos_cartesian()   const;
    cylindrical_coordinates get_pos_cylindrical() const;
    spherical_coordinates   get_pos_spherical()   const;

    cartesian_coordinates   get_vec_cartesian(spherical_coordinates from)   const;
    cylindrical_coordinates get_vec_cylindrical(spherical_coordinates from) const;
    spherical_coordinates   get_vec_spherical(spherical_coordinates from)   const;

/* Not in class because of inconsistencies between position and
   vector convertion. Adding a vector field to a pose vector
   is different to adding a vector field to a vector field. */
/*
    inline spherical_coordinates operator+(spherical_coordinates);
    inline spherical_coordinates operator-(spherical_coordinates);
*/
    /* distance normed on the max of both. 0 <= dist <= 1 */
    double distanceTo(spherical_coordinates) const;
};

#endif // __GRASP_COORDINATES_H__


/******************
   Local Variables:
   mode:c++
   End:
******************/










