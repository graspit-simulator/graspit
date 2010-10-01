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
// $Id: grasp_directions.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_directions.cc                                      */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the class GraspDirection , the palm position and approach vector for a candidate grasp (used in the grasp planner)
*/


#include <math.h>
#include "matvec3D.h"

/* STL */
#include "list"

#include "grasp_coordinates.h"
#include "grasp_directions.h"

/*!
  Sets empty to false.
*/
GraspDirection::GraspDirection()
{
  //    point = new coordinates;
  //    dir = new coordinates;
    empty = false;
}

/*!
  Stub destructor.
*/
GraspDirection::~GraspDirection()
{
  //    delete point;
  //    delete dir;
}

/*!
  Returns a pointer to the palm position coordinates.
*/
coordinates         
GraspDirection::get_point()      const
{
    return *point;
}

/*!
  Returns a pointer to the palm approach vector coordinates.
*/
coordinates         
GraspDirection::get_dir()          const
{ 
    return *dir;
}

/*!
  Sets empty to the value of \a in .
*/
void                
GraspDirection::set_empty(bool in)
{
    empty = in;
}

/*!
  Returns the value of empty.
*/
bool                
GraspDirection::get_empty()        const
{
    return empty;
}

/*!
  Sets the grasp direction type to \a in .
*/
void                
GraspDirection::set_gdType(graspDirectionType in)
{
    gdType = in;
}

/*!
  Returns the grasp direction type.
*/
graspDirectionType  
GraspDirection::get_gdType()       const
{
    return gdType;
}

/*!
  Compares this grasp direction to \a p .  If the point and direction
  are the same and the value of empty is the same, then it returns TRUE.  
*/
bool            
GraspDirection::operator==(const GraspDirection& p)
{

  //ATM: Hmmmm why is steffen comparing pointer values?
    if (p.get_point() == get_point() &&
	p.get_dir() == get_dir() &&
	p.get_empty() == get_empty())
	return true;
    return false;
}



/***************
CARTESIAN GraspDirection
***************/

/*!
  Creates new cartesian coordinates for both point and dir . Sets empty to
  FALSE.
*/
cartesianGraspDirection::cartesianGraspDirection()
{
    point = new cartesian_coordinates();
    dir = new cartesian_coordinates();
    empty = false;
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(GraspDirection* p)
{
    point = new cartesian_coordinates(p->get_point());
    dir = new cartesian_coordinates(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(cartesianGraspDirection* p)
{
    point = new cartesian_coordinates(p->get_point());
    dir = new cartesian_coordinates(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cartesianGraspDirection::cartesianGraspDirection(const cartesianGraspDirection& p) : GraspDirection()
{
    point = new cartesian_coordinates(p.get_point());
    dir = new cartesian_coordinates(p.get_dir());
    empty = p.get_empty();
    set_gdType(p.get_gdType());
}

/*!
  Deletes point and dir .
*/
cartesianGraspDirection::~cartesianGraspDirection()
{
    delete point;
    delete dir;
}

/*!
  Sets the point to the value of \a in .
*/
void        
cartesianGraspDirection::set_point(coordinates in)
{
    *point = in;
}

/*!
  Sets the directions to the value of \a in .
*/
void        
cartesianGraspDirection::set_dir(coordinates in)
{
    *dir = in;
}

/*!
  Returns a normalized distance measure between this grasp direction and
  \a to .
*/
double 
cartesianGraspDirection::distanceTo(cartesianGraspDirection to) const
{
    double dist = point->distanceTo(to.get_point());
    dist += dir->distanceTo(to.get_dir());
    /* max is 2.0, every dist normed to 1 */
    return dist / 2.0;
}

/***************
CYLINDRICAL GraspDirection
***************/
/*!
  Creates new cylindrical coordinates for both point and dir . Sets empty to
  FALSE.
*/
cylindricalGraspDirection::cylindricalGraspDirection()  : GraspDirection()
{
    point = new cylindrical_coordinates();
    dir = new cylindrical_coordinates();
    empty = false;
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cylindricalGraspDirection::cylindricalGraspDirection(GraspDirection* p) : GraspDirection()
{
    point = new cylindrical_coordinates();
    set_point(p->get_point());
    dir = new cylindrical_coordinates();
    set_dir(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cylindricalGraspDirection::cylindricalGraspDirection(cylindricalGraspDirection* p) : GraspDirection()
{
    point = new cylindrical_coordinates(p->get_point());
    dir = new cylindrical_coordinates(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
cylindricalGraspDirection::cylindricalGraspDirection(const cylindricalGraspDirection& p) : GraspDirection()
{
    point = new cylindrical_coordinates(p.get_point());
    dir = new cylindrical_coordinates(p.get_dir());
    empty = p.get_empty();
    set_gdType(p.get_gdType());
}

/*!
  Deletes point and dir .
*/
cylindricalGraspDirection::~cylindricalGraspDirection()
{
    delete point;
    delete dir;
}

/*!
  Sets the point to the value of \a in .
*/
void        
cylindricalGraspDirection::set_point(coordinates in)
{
    (*point)[0] = in[0];

    while (in[1] > (2*M_PI))
	in[1]-=(2*M_PI);
    while (in[1] < (2*M_PI))
	in[1]+=(2*M_PI);
    (*point)[1] = in[1];

    (*point)[2] = in[2];
}

/*!
  Sets the directions to the value of \a in .
*/
void        
cylindricalGraspDirection::set_dir(coordinates in)
{
    (*dir)[0] = in[0];
    (*dir)[1] = (in[1] > (2 * M_PI)) ? (2*M_PI) : in[1];
    (*dir)[2] = in[2];
}


/***************
SPHERICAL PART
***************/

/*!
  Creates new spherical coordinates for both point and dir . Sets empty to
  FALSE.
*/
sphericalGraspDirection::sphericalGraspDirection() : GraspDirection()
{
    point = new spherical_coordinates();
    dir = new spherical_coordinates();
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
sphericalGraspDirection::sphericalGraspDirection(GraspDirection* p) : GraspDirection()
{
    point = new spherical_coordinates();
    set_point(p->get_point());
    dir = new spherical_coordinates();
    set_dir(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
sphericalGraspDirection::sphericalGraspDirection(sphericalGraspDirection* p) : GraspDirection()
{
    point = new spherical_coordinates(p->get_point());
    dir = new spherical_coordinates(p->get_dir());
    empty = p->get_empty();
    set_gdType(p->get_gdType());
}

/*!
  Copies the point and dir pointers from \a p.  Also copies the values
  of empty and gdType .
*/
sphericalGraspDirection::sphericalGraspDirection(const sphericalGraspDirection& p) : GraspDirection()
{
    point = new spherical_coordinates(p.get_point());
    dir = new spherical_coordinates(p.get_dir());
    empty = p.get_empty();
    set_gdType(p.get_gdType());
}

/*!
  Deletes point and dir .
*/
sphericalGraspDirection::~sphericalGraspDirection()
{
    delete point;
    delete dir;
}

/*!
  Sets the point to the value of \a in .
*/
void        
sphericalGraspDirection::set_point(coordinates in)
{
    (*point)[0] = in[0];

    while (in[1] > M_PI)
	in[1]-=M_PI;
    while (in[1] < M_PI)
	in[1]+=M_PI;
    (*point)[1] = in[1];

    while (in[2] > (2*M_PI))
	in[2]-=(2*M_PI);
    while (in[2] < (2*M_PI))
	in[2]+=(2*M_PI);
    (*point)[2] = in[2];
}

/*!
  Sets the directions to the value of \a in .
*/
void        
sphericalGraspDirection::set_dir(coordinates in)
{
    (*dir)[0] = in[0];
    (*dir)[1] = (in[1] > M_PI) ? M_PI : in[1];
    (*dir)[2] = (in[1] > (2 * M_PI)) ? (2*M_PI) : in[1];

}





















