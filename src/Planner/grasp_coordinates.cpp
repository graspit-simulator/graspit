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
// $Id: grasp_coordinates.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_coordinates.cc                                     */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-27-2002                                               */
/***************************************************************************/

/*! \file
  \brief Implements several types of coordinates classes (used in the grasp planner)
*/


#include <math.h>

#include "graspit/Planner/grasp_coordinates.h"
#include <iostream>

#include <QtGlobal>

#define cout std::cout
#define endl std::endl


/*!
  Initializes vec3 class with \a a , \a b , \a c .
*/
coordinates::coordinates(double a, double b, double c) : mvec(a, b, c)
{
}

/*!
  Initializes vec3 class with a copy of the values in \a c .
*/
coordinates::coordinates(coordinates *c) : mvec(c->x(), c->y(), c->z())
{
}

/*!
  Initializes vec3 class by copying the values of \a v .
*/
coordinates::coordinates(vec3 v) : mvec(v.x(), v.y(), v.z())
{
}

/*!
  Initializes vec3 class with 0,0,0.
*/
coordinates::coordinates() : mvec(0., 0., 0.)
{
}

/*!
  Returns the coordinates type.
*/
coordinates::coord_system_type
coordinates::get_coord_system_type()
{
  return coord_type;
}


/*!
  Sets the coordinate system type to \a ct .
*/
void
coordinates::set_coord_system_type(coord_system_type ct)
{
  coord_type = ct;
}

/*!
  Returns the cartesian coordinates representation of the 3 vector values.
*/
cartesian_coordinates
coordinates::get_pos_cartesian() const
{
  return cartesian_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  Returns the cylindrical coordinates representation of the 3 vector values.
*/
cylindrical_coordinates
coordinates::get_pos_cylindrical() const
{
  return cylindrical_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  Returns the spherical coordinates representation of the 3 vector values.
*/
spherical_coordinates
coordinates::get_pos_spherical() const
{
  return spherical_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  This is overrided in other sub-classes.
*/
cartesian_coordinates
coordinates::get_vec_cartesian(coordinates from) const
{
  cout << "PL_OUT: get_vec_whatever of base class coordinates should not be called; makes no sense" << from.mvec << endl;
  return cartesian_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  This is overrided in other sub-classes.
*/
cylindrical_coordinates
coordinates::get_vec_cylindrical(coordinates from) const
{
  cout << "PL_OUT: get_vec_whatever of base class coordinates should not be called; makes no sense" << from.mvec << endl;
  return cylindrical_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  This is overrided in other sub-classes.
*/
spherical_coordinates
coordinates::get_vec_spherical(coordinates from) const
{
  cout << "PL_OUT: get_vec_whatever of base class coordinates should not be called; makes no sense" << from.mvec << endl;
  return spherical_coordinates(mvec.x(), mvec.y(), mvec.z());
}

/*!
  Compares this coordinate with \a c.  If their elements are equal then TRUE
  is returned.
*/
bool
coordinates::operator==(coordinates c)
{
  if (c.x() == x() &&
      c.y() == y() &&
      c.z() == z()) {
    return true;
  }
  return false;
}

/*!
  This is overrided in other sub-classes.
*/
double
coordinates::distanceTo(coordinates to) const
{
  cout << "PL_OUT: Distance in coordinates base class makes no sense: NOT IMPLEMENTED. Returning 0.0" << to.mvec << endl;
  return 0.0;
}





/*!
  Initializes the coordinates with the values \a x , \a y , \a z .
*/
cartesian_coordinates::cartesian_coordinates(double x, double y, double z) : coordinates(x, y, z)
{
  set_coord_system_type(cartesian);
}

/*!
  Copies coordinate values from \a cc .
*/
cartesian_coordinates::cartesian_coordinates(cartesian_coordinates *cc) : coordinates(cc->x(), cc->y(), cc->z())
{
  set_coord_system_type(cartesian);
}

/*!
  Copies coordinate values from \a cc .
*/
cartesian_coordinates::cartesian_coordinates(const cartesian_coordinates &cc) : coordinates(cc.mvec.x(), cc.mvec.y(), cc.mvec.z())
{
  set_coord_system_type(cartesian);
}

/*!
  Initializes coordinate values with values from \a v . .
*/
cartesian_coordinates::cartesian_coordinates(vec3 v) : coordinates(v)
{
  set_coord_system_type(cartesian);
}

/*!
  Initializes coordinates to 0,0,0.
*/
cartesian_coordinates::cartesian_coordinates() : coordinates()
{
  set_coord_system_type(cartesian);
}

/*!
  Returns a copy of this.
*/
cartesian_coordinates
cartesian_coordinates::get_pos_cartesian() const
{
  return cartesian_coordinates(*this);
}

/*!
  Converts the cartesian coordinates to cylindrical ones, by computing
  the radius, and azimuth from x and y.
*/
cylindrical_coordinates
cartesian_coordinates::get_pos_cylindrical() const
{
  double R, phi;
  R   = sqrt(mvec.x() * mvec.x() + mvec.y() * mvec.y());
  phi = atan2(mvec.y(), mvec.x());
  // z = z();
  return cylindrical_coordinates(R , phi , mvec.z());
}

/*!
  Converts the cartesian coordinates to spherical ones, by computing
  the radius, azimuth, and elevation angles from x, y, and z.
*/
spherical_coordinates
cartesian_coordinates::get_pos_spherical() const
{
  double r, teta, phi;
  r    = sqrt(mvec.x() * mvec.x() + mvec.y() * mvec.y() + mvec.z() * mvec.z());
  teta = atan2(sqrt(mvec.x() * mvec.x() + mvec.y() * mvec.y()), mvec.z());
  phi  = atan2(mvec.y(), mvec.x());
  return spherical_coordinates(r, teta, phi);
}

/*!
  Returns a copy of this.
*/
cartesian_coordinates
cartesian_coordinates::get_vec_cartesian(cartesian_coordinates from) const
{
  from.mvec.x() = 0;  // get rid of unused parameter warning
  return cartesian_coordinates(*this);
}

/*!

*/
cylindrical_coordinates
cartesian_coordinates::get_vec_cylindrical(cartesian_coordinates from) const
{
  double Ar, Aphi;
  double phi = from.get_pos_cylindrical().y();
  Ar   = mvec.x() * cos(phi) + mvec.y() * sin(phi);          /* Ar   =  Ax * cos(phi) + Ay * sin(phi) */
  Aphi = (-1) * mvec.x() * sin(phi) +  mvec.y() * cos(phi);  /* Aphi = -Ax * sin(phi) + Ay * cos(phi) */
  // z = z();
  return cylindrical_coordinates(Ar , Aphi , mvec.z());
}

/*!

*/
spherical_coordinates
cartesian_coordinates::get_vec_spherical(cartesian_coordinates from) const
{
  double Ar, Ateta, Aphi;
  double teta = from.get_pos_spherical().y();
  double phi  = from.get_pos_spherical().z();
  Ar =  mvec.x() * sin(teta) * cos(phi)
        + mvec.y() * sin(teta) * sin(phi)
        + mvec.z() * cos(teta);                       /* Ar = Ax * sin(teta) * cos(phi) + Ay * sin(teta) * sin(phi) + Az * cos(teta) */
  Ateta = mvec.x() * cos(teta) * cos(phi)
          + mvec.y() * cos(teta) * sin(phi)
          - mvec.z() * sin(teta);                       /* Ateta = Ax * cos(teta) * cos(phi) + Ay * cos(teta) * sin(phi) - Az * sin(teta) */
  Aphi  = - mvec.x() * sin(phi)
          + mvec.y() * cos(phi);                        /* Aphi = -Ax * sin(phi) + Ay * cos(phi) */
  return spherical_coordinates(Ar, Ateta, Aphi);
}

/*!
  Returns the sum of this coordinate with \a in .
*/
inline cartesian_coordinates
cartesian_coordinates::operator+(cartesian_coordinates in)
{
  return cartesian_coordinates(mvec.x() + in.mvec(0), mvec.y() + in.mvec(1), mvec.z() + in.mvec(2));
}

/*!
  Returns the difference between this and \a in .
*/
inline cartesian_coordinates
cartesian_coordinates::operator-(cartesian_coordinates in)
{
  return cartesian_coordinates(mvec.x() - in.mvec(0), mvec.y() - in.mvec(1), mvec.z() - in.mvec(2));
}

/*!
  Computes a distance ratio of the length of this - \a to over the max length
  of this and \a to .
*/
double
cartesian_coordinates::distanceTo(coordinates to) const
{
  /* dist divided by max length of dist */
  double max_length = (mvec.norm() > to.mvec.norm() ? mvec.norm() : to.mvec.norm());
  return ((this->mvec - to.mvec).norm() / max_length) / 2.0;
}

/*
 * Methods for cylindrical representation
 */

/*!
  Initializes the coordinates with the values \a R , \a phi , \a z .
*/
cylindrical_coordinates::cylindrical_coordinates(double R, double phi, double z) : coordinates(R, phi, z)
{
  set_coord_system_type(cylindrical);
}

/*!
  Copies the values in \a cc .
*/
cylindrical_coordinates::cylindrical_coordinates(cylindrical_coordinates *cc) : coordinates(cc->x(), cc->y(), cc->z())
{
  set_coord_system_type(cylindrical);
}

/*!
  Copies the values in \a cc
*/
cylindrical_coordinates::cylindrical_coordinates(const cylindrical_coordinates &cc) : coordinates(cc.mvec.x(), cc.mvec.y(), cc.mvec.z())
{
  set_coord_system_type(cylindrical);
}

/*!
  Initializes the coordinates with the values from \a v .
*/
cylindrical_coordinates::cylindrical_coordinates(vec3 v) : coordinates(v)
{
  set_coord_system_type(cylindrical);
}

/*!
  Initializes the coordinates to 0,0,0.
*/
cylindrical_coordinates::cylindrical_coordinates() : coordinates()
{
  set_coord_system_type(cylindrical);
}

/*!
  Converts this to cartesian coordinates by computing x and y from radius and
  azimuth.
*/
cartesian_coordinates
cylindrical_coordinates::get_pos_cartesian() const
{
  double cx, cy;
  cx = mvec.x() * cos(mvec.y());   // x = R * cos(phi)
  cy = mvec.x() * sin(mvec.y());   // y = R * sin(phi)
  // z = z
  return cartesian_coordinates(cx, cy, mvec.z());
}


/*!
  Returns a copy of this.
*/
cylindrical_coordinates
cylindrical_coordinates::get_pos_cylindrical() const
{
  return cylindrical_coordinates(*this);
}

/*!
  Converts this to spherical coordinates by computing a new radius and
  elevation.
*/
spherical_coordinates
cylindrical_coordinates::get_pos_spherical() const
{
  double r, teta;
  r    = sqrt(mvec.x() * mvec.x() + mvec.z() * mvec.z());   // r = sqrt(R*R+z*z)
  teta = atan2(mvec.x(), mvec.z());               // teta = atan(R/z)
  return spherical_coordinates(r, teta, mvec.y()); // phi = phi
}

/*!

*/
cartesian_coordinates
cylindrical_coordinates::get_vec_cartesian(cylindrical_coordinates from) const
{
  double Ax, Ay;
  double phi = from.y();
  Ax = mvec.x() * cos(phi) - mvec.y() * sin(phi); /* Ax = Ar * cos(phi) - Aphi * sin(phi) */
  Ay = mvec.x() * sin(phi) + mvec.y() * cos(phi); /* Ay = Ar * sin(phi) + Aphi * cos(phi) */
  /* z = z */
  return cartesian_coordinates(Ax, Ay, mvec.z());
}

/*!

*/
cylindrical_coordinates
cylindrical_coordinates::get_vec_cylindrical(cylindrical_coordinates from) const
{
  Q_UNUSED(from);  // get rid of unused parameter warning
  return cylindrical_coordinates(*this);
}

/*!

*/
spherical_coordinates
cylindrical_coordinates::get_vec_spherical(cylindrical_coordinates from) const
{
  double Ar, Ateta;
  double teta = from.get_pos_spherical().y();
  Ar    = mvec.x() * sin(teta) + mvec.z() * cos(teta);  /* Ar_sph = Ar_cyl * sin(teta) + Az * cos(teta) */
  Ateta = mvec.x() * cos(teta) - mvec.z() * sin(teta);  /* Ateta  = Ar_cyl * cos(teta) - Az * (sin(teta) */
  // phi_sph = phi_cyl
  return spherical_coordinates(Ar, Ateta, mvec.y());
}


//  SEE HEADER FILE
//  inline cylindrical_coordinates
//  cylindrical_coordinates::operator+(cylindrical_coordinates in)
//  {
//      /* change to cartesian, add, change back */
//      return (get_cartesian() + in.get_cartesian()).get_cylindrical();
//  }
//  inline cylindrical_coordinates
//  cylindrical_coordinates::operator-(cylindrical_coordinates in)
//  {
//      /* change to cartesian, add, change back */
//      return (get_cartesian() - in.get_cartesian()).get_cylindrical();
//  }

/*!
  Converts both this and \a to to cartesian coordinates to compute distance
  measure.
*/
double
cylindrical_coordinates::distanceTo(cylindrical_coordinates to) const
{
  /* dist divided by max length of dist */
  return ((this->get_pos_cartesian() - to.get_pos_cartesian()).norm() /
          (get_pos_cartesian().norm() > to.get_pos_cartesian().norm() ?
           get_pos_cartesian().norm() : to.get_pos_cartesian().norm())) /
         2.0;
}

/*
 * Methods for spherical representation
 */

/*!
  Initializes the coordinates with the values \a R , \a teta , \a phi .
*/
spherical_coordinates::spherical_coordinates(double r, double teta, double phi) : coordinates(r, teta, phi)
{
  set_coord_system_type(spherical);
}

/*!
  Copies the values from \a cc.
*/
spherical_coordinates::spherical_coordinates(spherical_coordinates *cc) : coordinates(cc->x(), cc->y(), cc->z())
{
  set_coord_system_type(spherical);
}

/*!
  Copies the values from \a cc.
*/
spherical_coordinates::spherical_coordinates(const spherical_coordinates &cc) : coordinates(cc.mvec.x(), cc.mvec.y(), cc.mvec.z())
{
  set_coord_system_type(spherical);
}

/*!
  Initializes the coordinates with the values \a R , \a phi , \a z .
*/
spherical_coordinates::spherical_coordinates(vec3 v) : coordinates(v)
{
  set_coord_system_type(spherical);
}

/*!
  Initializes the coordinates with the values \a R , \a phi , \a z .
*/
spherical_coordinates::spherical_coordinates() : coordinates()
{
  set_coord_system_type(spherical);
}

/*!
  Converts this to cartesian coordinates.
*/
cartesian_coordinates
spherical_coordinates::get_pos_cartesian() const
{
  double cx, cy, cz;
  cx = mvec.x() * sin(mvec.y()) * cos(mvec.z()); // x = r * sin(teta) * cos(phi)
  cy = mvec.x() * sin(mvec.y()) * sin(mvec.z()); // y = r * sin(teta) * sin(phi)
  cz = mvec.x() * cos(mvec.y());            // z = r * cos(teta)
  return cartesian_coordinates(cx, cy, cz);
}

/*!
  Converts this to cylindrical coordinates.
*/
cylindrical_coordinates
spherical_coordinates::get_pos_cylindrical() const
{
  double R, cz;
  R = mvec.x() * sin(mvec.y());             // R = r * sin(teta)
  cz = mvec.x() * cos(mvec.y());             // z = r * cos(teta)
  // phi = phi
  return cylindrical_coordinates(R, mvec.z(), cz);
}

/*!
  Returns a copy of this.
*/
spherical_coordinates
spherical_coordinates::get_pos_spherical() const
{
  return spherical_coordinates(*this);
}

/*!

*/
cartesian_coordinates
spherical_coordinates::get_vec_cartesian(spherical_coordinates from) const
{
  double Ax, Ay, Az;
  double teta = from.y();
  double phi = from.z();
  Ax = mvec.x() * sin(teta) * cos(phi)
       + mvec.y() * cos(teta) * cos(phi)
       - mvec.z() * sin(phi);            /* Ax = Ar * sin(teta) * sin(phi) + Ateta * cos(teta) * cos(phi) - Aphi * sin(phi) */
  Ay = mvec.x() * sin(teta) * sin(phi)
       + mvec.y() * cos(teta) * sin(phi)
       + mvec.z() * cos(phi);            /* Ay = Ar * sin(teta) * sin(phi) + Ateta * cos(teta) * sin(phi) + Aphi * cos(phi) */
  Az = mvec.x() * cos(teta)
       - mvec.y() * sin(teta);           /* Az = Ar * cos(teta) - Ateta * sin(teta) */
  return cartesian_coordinates(Ax, Ay, Az);
}

/*!

*/
cylindrical_coordinates
spherical_coordinates::get_vec_cylindrical(spherical_coordinates from) const
{
  double Ar, Az;
  double teta = from.y();
  Ar = mvec.x() * sin(teta)
       + mvec.y() * cos(teta);   /* Ar_cyl = Ar_sph * sin(teta) + Ateta *cos(teta) */
  // Aphi_cyl = Aphi_sph
  Az = mvec.x() * cos(teta)
       - mvec.y() * sin(teta);   /* Az = Ar_sph * cos(teta) - Ateta * sin(teta) */
  return cylindrical_coordinates(Ar, mvec.z(), Az);
}

/*!
  Returns a copy of this.
*/
spherical_coordinates
spherical_coordinates::get_vec_spherical(spherical_coordinates from) const
{
  Q_UNUSED(from); // get rid of unused parameter warning
  return spherical_coordinates(*this);
}
//  SEE HEADER FILE
//  inline spherical_coordinates
//  spherical_coordinates::operator+(spherical_coordinates in)
//  {
//      /* change to cartesian, add, change back */
//      return (get_cartesian() + in.get_cartesian()).get_spherical();
//  }
//  inline spherical_coordinates
//  spherical_coordinates::operator-(spherical_coordinates in)
//  {
//      /* change to cartesian, add, change back */
//      return (get_cartesian() - in.get_cartesian()).get_spherical();
//  }

double
spherical_coordinates::distanceTo(spherical_coordinates to) const
{
  /* dist divided by max length of dist */
  return ((this->get_pos_cartesian() - to.get_pos_cartesian()).norm() /
          (get_pos_cartesian().norm() > to.get_pos_cartesian().norm() ?
           get_pos_cartesian().norm() : to.get_pos_cartesian().norm())) /
         2.0;
}



/******************
   Local Variables:
   mode:c++
   End:
******************/











