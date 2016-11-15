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
// $Id: grasp_preshape.h,v 1.2 2009/03/25 22:10:24 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_preshape.h                                         */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the class preshape , a pre-grasp hand shape (used in the grasp planner)
*/

#ifndef GRASP_PRESHAPE_H
#define GRASP_PRESHAPE_H

/* finger angles of preshape types for
   BARRETT HAND */
#define PR_circle_Angle 60.0
#define PR_circle_F1 0.0
#define PR_circle_F2 0.0
#define PR_circle_F3 0.0

#define PR_two_opp_one_Angle 0.0
#define PR_two_opp_one_F1 0.0
#define PR_two_opp_one_F2 0.0
#define PR_two_opp_one_F3 0.0

#define PR_three_opp_palm_Angle 180.0
#define PR_three_opp_palm_F1 0.0
#define PR_three_opp_palm_F2 0.0
#define PR_three_opp_palm_F3 0.0

#define PR_t_shape_Angle 90.0
#define PR_t_shape_F1 0.0
#define PR_t_shape_F2 0.0
#define PR_t_shape_F3 0.0

/*! The 4 main preshape types for the Barrett hand are:
  - \b PR_two_opp_one where the spread angle is 0, so that 2 fingers oppose the thumb
  - \b PR_circle where the spread angle is 60 degrees resulting in 120 degrees between each finger
  - \b PR_t_shape where the spread angle is 90 so that the 2 fingers oppose each other directly and the thumb is 90 degrees away from each of them.
  - \b PR_three_opp_palm where the spread angle is 180, so that all 3 fingers are together and oppose the palm
*/
enum preshapeType{PR_None, PR_circle, PR_two_opp_one, PR_three_opp_palm, PR_t_shape};

//! Holds the grasp preshape type and the DOF values for that presahpe.
/*!
  For the Barrett hand there are currently 4 defined preshapes.  Each one
  is defined by its DOF values.  Right now, the 4 shapes are only
  distinguished by the spread angle of the fingers.
*/
class preshape
{
private:
  //! Spread angle for Barrett Hand in degrees (DOF #0)
  double a;

  //! Flexion of finger 1 of Barrett Hand in degrees (DOF #1)
  double f1;

  //! Flexion of finger 2 of Barrett Hand in degrees (DOF #2)
  double f2;
  
  //! Flexion of finger 3 of Barrett Hand in degrees (DOF #3)
  double f3;

  //! The current preshape 
  preshapeType pType;

    void updateAngles();

public:

    preshape();
    preshape(preshapeType);
    preshape(const preshape&);
    
    void set_preshapeType(preshapeType);
    void set_preshape(double, double, double, double);

    preshapeType get_preshapeType()   const;   
    void get_preshape(double&, double&, double&, double&) const;

    double distanceTo(preshape) const;
};


#endif


/******************
   Local Variables:
   mode:c++
   End:
******************/
