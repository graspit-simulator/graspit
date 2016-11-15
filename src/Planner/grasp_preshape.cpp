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
// $Id: grasp_preshape.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_preshape.cc                                        */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the class preshape , a pre-grasp hand shape (used in the grasp planner)
*/

#include <math.h>
#include <iostream>

#include "grasp_preshape.h"

/*!
  Sets the preshape to PR_None and sets the initial angle values.
*/
preshape::preshape(){
    pType = PR_None;
    updateAngles();
}

/*!
  Initialized the preshape type to \a p and updates the DOF values.
*/
preshape::preshape(preshapeType p){
    pType = p;
    updateAngles();
}

/*!
  Copies preshape \a p .
*/
preshape::preshape(const preshape& p){
    pType = p.get_preshapeType();
    p.get_preshape(a, f1, f2, f3);
    updateAngles();
}

/*!    
  Sets the preshape type to \a p and updates the DOF values.
*/
void 
preshape::set_preshapeType(preshapeType p){
    pType = p;
    updateAngles();
}

/*!
  Sets the preshape type to PR_None and sets the DOF values directly.
*/
void 
preshape::set_preshape(double aa, double ff1, double ff2, double ff3){

  pType = PR_None;
  a=aa; f1 = ff1; f2 = ff2; f3=ff3;
}

/*!
  Returns the preshape type.
*/
preshapeType
preshape::get_preshapeType() const{
    return pType;
}

/*!
  Returns the preshape DOF values.
*/
void 
preshape::get_preshape(double& aa, double& ff1, double& ff2, double& ff3) const{
    aa = a;
    ff1 = f1;
    ff2 = f2;
    ff3 = f3;
    return;
}

/*!
  Computes a scalar distance between this preshape and preshape \a p.  
  Since all preshapes currently only set the spread angle, this is
  simply computed as abs(a - p.a) / max(a, p.a) .
*/
double 
preshape::distanceTo(preshape p) const{
    double aa,ff1,ff2,ff3;
    p.get_preshape(aa,ff1,ff2,ff3);
    if((aa*aa+ff1*ff1+ff2*ff2+ff3*ff3) == 0.0 &&
       (a*a  +f1*f1  +f2*f2  +f3*f3) == 0.0)
	return 0.0;
//      return (sqrt((aa-a)*(aa-a)+
//  		 (ff1-f1)*(ff1-f1)+
//  		 (ff2-f2)*(ff2-f2)+
//  		 (ff3-f3)*(ff3-f3)) / 
//  	((aa*aa+ff1*ff1+ff2*ff2+ff3*ff3) >
//  	 (a*a  +f1*f1  +f2*f2  +f3*f3)   ?
//  	 (aa*aa+ff1*ff1+ff2*ff2+ff3*ff3) :
//  	 (a*a  +f1*f1  +f2*f2  +f3*f3)));
    /* Should be a-aa/max here, but max doesnt exist in this class 
       as well as the hand and I'm too lazy to implement that
       now... */
    return fabs((a-aa)/(a>aa?a:aa));
}

/*!
  Uses the preshape type to set the DOF values.
*/
void
preshape::updateAngles(){
    if (pType != PR_None){
	switch (pType) {
	    case PR_circle:
		a  = PR_circle_Angle;
		f1 = PR_circle_F1;
		f2 = PR_circle_F2;
		f3 = PR_circle_F3;
		break;
	    case PR_two_opp_one:
		a =  PR_two_opp_one_Angle;
		f1 = PR_two_opp_one_F1;
		f2 = PR_two_opp_one_F2;
		f3 = PR_two_opp_one_F3;
		break;
	    case PR_three_opp_palm:
		a =  PR_three_opp_palm_Angle;
		f1 = PR_three_opp_palm_F1;
		f2 = PR_three_opp_palm_F2;
		f3 = PR_three_opp_palm_F3;
		break;
	    case PR_t_shape:
		a =  PR_t_shape_Angle;
		f1 = PR_t_shape_F1;
		f2 = PR_t_shape_F2;
		f3 = PR_t_shape_F3;
		break;
	    default:
		std::cout << "Preshape not defined or implemented." << std::endl;
		break;
	}
    }
}















