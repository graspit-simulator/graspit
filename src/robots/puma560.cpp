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
// Author(s): Andrew T. Miller 
//
// $Id: puma560.cpp,v 1.4 2009/05/28 15:15:50 hao Exp $
//
//######################################################################

/*! \file
  \brief Implements the Puma560 robot subclass that has an analytic IK solution.
*/

#include "puma560.h"

const int Puma560::NUM_SOLUTIONS=8;

//! Structure used for Puma inverse kinematics.
/*! Holds the joint angles of a puma IK solution.  Angle12 is the sum joint
    angles 1 and 2.  Valid solutions are ones within the joint limits of the
    robot.
*/
struct Puma560Solution{
   double angle[6];
   double angle12;
   bool valid;
};


#define JOINT_DIST_EPSILON .05
int
Puma560::findClosestSol(Puma560Solution *candidates, Puma560Solution *current)
{
  unsigned char bits[Puma560::NUM_SOLUTIONS]=
    {0x1,0x2,0x4,0x08,0x10,0x20,0x40,0x80};
  unsigned char masks[Puma560::NUM_SOLUTIONS]=
    {0xFE,0xFD,0xFB,0xF7,0xEF,0xDF,0xBF,0x7F};

  unsigned char possible = 0xFF;
  unsigned char best;
  double shortest_distance,distance;
  bool one_solution;
  
  int i,j;
  
  // first ignore all unreachable solutions
  for(i=0; i < Puma560::NUM_SOLUTIONS; i++)
    if(candidates[i].valid != true) possible &= masks[i];
  
  // none are reachable so return Puma560::NUM_SOLUTIONS
  if(possible == 0)
    return(Puma560::NUM_SOLUTIONS);

  // for each joint find all closest solutions
  // Joints are travered from low to high because we want to
  // minimize movement in the lower order joints
  for(j=0; j < numDOF; j++) {
    shortest_distance = 6*M_PI;
    best = 0;
    for(i=0; i < Puma560::NUM_SOLUTIONS; i++) {
      if(possible & bits[i]) {
	distance = fabs(candidates[i].angle[j] - current->angle[j]);
	if(fabs(distance - shortest_distance) < JOINT_DIST_EPSILON) {
	  best |= bits[i];
	  one_solution = false;
	}
	else if(distance < shortest_distance) {
	  shortest_distance = distance;
	  best = bits[i];
	  one_solution = true;
	}
      }
    }
    possible = best;
    if(one_solution == true) break;
  }
  
  // return the first solution which is the closest
  for(i=0; i < Puma560::NUM_SOLUTIONS; i++)
    if(possible & bits[i]) return(i);
  
  // if no solutions match return Puma560::NUM_SOLUTIONS to
  // signify no solution possible
  // NOTE -- WE SHOULD NEVER GET HERE!!! this case
  // should have been caught above the loop
  return(Puma560::NUM_SOLUTIONS); 
}

int
Puma560::invKinematics(const transf& endTranLocal,double* dofVals,int)
{
	transf endTran = endTranLocal * base->getTran().inverse();
  int i;
  const double a2=431.8,a3=20.3;
  const double d3=149.0,d4=431.8;
  mat3 rot;
  endTran.rotation().ToRotationMatrix(rot);

  double r21 = rot.element(1,0);
  double r22 = rot.element(1,1);
  double r23 = rot.element(1,2);

  double r31 = rot.element(2,0);
  double r32 = rot.element(2,1);  
  double r33 = rot.element(2,2);

  double px = endTran.translation()[0];
  double py = endTran.translation()[1];
  double pz = endTran.translation()[2];

  double r = sqrt(px*px + py*py);
  double r1;

  double num,den,Psi,V114,V113,V323,V313,V112,V132,V312,V332,V412,V432;
  Puma560Solution sol[Puma560::NUM_SOLUTIONS];
  
  for(i=0; i < Puma560::NUM_SOLUTIONS; i++) {
     sol[i].valid = true;
     
     // theta 1
     if (i > 3)
       sol[i].angle[0] = atan2(py,px) + asin(d3/r);
     else
       sol[i].angle[0] = atan2(py,px) + M_PI - asin(d3/r);

     // theta 2
     V114 = px*cos(sol[i].angle[0]) + py*sin(sol[i].angle[0]);
     r1=sqrt(V114*V114 + pz*pz);
     Psi = acos((a2*a2 - d4*d4 - a3*a3 + V114*V114 + pz*pz)/(2.0*a2*r1));
     if (i%4 == 0 || i%4 == 1)
       sol[i].angle[1] = atan2(pz,V114) - Psi;
     else
       sol[i].angle[1] = atan2(pz,V114) + Psi;

     // theta 3
     num = cos(sol[i].angle[1])*V114 + sin(sol[i].angle[1])*pz - a2;
     den = cos(sol[i].angle[1])*pz - sin(sol[i].angle[1])*V114;
     sol[i].angle[2] = atan2(a3,d4) - atan2(num,den);

     // theta 4
     V113 = cos(sol[i].angle[0])*r31 + sin(sol[i].angle[0])*r32;
     V323 = cos(sol[i].angle[0])*r32 - sin(sol[i].angle[0])*r31;
     V313 = cos(sol[i].angle[1] + sol[i].angle[2])*V113 +
            sin(sol[i].angle[1] + sol[i].angle[2])*r33;
     if (i % 2)
       sol[i].angle[3] = atan2(V323,V313);
     else
       sol[i].angle[3] = atan2(-V323,-V313);

     // theta 5
     num = -cos(sol[i].angle[3])*V313 - V323*sin(sol[i].angle[3]);
     den = -V113*sin(sol[i].angle[1] + sol[i].angle[2]) + 
             r33*cos(sol[i].angle[1] + sol[i].angle[2]);
     sol[i].angle[4] = atan2(num,den);

     //theta 6
     V112 = cos(sol[i].angle[0])*r21 + sin(sol[i].angle[0])*r22;
     V132 = sin(sol[i].angle[0])*r21 - cos(sol[i].angle[0])*r22;
     V312 =  V112*cos(sol[i].angle[1]+sol[i].angle[2]) +
              r23*sin(sol[i].angle[1]+sol[i].angle[2]);
     V332 = -V112*sin(sol[i].angle[1]+sol[i].angle[2]) +
              r23*cos(sol[i].angle[1]+sol[i].angle[2]);
     V412 = V312*cos(sol[i].angle[3]) - V132*sin(sol[i].angle[3]);
     V432 = V312*sin(sol[i].angle[3]) + V132*cos(sol[i].angle[3]);
     num = -V412*cos(sol[i].angle[4]) - V332*sin(sol[i].angle[4]);
     den = -V432;
     sol[i].angle[5] = atan2(num,den);     
  }
     
       
  int j;
  
  /* invalidate solutions which are not possible */
  for(j=0; j < Puma560::NUM_SOLUTIONS; j++) {
    for(i=0;i<numDOF;i++) {
      if(sol[j].angle[i] < dofVec[i]->getMin() && sol[j].angle[i] +(2*M_PI)>
	 dofVec[i]->getMax()) {
	sol[j].valid = false;
	break;
      }
      else if (sol[j].angle[i] < dofVec[i]->getMin())
	sol[j].angle[i] += 2*M_PI;
      if(sol[j].angle[i] > dofVec[i]->getMax() && sol[j].angle[i] -(2*M_PI)< 
	 dofVec[i]->getMin()) {
	sol[j].valid = FALSE;
	break;
      }
      else if (sol[j].angle[i] > dofVec[i]->getMax())
	sol[j].angle[i] -= 2*M_PI;
    }
  }

#ifdef PUMADBG
  for(j=0; j < Puma560::NUM_SOLUTIONS; j++)
  {
     fprintf(stderr,"solution %d = ", j);

     for(i=0;i<numDOF;i++)
         fprintf(stderr,"%f ",sol[j].angle[i]);
     fprintf(stderr,"\n");

  }
#endif

  Puma560Solution current;
  for(i=0;i<numDOF;i++)
     current.angle[i] = dofVec[i]->getVal();

  int chosen = findClosestSol(sol, &current);

  if(chosen == Puma560::NUM_SOLUTIONS) return FAILURE;

#ifdef PUMADBG
  fprintf(stderr,"chosen = %d\n",chosen);
#endif

  for (i=0;i<numDOF;i++)
    dofVals[i] = sol[chosen].angle[i];

  // check big motions?
  return SUCCESS;
}
