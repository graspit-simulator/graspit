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
// Author(s): Matei T. Ciocarlie
//
// $Id: jacobian.cpp,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#include "jacobian.h"
#include <math.h>

void jacobian(double t1, double t2, double t3, double t4, double px, double py, double pz, double *J)
{
	double dxdt1 = -52.11*cos(t2)*sin(t1) - 40.76*cos(t2)*cos(t3)*sin(t1) + py*(0.7193709695451316*cos(t1) - 0.6946260923516316*sin(t1)*sin(t2)) + 
  40.76*(0.6946260923516316*cos(t1) + 0.7193709695451316*sin(t1)*sin(t2))*sin(t3) + 
  px*(cos(t4)*((0.6946260923516316*cos(t1) + 0.7193709695451316*sin(t1)*sin(t2))*sin(t3) - cos(t2)*cos(t3)*sin(t1)) + 
    (cos(t3)*(0.6946260923516316*cos(t1) + 0.7193709695451316*sin(t1)*sin(t2)) + cos(t2)*sin(t1)*sin(t3))*sin(t4)) + 
  pz*(((0.6946260923516316*cos(t1) + 0.7193709695451316*sin(t1)*sin(t2))*sin(t3) - cos(t2)*cos(t3)*sin(t1))*sin(t4) - 
    cos(t4)*(cos(t3)*(0.6946260923516316*cos(t1) + 0.7193709695451316*sin(t1)*sin(t2)) + cos(t2)*sin(t1)*sin(t3)));

	double dxdt2 = 0.6946260923516316*py*cos(t1)*cos(t2) - 29.321560718659562*cos(t1)*sin(t3)*cos(t2) - 52.11*cos(t1)*sin(t2) - 40.76*cos(t1)*cos(t3)*sin(t2) + 
  pz*(((-cos(t1))*cos(t3)*sin(t2) - 0.7193709695451316*cos(t1)*cos(t2)*sin(t3))*sin(t4) - 
    cos(t4)*(cos(t1)*sin(t2)*sin(t3) - 0.7193709695451316*cos(t1)*cos(t2)*cos(t3))) + 
  px*(cos(t4)*((-cos(t1))*cos(t3)*sin(t2) - 0.7193709695451316*cos(t1)*cos(t2)*sin(t3)) + (cos(t1)*sin(t2)*sin(t3) - 0.7193709695451316*cos(t1)*cos(t2)*cos(t3))*
     sin(t4));

	double dxdt3 = 40.76*cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - 40.76*cos(t1)*cos(t2)*sin(t3) + 
  pz*((cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3))*sin(t4) - 
    cos(t4)*((-cos(t1))*cos(t2)*cos(t3) - (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3))) + 
  px*(cos(t4)*(cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3)) + 
    ((-cos(t1))*cos(t2)*cos(t3) - (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3))*sin(t4));

	double dxdt4 = pz*(cos(t4)*(cos(t1)*cos(t2)*cos(t3) + (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3)) + 
    (cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3))*sin(t4)) + 
  px*(cos(t4)*(cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3)) - 
    (cos(t1)*cos(t2)*cos(t3) + (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3))*sin(t4));

	double dydt1 = 52.11*cos(t1)*cos(t2) + 40.76*cos(t1)*cos(t3)*cos(t2) + py*(0.7193709695451316*sin(t1) + 0.6946260923516316*cos(t1)*sin(t2)) + 
  40.76*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3) + 
  px*(cos(t4)*(cos(t1)*cos(t2)*cos(t3) + (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3)) + 
    (cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3))*sin(t4)) + 
  pz*((cos(t1)*cos(t2)*cos(t3) + (0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2))*sin(t3))*sin(t4) - 
    cos(t4)*(cos(t3)*(0.6946260923516316*sin(t1) - 0.7193709695451316*cos(t1)*sin(t2)) - cos(t1)*cos(t2)*sin(t3)));

	double dydt2 = 0.6946260923516316*py*cos(t2)*sin(t1) - 40.76*cos(t3)*sin(t2)*sin(t1) - 52.11*sin(t2)*sin(t1) - 29.321560718659562*cos(t2)*sin(t3)*sin(t1) + 
  pz*(((-cos(t3))*sin(t1)*sin(t2) - 0.7193709695451316*cos(t2)*sin(t1)*sin(t3))*sin(t4) - 
    cos(t4)*(sin(t1)*sin(t2)*sin(t3) - 0.7193709695451316*cos(t2)*cos(t3)*sin(t1))) + 
  px*(cos(t4)*((-cos(t3))*sin(t1)*sin(t2) - 0.7193709695451316*cos(t2)*sin(t1)*sin(t3)) + (sin(t1)*sin(t2)*sin(t3) - 0.7193709695451316*cos(t2)*cos(t3)*sin(t1))*
     sin(t4));

	double dydt3 = 40.76*cos(t3)*(-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2)) - 40.76*cos(t2)*sin(t1)*sin(t3) + 
  pz*((cos(t3)*(-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2)) - cos(t2)*sin(t1)*sin(t3))*sin(t4) - 
    cos(t4)*((-cos(t2))*cos(t3)*sin(t1) - (-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2))*sin(t3))) + 
  px*(cos(t4)*(cos(t3)*(-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2)) - cos(t2)*sin(t1)*sin(t3)) + 
    ((-cos(t2))*cos(t3)*sin(t1) - (-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2))*sin(t3))*sin(t4));

	double dydt4 = pz*(cos(t4)*(cos(t2)*cos(t3)*sin(t1) + (-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2))*sin(t3)) + 
    (cos(t3)*(-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2)) - cos(t2)*sin(t1)*sin(t3))*sin(t4)) + 
  px*(cos(t4)*(cos(t3)*(-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2)) - cos(t2)*sin(t1)*sin(t3)) - 
    (cos(t2)*cos(t3)*sin(t1) + (-0.6946260923516316*cos(t1) - 0.7193709695451316*sin(t1)*sin(t2))*sin(t3))*sin(t4));

	double dzdt1 = 0;

	double dzdt2 = 40.76*cos(t3)*cos(t2) + 52.11*cos(t2) + 0.6946260923516316*py*sin(t2) - 29.321560718659562*sin(t2)*sin(t3) + 
  px*(cos(t4)*(cos(t2)*cos(t3) - 0.7193709695451316*sin(t2)*sin(t3)) + (-0.7193709695451316*cos(t3)*sin(t2) - cos(t2)*sin(t3))*sin(t4)) + 
  pz*((cos(t2)*cos(t3) - 0.7193709695451316*sin(t2)*sin(t3))*sin(t4) - cos(t4)*(-0.7193709695451316*cos(t3)*sin(t2) - cos(t2)*sin(t3)));

	double dzdt3 = 29.321560718659562*cos(t2)*cos(t3) - 40.76*sin(t2)*sin(t3) + px*(cos(t4)*(0.7193709695451316*cos(t2)*cos(t3) - sin(t2)*sin(t3)) + 
    ((-cos(t3))*sin(t2) - 0.7193709695451316*cos(t2)*sin(t3))*sin(t4)) + pz*((0.7193709695451316*cos(t2)*cos(t3) - sin(t2)*sin(t3))*sin(t4) - 
    cos(t4)*((-cos(t3))*sin(t2) - 0.7193709695451316*cos(t2)*sin(t3)));

	double dzdt4 = px*(cos(t4)*(0.7193709695451316*cos(t2)*cos(t3) - sin(t2)*sin(t3)) - (cos(t3)*sin(t2) + 0.7193709695451316*cos(t2)*sin(t3))*sin(t4)) + 
  pz*(cos(t4)*(cos(t3)*sin(t2) + 0.7193709695451316*cos(t2)*sin(t3)) + (0.7193709695451316*cos(t2)*cos(t3) - sin(t2)*sin(t3))*sin(t4));

	J[0] = dxdt1;   J[3] = dxdt2;   J[6] = dxdt3;   J[9]  = dxdt4;
	J[1] = dydt1;   J[4] = dydt2;   J[7] = dydt3;   J[10] = dydt4;
	J[2] = dzdt1;   J[5] = dzdt2;   J[8] = dzdt3;   J[11] = dzdt4;
}

void compute_dTdG(double s0, double s1, double s2, double s3, double *dTdG)
{
	dTdG[0] = s0;	dTdG[4] = s1;	dTdG[8] =   0;	dTdG[12] =  0;	dTdG[16] =  0;	dTdG[20] =  0;
	dTdG[1] =  0;	dTdG[5] =  0;	dTdG[9] =  s0;	dTdG[13] = s1;	dTdG[17] =  0;	dTdG[21] =  0;
	dTdG[2] =  0;	dTdG[6] =  0;	dTdG[10] =  0;	dTdG[14] =  0;	dTdG[18] = s2;	dTdG[22] =  0;
	dTdG[3] =  0;	dTdG[7] =  0;	dTdG[11] =  0;	dTdG[15] =  0;	dTdG[19] =  0;	dTdG[23] = s3;
}
