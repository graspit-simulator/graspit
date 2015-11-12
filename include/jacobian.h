//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: jacobian.h,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#ifndef _jacobian_h_
#define _jacobian_h_

/*! \file
	A hard-coded jacobian and derivative for the thumb of the human 
	hand, computed in Mathematica and then translated here. Used 
	only for CyberGlove thumb calibration. 

	This file needs a less generic name - this is only a very 
	specific thing for the human hand and glove calibration. More
	general Jacobian computations can be found in the Grasp class.
*/

void jacobian(double t1, double t2, double t3, double t4, double px, double py, double pz, double *J);
void compute_dTdG(double s0, double s1, double s2, double s3, double *dTdG);

#endif
