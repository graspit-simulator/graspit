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
// Author(s):  Andrew T. Miller 
//
// $Id: dynamics.h,v 1.7 2009/03/30 14:59:55 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Prototypes for moveBodies and iterateDynamics.
  This is the heart of the dynamics engine, where all computations are performed.
  The design is fairly old and had not been update to an object-oriented more
  modular design, so the functions are hard to read through. However, they
  get the job done.
 */

class DynamicBody;
class Contact;
class Robot;

//original GraspIt: 0.2
#define ERP 0.05

typedef struct StructDynamicParameters {
	double timeStep;
	bool useContactEps;
	double gravityMultiplier;
} DynamicParameters;

//! Moves all dynamic bodies for one time step
int
moveBodies(int numBodies,std::vector<DynamicBody *> bodyVec,double h);

//! Computes the new velocites of all bodies, based on contact and joint constraints
int
iterateDynamics(std::vector<Robot *> robotVec,
		std::vector<DynamicBody *> bodyVec,
		DynamicParameters *dp);


