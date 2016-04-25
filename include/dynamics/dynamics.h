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


