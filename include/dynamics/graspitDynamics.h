//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2015  Columbia University in the City of New York.
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
// $Id:$
//
//######################################################################

#ifndef GRASPITDYNAMICS_HXX
#define GRASPITDYNAMICS_HXX

#include "dynamicsEngine.h"

class GraspitDynamics : public DynamicsEngine {
 public:
  explicit GraspitDynamics(World *world);
  ~GraspitDynamics();
  void addBody(Body *newBody);
  void addRobot(Robot *robot);
  void turnOnDynamics();
  void turnOffDynamics();
  int stepDynamics();
  double moveDynamicBodies(double timeStep);
  int computeNewVelocities(double timeStep);
 private:
  World *mWorld;
};

#endif
