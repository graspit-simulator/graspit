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
// Author(s): Matei T. Ciocarlie, Jake Varley
//
// $Id:$
//
//######################################################################

#ifndef BULLETDYNAMICS_HXX
#define BULLETDYNAMICS_HXX

#include <LinearMath/btAlignedObjectArray.h>

#include "graspit/dynamicsEngine.h"
#include "graspit/joint.h"

class btDiscreteDynamicsWorld;
class btRigidBody;
class btHingeConstraint;
class World;
class KinematicChain;

class BulletDynamics : public DynamicsEngine {
  public:
    explicit BulletDynamics(World *world);
    ~BulletDynamics();
    void addBody(Body *newBody);
    void addRobot(Robot *robot);
    void addChain(KinematicChain *chain, btRigidBody *btbase);
    void turnOnDynamics();
    void turnOffDynamics();
    int stepDynamics();
    double moveDynamicBodies(double timeStep);
    int computeNewVelocities(double timeStep);
    void btApplyInternalWrench(Joint *activeJoint, double magnitude, std::map<Body *, btRigidBody *> btBodyMap);

  private:
    World *mWorld;

    btDiscreteDynamicsWorld *mBtDynamicsWorld;
    btAlignedObjectArray<btRigidBody *> mBtLinks;

    typedef std::pair<Body *, btRigidBody *> btBodyPair;
    std::map<Body *, btRigidBody *> btBodyMap;

    typedef std::pair<Body *, btHingeConstraint *> btJointPair;
    std::map<Joint *, btHingeConstraint *> btJointMap;
};

#endif
