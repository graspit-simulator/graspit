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
// Author(s): Matei T. Ciocarlie, Mengyu Wu
//
// $Id:$
//
//######################################################################

#include "graspit/dynamics/graspitDynamics.h"

#include "graspit/body.h"
#include "graspit/dynamics/dynJoint.h"
#include "graspit/robot.h"
#include "graspit/triangle.h"
#include "graspit/world.h"

#include "graspit/debug.h"
#include "graspit/dynamics/dynamics.h"
#include "graspit/robots/humanHand.h"
GraspitDynamics::GraspitDynamics(World *world) {
  mWorld = world;
}

GraspitDynamics::~GraspitDynamics() {
}

void GraspitDynamics::addBody(Body *newBody) {
  Q_UNUSED(newBody);
}

void GraspitDynamics::addRobot(Robot *robot) {
  Q_UNUSED(robot);
}


void GraspitDynamics::turnOnDynamics() {
}

void GraspitDynamics::turnOffDynamics() {
}

/*! One of the two main functions of the dynamics time step. This function is
called to move the bodies according to the velocities and accelerations found
in the previous step. The move is attempted for the duration of the time step
given in \a timeStep.

After all the bodies are moved, then collisions are checked. If any collisions
are found, the move is traced back and the value of the times step is
interpolated until the exact moment of contact is found. The actual value
of the time step until contact is made is returned. If interpolation fails,
a negative actual time step is returned. All the resulting contacts are added
to the bodies that are in contact to be used for subsequent computations.

The same procedure is carried out if, by executing a full time step, a joint
ends up outside of its legal range.
*/
double GraspitDynamics::moveDynamicBodies(double timeStep) {
  int i, numDynBodies, numCols, moveErrCode;
  std::vector<DynamicBody *> dynBodies;
  static CollisionReport colReport;
  bool jointLimitHit;
  double contactTime, delta, tmpDist, minDist, dofLimitDist;

  int numBodies = mWorld->getNumBodies();
  int numRobots = mWorld->getNumRobots();

  // save the initial position
  for (i = 0; i < numBodies; i++) {
    if (mWorld->getBody(i)->isDynamic()) {
      dynBodies.push_back((DynamicBody *)mWorld->getBody(i));
      ((DynamicBody *)mWorld->getBody(i))->markState();
    }
  }
  numDynBodies = dynBodies.size();

  // call to the dynamics engine to perform the move by the full time step
  DBGP("moving bodies with timestep: " << timeStep);
  moveErrCode = moveBodies(numDynBodies, dynBodies, timeStep);
  if (moveErrCode == 1) {  // object out of bounds
    mWorld->popDynamicState();
    turnOffDynamics();
    return -1.0;
  }

  // this sets the joints internal values according to how bodies have moved
  for (i = 0; i < numRobots; i++) {
    mWorld->getRobot(i)->updateJointValuesFromDynamics();
  }

  // check if we have collisions
  if (numDynBodies > 0) { numCols = mWorld->getCollisionReport(&colReport); }
  else { numCols = 0; }

  // check if we have joint limits exceeded
  jointLimitHit = false;
  for (i = 0; i < numRobots; i++) {
    if (mWorld->getRobot(i)->jointLimitDist() < 0.0) { jointLimitHit = true; }
  }

  // if so, we must interpolate until the exact moment of contact or limit hit
  if (numCols || jointLimitHit) {
    // return to initial position
    for (i = 0; i < numDynBodies; i++) {
      dynBodies[i]->returnToMarkedState();
    }
    minDist = 1.0e+255;
    dofLimitDist = 1.0e+255;

#ifdef GRASPITDBG
    if (numCols) {
      std::cout << "COLLIDE!" << std::endl;
      for (i = 0; i < numCols; i++) {
        std::cout << colReport[i].first->getName().toStdString().c_str() << " collided with " <<
                  colReport[i].second->getName().toStdString().c_str() << std::endl;
      }

      for (i = 0; i < numCols; i++) {
        tmpDist = mWorld->getDist(colReport[i].first, colReport[i].second);
        if (tmpDist < minDist) { minDist = tmpDist; }
        std::cout << "minDist: " << tmpDist << " between " << std::endl;
        std::cout << colReport[i].first->getName().toStdString().c_str() << " and " <<
                  colReport[i].second->getName().toStdString().c_str() << std::endl;
      }
    }
#endif

    // this section refines the timestep until the objects are separated
    // by a distance less than CONTACT_THRESHOLD
    bool done = false;
    contactTime = timeStep;
    delta = contactTime / 2;
    contactTime -= delta;

    while (!done) {
      delta /= 2.0;
      for (i = 0; i < numDynBodies; i++) {
        dynBodies[i]->returnToMarkedState();
      }
      DBGP("moving bodies with timestep: " << contactTime);
      moveErrCode = moveBodies(numDynBodies, dynBodies, contactTime);

      if (moveErrCode == 1) {  // object out of bounds
        mWorld->popDynamicState();
        turnOffDynamics();
        return -1.0;
      }

      const char *min_body_1, *min_body_2;

      // this computes joints values according to how dynamic bodies have moved
      for (i = 0; i < numRobots; i++) {
        mWorld->getRobot(i)->updateJointValuesFromDynamics();
      }

      if (numCols) {
        minDist = 1.0e+255;
        for (i = 0; i < numCols; i++) {
          tmpDist = mWorld->getDist(colReport[i].first, colReport[i].second);
          if (tmpDist < minDist) {
            minDist = tmpDist;
            min_body_1 = colReport[i].first->getName().toStdString().c_str();
            min_body_2 = colReport[i].second->getName().toStdString().c_str();
            DBGP("minDist: " << minDist << " between " << colReport[i].first->getName().toStdString().c_str() <<
                 " and " << colReport[i].second->getName().toStdString().c_str());
          }
        }
      }


      if (jointLimitHit) {
        dofLimitDist = 1.0e10;
        for (i = 0; i < numRobots; i++) {
          dofLimitDist = MIN(dofLimitDist, mWorld->getRobot(i)->jointLimitDist());
        }
      }

      if (minDist <= 0.0 || dofLimitDist < -resabs) {
        contactTime -= delta;
      }
      else if (minDist > Contact::THRESHOLD * 0.5 && dofLimitDist > 0.01) { // why is this not resabs
        contactTime += delta;
      }
      else { break; }

      if (fabs(delta) < 1.0E-15 || contactTime < 1.0e-7) {
        if (minDist <= 0) {
          fprintf(stderr, "Delta failsafe due to collision: %s and %s\n", min_body_1, min_body_2);
        } else {
          fprintf(stderr, "Delta failsafe due to joint\n");
        }
        done = true;  // failsafe
      }
    }

    // COULD NOT FIND COLLISION TIME
    if (done && contactTime < 1.0E-7) {
      DBGP("!! could not find contact time !!");
      for (i = 0; i < numDynBodies; i++) {
        dynBodies[i]->returnToMarkedState();
      }
    }
    mWorld->getWorldTimeRef() += contactTime;
  } else {  // if no collision
    mWorld->getWorldTimeRef() += timeStep;
    contactTime = timeStep;
  }

#ifdef GRASPITDBG
  std::cout << "CHECKING COLLISIONS AT MIDDLE OF STEP: ";
  numCols = mWorld->getCollisionReport(&colReport);

  if (!numCols) {
    std::cout << "None." << std::endl;
  } else {
    std::cout << numCols << " found!!!" << std::endl;
    for (i = 0; i < numCols; i++) {
      std::cout << colReport[i].first->getName().toStdString().c_str() << " collided with " <<
                colReport[i].second->getName().toStdString().c_str() << std::endl;
    }
  }
#endif

  if (numDynBodies > 0) {
    mWorld->findAllContacts();
  }

  for (i = 0; i < numRobots; i++) {
    if (mWorld->getRobot(i)->inherits("HumanHand")) { ((HumanHand *)mWorld->getRobot(i))->updateTendonGeometry(); }
    mWorld->getRobot(i)->emitConfigChange();
  }
  mWorld->tendonChange();

  if (contactTime < 1.0E-7) { return -1.0; }
  return contactTime;
}


/*! Asks the dynamics engine to compute the velocities of all bodies at
the current time step. These will be used in the next time step when
the bodies are moved by World::moveDynamicBodies.

The bodies are separated into "islands" of bodies connected by contacts
or joints.  Two dynamic bodies are connected if they share a contact or
a joint.  Then for each island, this calls the iterate dynamics routine
to build and solve the LCP to find the velocities of all the bodies
in the next iteration.
*/
int GraspitDynamics::computeNewVelocities(double timeStep) {
  bool allDynamicsComputed;
  static std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  std::vector<DynamicBody *> robotLinks;
  std::vector<DynamicBody *> dynIsland;
  std::vector<Robot *> islandRobots;
  int i, j, numLinks, numDynBodies, lemkeErrCode;

  int numBodies = mWorld->getNumBodies();

#ifdef GRASPITDBG
  int islandCount = 0;
#endif

  do {
    // seed the island with one dynamic body
    for (i = 0; i < numBodies; i++)
      if (mWorld->getBody(i)->isDynamic() &&
          !((DynamicBody *)mWorld->getBody(i))->dynamicsComputed()) {
        // if this body is a link, add all robots connected to the link
        if (mWorld->getBody(i)->inherits("Link")) {
          Robot *robot = ((Robot *)((Link *)mWorld->getBody(i))->getOwner())->getBaseRobot();
          robot->getAllLinks(dynIsland);
          robot->getAllAttachedRobots(islandRobots);
        } else {
          dynIsland.push_back((DynamicBody *)mWorld->getBody(i));
        }
        break;
      }
    numDynBodies = dynIsland.size();
    for (i = 0; i < numDynBodies; i++) {
      dynIsland[i]->setDynamicsFlag();
    }

    // add any bodies that contact any body already in the dynamic island
    for (i = 0; i < numDynBodies; i++) {
      contactList = dynIsland[i]->getContacts();
      for (cp = contactList.begin(); cp != contactList.end(); cp++) {
        // if the contacting body is dynamic and not already in the list, add it
        if ((*cp)->getBody2()->isDynamic() &&
            !((DynamicBody *)(*cp)->getBody2())->dynamicsComputed()) {
          DynamicBody *contactedBody = (DynamicBody *)(*cp)->getBody2();

          // is this body is a link, add all robots connected to the link
          if (contactedBody->isA("Link")) {
            Robot *robot = ((Robot *)((Link *)contactedBody)->getOwner())->getBaseRobot();
            robot->getAllLinks(robotLinks);
            robot->getAllAttachedRobots(islandRobots);
            numLinks = robotLinks.size();
            for (j = 0; j < numLinks; j++)
              if (!robotLinks[j]->dynamicsComputed()) {
                dynIsland.push_back(robotLinks[j]);
                robotLinks[j]->setDynamicsFlag();
                numDynBodies++;
              }
            robotLinks.clear();
          } else {
            dynIsland.push_back(contactedBody);
            contactedBody->setDynamicsFlag();
            numDynBodies++;
          }
        }
      }
    }



#ifdef GRASPITDBG
    int numIslandRobots = islandRobots.size();
    std::cout << "Island " << ++islandCount << " Bodies: ";
    for (i = 0; i < numDynBodies; i++) {
      std::cout << dynIsland[i]->getName().toStdString().c_str() << " ";
    }
    std::cout << std::endl;
    std::cout << "Island Robots" << islandCount << " Robots: ";
    for (i = 0; i < numIslandRobots; i++) {
      std::cout << islandRobots[i]->getName().toStdString().c_str() << " ";
    }
    std::cout << std::endl << std::endl;
#endif

    for (i = 0; i < numDynBodies; i++) {
      dynIsland[i]->markState();
    }

    DynamicParameters dp;
    if (numDynBodies > 0) {
      dp.timeStep = timeStep;
      dp.useContactEps = true;
      dp.gravityMultiplier = 1.0;
      lemkeErrCode = iterateDynamics(islandRobots, dynIsland, &dp);

      if (lemkeErrCode == 1) { // dynamics could not be solved
        std::cerr << "LCP COULD NOT BE SOLVED!" << std::endl << std::endl;
        turnOffDynamics();
        return -1;
      }
    }

    dynIsland.clear();
    islandRobots.clear();
    allDynamicsComputed = true;
    for (i = 0; i < numBodies; i++)
      if (mWorld->getBody(i)->isDynamic() &&
          !((DynamicBody *)mWorld->getBody(i))->dynamicsComputed()) {
        allDynamicsComputed = false;
        break;
      }
  }  while (!allDynamicsComputed);

  // clear all the dynamicsComputed flags
  for (i = 0; i < numBodies; i++)
    if (mWorld->getBody(i)->isDynamic()) {
      ((DynamicBody *)mWorld->getBody(i))->resetDynamicsFlag();
    }

  mWorld->emitDynamicStepTaken();
  return 0;
}


int GraspitDynamics::stepDynamics() {

  double actualTimeStep = moveDynamicBodies(mWorld->getTimeStep());
  if (actualTimeStep < 0) {
    GraspitDynamics::turnOffDynamics();
    mWorld->emitdynamicsError("Timestep failsafe reached.");
    return -1;
  }

  for (int i = 0; i < mWorld->getNumRobots(); i++) {
    if (runController)
    {
      mWorld->getRobot(i)->DOFController(actualTimeStep);
    }
    mWorld->getRobot(i)->applyJointPassiveInternalWrenches();
  }

  if (computeNewVelocities(actualTimeStep)) {
    mWorld->emitdynamicsError("LCP could not be solved.");
    return -1;
  }

  mWorld->resetDynamicWrenches();

  return 0;
}
