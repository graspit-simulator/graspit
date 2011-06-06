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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: worldElement.cpp,v 1.32 2010/01/14 01:08:45 cmatei Exp $
//
//######################################################################


/*! \file 
  \brief Implements the world element base class
 */
#include "worldElement.h"
#include "matvec3D.h"
#include "world.h"
#include "mytools.h"
#include "contact.h"
#include "body.h"
#include "collisionInterface.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"
//Added by qt3to4:
#include <QTextStream>

const double WorldElement::ONE_STEP = 1.0e6;

/*!
  Protected constructor should only be called by subclasses.
  It initializes an empty worldElement.
*/
WorldElement::WorldElement(World *w,const char *name) : QObject((QObject *)w,name)
{
  myWorld=w; IVRoot=NULL;
  if (!name) myName = "unnamed";
  else myName = name;
  myFilename = "unspecified";
  contactsChangedFlag=false;
}


/*!
  Protected copy constructor (should not be called by user)
*/
WorldElement::WorldElement(const WorldElement &e) : QObject((QObject *)e.myWorld,e.name())
{
  myWorld = e.myWorld;
  myName = e.myName;
  IVRoot = e.IVRoot;
  myFilename = e.myFilename;
  contactsChangedFlag = e.contactsChangedFlag;
}


/*!
  Protected destructor should only be called by subclasses
  Currently this is simply a stub.
*/
WorldElement::~WorldElement()
{

}

/*! Given a start position which is expected to be collision-free, and a
  new position which which causes inter-penetration, it interpolates
  between the two to find the exact moment of contact. Returns false if
  the interpolation fails (usually because the starting point is also in
  collision).
  
  Only looks at possible collisions in \a colReport, which the caller must
  determine before calling this.
*/
bool 
WorldElement::interpolateTo(transf lastTran, transf newTran, const CollisionReport &colReport)
{
  vec3 nextTranslation;
  Quaternion nextRotation;
  transf nextTran;
  int numCols = colReport.size();
  
  //this causes the interpolation to first check the original transform
  //since in many cases the original position is actually the one in contact, this can save a lot of computation
  //as well as machine precision problems
  //technically, it allows t to become negative, but in practice this should never happen as long as the initial position
  //is collision free
  double t = 0.0, deltat = 1.0, minDist;
  bool done = false;
  
  while (!done && deltat > 1.0e-20 && t >= 0) {
    DBGP("move interpolation cycle")
      deltat /= 2.0;
    
    nextTranslation = (1.0-t)*lastTran.translation() + t*newTran.translation();
    nextRotation = Quaternion::Slerp(t,lastTran.rotation(),newTran.rotation());
    nextTran = transf(nextRotation,nextTranslation);
    DBGP("moving to time : " << t);
    if (setTran(nextTran) == FAILURE) {
      deltat = 0;
      break;
    }
    
    minDist = myWorld->getDist(colReport[0].first,colReport[0].second);
    for (int i=1; i<numCols; i++) {
      double dist = myWorld->getDist(colReport[i].first,colReport[i].second);
			minDist = MIN(minDist,dist);
    }
    DBGP("minDist: " << minDist);
    if (minDist > 0) {
      if (minDist < Contact::THRESHOLD * 0.5)
        break;
      t += deltat;
    }
    else
      t -= deltat;
    
    //debug code
    if ( deltat <= 1.0e-20 || t < 0) {
      for (int i=0; i<numCols; i++) {
        double dist = myWorld->getDist(colReport[i].first,colReport[i].second);
        DBGA(colReport[i].first->getName().latin1() << " -- " <<
             colReport[i].second->getName().latin1() << " is " << dist);			
      }
    }
    
  }
  if (deltat < 1.0e-20 || t < 0) {
    DBGP("deltat failsafe or negative t hit; interpolate failure");
    fprintf(stdout,"WorldElement interpolation error!\n");
    return false;
  } else {
    DBGP("deltat: " << deltat << "; minDist: " << minDist <<"; interpolate success.");
    return true;
  }  
}

/*! Attempts to move the element from its current pose to the new pose
  in \a newTran in a single step. If the final pose if collision-free
  it is done. If not, it interpolates to find the exact moment of contact.
  Returns \a false if the interpolation fails.
*/
bool WorldElement::jumpTo(transf newTran, CollisionReport *contactReport)
{
  int i, numCols;
  bool success;
  CollisionReport colReport;
  transf lastTran = getTran();
  if ( setTran(newTran) == FAILURE) return false;
  //we are only interested in collisions involving this body
  std::vector<Body*> interestList;
  //a robot will place all of its links in here
  getBodyList(&interestList);
  contactReport->clear();
  while (1) {
    numCols = myWorld->getCollisionReport(&colReport, &interestList);
    if (!numCols) {
      return true;
    }
    
#ifdef GRASPITDBG
    for (i=0; i<numCols; i++) {
      std::cerr << colReport[i].first->getName().latin1()<<" -- " 
                << colReport[i].second->getName().latin1() << std::endl;
    }
    DBGP("I am " << myName.latin1() );
#endif
    
    success = interpolateTo(lastTran, getTran(), colReport );
    if (!success) {
      return false;
    }
    contactReport->clear();
    for (i=0;i<numCols;i++) {
      if (myWorld->getDist(colReport[i].first,colReport[i].second) < Contact::THRESHOLD)
        contactReport->push_back(colReport[i]);
    }
  }
}

/*! 
  Moves the element from its current pose to the new pose specified by \a tr.
  This motion is performed in several steps such that the translation
  between each step does not exceed \a translStepSize and the angle of
  rotation does not exceed \a rotStepSize (expressed in radians).  The
  intermediate poses are determined using linear interpolation for the
  translation and spherical linear interpolation for the rotation.  If
  a collision is encountered during the motion, the point of first contact
  is determined and the element is left in that position.  This function
  returns false if a collision was encountered (or contacts prevented the motion)
  or true if no collisions were encountered and the move was completed.
*/
bool
WorldElement::moveTo(transf &newTran,double translStepSize, double rotStepSize)
{
  bool moveFinished = false;
  transf origTran,nextTran,motion;
  Quaternion nextRotation;
  vec3 nextTranslation;
  double percentComplete,moveIncrement,translationLength;
  double angle;
  vec3 axis;
  bool success;
  
  CollisionReport contactReport;
  
  //DBGP("moveTo called");
  
  origTran = getTran();
  /*
    std::cout << "WorldElement origTran: " << origTran.translation().x() << " " <<
    origTran.translation().y() << " " <<
    origTran.translation().z() << " " <<
    origTran.rotation().w << " " <<
    origTran.rotation().x << " " <<
    origTran.rotation().y << " " <<
    origTran.rotation().z << " " << "\n";
  */
  //calculate the difference
  translationLength = (newTran.translation() - origTran.translation()).len();
  nextRotation = newTran.rotation() * origTran.rotation().inverse();
  nextRotation.ToAngleAxis(angle,axis);
  
  moveIncrement = 1.0;
  if (translationLength != 0.0) {
    if (translStepSize == ONE_STEP) 
      moveIncrement = 1.0;
    else
      moveIncrement = MIN(moveIncrement, translStepSize / translationLength);
  }
  if (angle != 0.0) {
    if (rotStepSize == ONE_STEP)
      moveIncrement = MIN(moveIncrement, 1.0);
    else
      moveIncrement = MIN(moveIncrement, rotStepSize / angle);
  }
  
  // check contacts
  nextTranslation = (1.0-moveIncrement)*origTran.translation() + moveIncrement*newTran.translation();
  nextRotation = Quaternion::Slerp(moveIncrement,origTran.rotation(), newTran.rotation());
  nextTran = transf(nextRotation,nextTranslation);
  motion = nextTran * getTran().inverse();
  
  if (contactsPreventMotion(motion)) {
    DBGP("contacts prevent motion")
      return false;
  }
  
  percentComplete = 0.0;
  while (!moveFinished) {
    percentComplete += moveIncrement;
    if (percentComplete >= 1.0) {
      percentComplete = 1.0;
      moveFinished = true;
    }
    
    nextTranslation = (1.0-percentComplete)*origTran.translation() + percentComplete*newTran.translation();
    nextRotation = Quaternion::Slerp(percentComplete,origTran.rotation(), newTran.rotation());
    nextTran = transf(nextRotation,nextTranslation);
    /*
      std::cout << "moveTo NextTran: " << nextTran.translation().x() << " " <<
      nextTran.translation().y() << " " <<
      nextTran.translation().z() << " " <<
      nextTran.rotation().w << " " <<
      nextTran.rotation().x << " " <<
      nextTran.rotation().y << " " <<
      nextTran.rotation().z << " " << "\n";
    */
    success = jumpTo(nextTran, &contactReport);
    if (!success || contactReport.size() != 0) {
      moveFinished = true;
    }
  }
  
  if (!success) {
    DBGA("JumpTo error, stopping execution. Object " << myName.latin1() << " in thread " 
         << getWorld()->getCollisionInterface()->getThreadId());
  } else {
    myWorld->findContacts(contactReport);
  }
  
  if (contactReport.size() != 0) return false;
  return true;
}

