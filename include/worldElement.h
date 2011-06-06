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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: worldElement.h,v 1.17 2009/06/18 20:41:02 saint Exp $
//
//######################################################################

/*! \file 
  \brief Defines the world element base class that is used for all robots and bodies.
 */
#ifndef WORLD_ELEMENT_H

#include <vector>

#include <Inventor/SoType.h>
#include <QString>
#include <QObject>
#include <QTextStream>

#include "collisionStructures.h"

class transf;
class World;
class SoSeparator;
class Body;

//! The base class for all physical elements within the world.
/*!  A world element is either an individual body or a robot which is comprised
     of a collection of articulated bodies.  It serves as the common base class
     for these two types of elements.
	 
	 A WorldElement can not be initialized directly, but only by sub-classes.
*/
class WorldElement : public QObject{
  Q_OBJECT  ;
public:
  //! Used to indicate moves should be made on one step, not broken down in small steps.
  static const double ONE_STEP;

private:
 //! Indicates whether any contacts have formed or been broken since the last time this was reset.
 bool contactsChangedFlag;

protected:
  //! A pointer to the world that this element is a part of
  World *myWorld;

  //! A pointer to the root of this element's Inventor scene graph
  SoSeparator *IVRoot;
 
  //! Holds the file name associated with this element.  
  /*! For bodies this is the geometry file, for robots its the configuration file.
	  It is always stored relative to the GraspIt root, which is obtained from the
	  GRASPIT environment variable.
  */
  QString myFilename;

  //! Holds the name of this element
  QString myName;

  //! Initializes and empty element belonging to a world
  WorldElement(World *w,const char *name);

  //! Copy constructor
  WorldElement(const WorldElement &e);

  //! Finds the moment of contacts between a collision and a collision-free position
  virtual bool interpolateTo(transf lastTran, transf newTran, const CollisionReport &colReport);

  //! Moves element to a new location in one step; resolves collisions along the way
  virtual bool jumpTo(transf newTran, CollisionReport *contactReport);

  //! Adds all the bodies that make up this WorldElement to the given list of bodies
  /*! This is implemented in both Body (which just adds itself) and Robot (which adds
	  all of its links plus the base).
  
	  This is not ideal, as it automatically implies that any WorldElement is made 
	  up of instances of the Body class, which actually inherits from WorldElement.
	  However, this is true for both classes that, at this time, inherit from 
	  WorldElement: the Body is obviously a Body itself, while the Robot is made up
	  from a collection of Body instances (the links and the base).
  */
  virtual void getBodyList(std::vector<Body*> *bodies) = 0;

public:
  //! Currently, just a stub
  virtual ~WorldElement();

  /*! Returns the world the element is a part of */
  World *getWorld() const {return myWorld;}

  /*! Returns a pointer to the root of this element's Inventor scene graph */
  SoSeparator *getIVRoot() const {return IVRoot;}

  /*! Returns the filename associated with this element */
  QString getFilename() const {return myFilename;}

  /*! Returns the name of this element */
  QString getName() const {return myName;}

  //! Sets the name of this element
  virtual void setName(QString newName){myName = newName;}

  //! Sets the filename of this element
  virtual void setFilename(QString newName){myFilename = newName;}

  /*! Returns the current pose of this element relative to the world frame.
      This is a purely abstract function that is reimplemented in the robot
      and body classes */
  virtual const transf& getTran() const =0;

  /*! Checks whether contacts on the element will prevent the element from 
      making the motion described by the transform motion.  This is a purely
      abstract function that is reimplemented in the robot and body classes */
  virtual bool contactsPreventMotion(const transf& motion) const =0;

  /*! Returns the state of the contactsChanged flag. */
  bool contactsChanged() const {return contactsChangedFlag;}

  /*! This function is called whenever a contact is formed or broken on a body
      or any body that is part of a robot.
  */
  virtual void setContactsChanged() {contactsChangedFlag=true;}

  /*! This function is called after a grasp has been analyzed. */
  virtual void resetContactsChanged() {contactsChangedFlag=false;}

  /*! Sets the current pose of the element with respect to the world frame.
      This is a purely abstract function that is reimplemented in the robot and
      body classes.
  */
  virtual int setTran(transf const& newTr)=0;
  
  //! Moves this element to a new location in small steps and resolves collisions along the way
  virtual bool moveTo(transf &tr,double translStepSize,double rotStepSize);

  //! Needed so that when processing the glove and flock we can access interpolateTo
  friend class SensorInputDlg;
};


#define WORLD_ELEMENT_H
#endif
