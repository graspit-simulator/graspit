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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: barrett.h,v 1.12 2009/06/17 21:10:38 saint Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %Barrett hand class, a specialized hand subclass
 */
#ifndef BARRETT_H

#include "robot.h"

class BarrettHand;

//! The Barrett hand has its own dedicated class so it can speak to a real Barrett hand
/*! The Barrett hand class can also hold an instance of the interface to
	a real Barrett hand. Using this interface, the real Barrett can be made
	to mimic the behavior of the simulated Barrett, or vice versa.
*/
class Barrett : public Hand {
	Q_OBJECT

protected:
	//! The interface to the real Barrett hand (or NULL if no real hand is used)
  BarrettHand *mRealHand;

 public:
  //! Just sets the interface to NULL, no real hand used by default
  Barrett(World *w,const char *name) : Hand(w,name) {mRealHand = NULL;}
  //! Also deletes the interface, if any
  ~Barrett();

  //! Also sets the real hand interface to NULL
  int loadFromXml(const TiXmlElement* root,QString rootPath);

  //! Returns the interface to the real barrett hand
  BarrettHand *getRealHand();

  //! Returns true if the real hand is currently executing a motor command
  bool isBusy();
};

#define BARRETT__H
#endif
