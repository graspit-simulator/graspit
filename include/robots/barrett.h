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
