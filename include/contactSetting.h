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
// Author(s):  Andrew T. Miller, Claire Lackner and Matei T. Ciocarlie
//
// $Id: contactSetting.h,v 1.1 2009/03/26 22:35:10 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief A number of global functions that are used by the GraspIt world 
  to set up new contacts between bodies. We should find a home for them
  inside a class at some point.
 */

#include "collisionStructures.h"

class Body;

//! Adds a new pair of contacts between two bodies
void addContacts(Body *body1, Body *body2, ContactReport &contactSet, 
				 bool softContactsOn = false );

//! Adds a virtual contact on a body pointing at another body
void addVirtualContacts(Body *body1, int f, int l, Body *body2, 
						ContactReport &contactSet, bool softContactsOn );
