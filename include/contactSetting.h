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
