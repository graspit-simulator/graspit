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
// Author(s): Matei T. Ciocarlie
//
// $Id: collisionStructures.h,v 1.3 2009/03/30 14:59:47 cmatei Exp $
//
//######################################################################
/*! \file 
	Defines the data structures that are used for collision
	and contact detection in GraspIt.
*/


#ifndef _collisionstructures_h_
#define _collisionstructures_h_

#include <vector>
#include "matvec3D.h"

class Body;
class transf;
class position;
class vec3;

/*! A neighborhood is a list of vertices on the surface of a body.
	Usually they are all within some threshold distance of a reference
	point on the body (usually a contact).
*/
typedef std::vector<position> Neighborhood;

/*! A contact data structure will hold:
	- the position on each body (in local coordinate system) where the
	  contact is occuring.
	- the local surface normal at the contact on each body
	- the squared distance between the bodies
	- optionally, a neighborhood of vertices on each body around the 
	  contact. This is used for computing analytical patches on each
	  body around the contact.
 */
typedef struct ContactDataS
{
  position b1_pos,b2_pos;
  vec3 b1_normal, b2_normal;
  Neighborhood nghbd1, nghbd2;
  double distSq;
  ContactDataS(position b1p, position b2p, vec3 b1n, vec3 b2n, double dsq=-1.0) : b1_pos(b1p), b2_pos(b2p),
	  b1_normal(b1n), b2_normal(b2n), distSq(dsq) {}
  ContactDataS() : b1_pos(0,0,0), b2_pos(0,0,0),
	  b1_normal(0,0,0), b2_normal(0,0,0), distSq(0) {}
} ContactData;

/*! A contact report is a list of multiple contacts, usually all 
	occuring between the same two bodies.
*/
typedef std::vector<ContactData> ContactReport;

/*! A collision data structure contains pointers to the two bodies involved.
	When used for a collision, this is all the information available. If used
	for a contact, it also hold a list of all the contacts between the two
	bodies. Recall that a contact is defined as a location where the two bodies
	are separated by less than the contact threshold (but never interpenetrating).
*/
typedef struct CollisionDataS
{
	Body *first;
	Body *second;
	ContactReport contacts;
	CollisionDataS(Body *b1, Body *b2) : first(b1), second(b2) {}
} CollisionData;

/*! A collision report is a list of multiple collision data structures, 
	usually one for each pair of colliding or contacting bodies.
*/
typedef std::vector<CollisionData> CollisionReport;

#endif
