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
// $Id: contactSetting.cpp,v 1.3 2009/06/16 19:28:28 cmatei Exp $
//
//######################################################################

#include "contactSetting.h"
#include "contact.h"
#include "matvec3D.h"
#include "world.h"
#include "body.h"
#include "mytools.h"
#include "FitParabola.h"

//#define GRASPITDBG
#include "debug.h"

double 
contactDistance(Body* body1, Body* body2, ContactData &cp)
{
	position b1_pos(cp.b1_pos), b2_pos(cp.b2_pos);
	b1_pos = b1_pos * body1->getTran();
	b2_pos = b2_pos * body2->getTran();
	vec3 dist_vec = (b1_pos-b2_pos);
	return dist_vec.len();
}

/*	
	Attempt to check if the normals of the contact are well defined by looking at the distance between the bodies.
*/	
bool 
checkContactNormals(Body *b1, Body *b2, ContactData *c)
{
	bool r = true;
	vec3 n1(c->b1_normal);
	vec3 n2(c->b2_normal);
	n1 = n1 * b1->getTran();
	n2 = n2 * b2->getTran();

	position p1(c->b1_pos);
	position p2(c->b2_pos);
	p1 = p1 * b1->getTran();
	p2 = p2 * b2->getTran();

	vec3 d = p2 - p1;
	if ( d % n1 > 0) {
		r = false;
		for (int i=0; i<3; i++) {
			c->b1_normal[i] = -c->b1_normal[i];
		}
	}
	if ( d % n2 < 0) {
		r = false;
		for (int i=0; i<3; i++) {
			c->b2_normal[i] = - c->b2_normal[i];
		}
	}
	return r;
}

bool neighborhoodsOverlap(const Neighborhood &n1, const Neighborhood &n2) {
	// both neighborhoods are assumed to be in the same frame 
	Neighborhood::const_iterator it1, it2;
	for (it1 = n1.begin(); it1!=n1.end(); it1++)
		for (it2 = n2.begin(); it2 != n2.end(); it2++) {
			if ( *it1 == *it2 )
				return true;
		}
	return false;
}

void mergeNeighborhoods(Neighborhood &n1, Neighborhood &n2)
{
	Neighborhood::iterator it1, it2;
	bool present;
	for (it2 = n2.begin(); it2 != n2.end(); it2++) {
		present = false;
		for (it1 = n1.begin(); it1 != n1.end(); it1++) {
			if ( *it1 == *it2 ) {
				present = true;
				break;
			}
		}
		if (!present) {
			n1.push_back( *it2 );
		}
	}
}

/*! For elastic bodies, calculates the contact neighborhoods and puts 
	them in the contact set. Not done for rigid contacts.
*/
void findSoftNeighborhoods( Body *body1, Body *body2, ContactReport &contactSet )
{
	ContactReport::iterator itr;

	for( itr = contactSet.begin(); itr != contactSet.end(); itr++ )	{
		DBGP("Contact finding regions:");
		//right now, findregion assumes point is in body frame
		//the units for the threshold and the radius should be in mm, NOT cm

		//The input radius is proportional to the fourth root of the youngs mod/depth of mattress
		//(units for Youngs Mod and mattress depth are in Pa and meters)
		//This gives a radius around 6 mm, for rubber with youngs = 1.5E6 and h = 3E-3which is reasonable

		double rad = pow( 1/(MAX( body1->getYoungs(), body2->getYoungs() )), 0.333 ) * 1000.0 * 0.4;
		//the 0.4 is a fudge factor for the time being

		//hack to ensure that the fit is at least resonable
		if( rad <= 3.0 && rad >= 10.0 )	rad = 5.0;

		body1->getWorld()->FindRegion( body1, itr->b1_pos, itr->b1_normal, rad, &(itr->nghbd1) );
		DBGP("Neighborhood on body1 has " << itr->nghbd1.size() << " points");
		body2->getWorld()->FindRegion( body2, itr->b2_pos, itr->b2_normal, rad, &(itr->nghbd2) );
		DBGP("Neighborhood on body2 has " << itr->nghbd2.size() << " points");
	}
}

/*! For elastic objects, if we have two contacts close to each other 
	we assume they are actually part of the same contact. This function 
	checks for overlapping neighborhoods and merges them.
*/
void
mergeSoftNeighborhoods(Body *body1, Body *body2, ContactReport &contactSet)
{
	bool mergePerformed = true;

	ContactReport::iterator refContact, otherContact;

	while (mergePerformed) {
		mergePerformed = false;
		//check each contact agains each other contact
		for (refContact=contactSet.begin(); refContact!=contactSet.end(); refContact++) {
			for (otherContact = contactSet.begin(); otherContact != contactSet.end(); otherContact++) {
				if (otherContact == refContact)	continue;

				//this arbitrarily checks only the neighborhoods on body1
				if ( neighborhoodsOverlap(refContact->nghbd1, otherContact->nghbd1) ) {
					DBGP("Overlap found");
					// this should be improved; it keeps the closest contact but merges neighborhoods
					// ideally it should keep the closest contact and RECALCULATE the neighborhoods
					if ( contactDistance(body1, body2, *refContact) < contactDistance(body1, body2, *otherContact) ) {
						mergeNeighborhoods( refContact->nghbd1, otherContact->nghbd1 );
						mergeNeighborhoods( refContact->nghbd2, otherContact->nghbd2 );
						contactSet.erase( otherContact );
					} else {
						mergeNeighborhoods( otherContact->nghbd1, refContact->nghbd1 );
						mergeNeighborhoods( otherContact->nghbd2, refContact->nghbd2 );
						contactSet.erase( refContact );
					}
					mergePerformed = true;
					break;
				} else {
					DBGP("Overlap not found");
				}
			}
			//if we have a merge, restart operation
			if (mergePerformed)
				break;
		}
	}
}

/*!
  Takes pointers to the two bodies in contact, and the set of contacts returned
  from the collision detection system, and adds a contact to each body for each
  contact in the set.
 */
void
addContacts(Body *body1, Body *body2, ContactReport &contactSet, bool softContactsOn )
{
	ContactReport::iterator cp;
	Contact *c1,*c2;
	int i;

	if ( softContactsOn && ( body1->isElastic() || body2->isElastic() ) ) {
		findSoftNeighborhoods( body1, body2, contactSet);
		DBGP("Before merge: " << contactSet.size());
		mergeSoftNeighborhoods( body1, body2, contactSet);
		DBGP("After merge: " << contactSet.size());
	}

	for (i=0,cp=contactSet.begin();cp!=contactSet.end();cp++,i++) {

		DBGP( body1->getName().latin1() << " - " << body2->getName().latin1() << " contact: " <<
		cp->b1_pos << " " <<  cp->b1_normal );

		//this is an attempt to check if the contact normals point in the right direction
		//based on the distance between the bodies. It is meant to help with bad geometry with ill-defined
		//normals. Can be removed completely - should never come up for good geometry
		if (! checkContactNormals(body1, body2, &(*cp)) ) {
			DBGP("Wrong normals detected!");
		}
		if ( softContactsOn && ( body1->isElastic() || body2->isElastic() ) ) {
			c1 = new SoftContact( body1, body2, cp->b1_pos, cp->b1_normal, &(cp->nghbd1) );
			c2 = new SoftContact( body2, body1, cp->b2_pos, cp->b2_normal, &(cp->nghbd2) );
			c1->setMate(c2);
			c2->setMate(c1);

			((SoftContact *)c1)->setUpFrictionEdges();
			((SoftContact *)c2)->setUpFrictionEdges();
		} else {
			c1 = new PointContact(body1,body2,cp->b1_pos,cp->b1_normal);
			c2 = new PointContact(body2,body1,cp->b2_pos,cp->b2_normal);
			c1->setMate(c2);
			c2->setMate(c1);
		}

		body1->addContact(c1);
		body2->addContact(c2);

		//check if the new contacts inherit two contacts from previous time step
		//if so, remove ancestors so nobody else inherits them
		Contact *ancestor = body1->checkContactInheritance(c1);
		if (ancestor) {
			c1->inherit(ancestor);
			if (!ancestor->getMate()) 
				fprintf(stderr,"No mate for inherited contact!!\n");
			else
				c2->inherit(ancestor->getMate());
			//careful: this also deletes the removed contact so remove the mate first
			if (ancestor->getMate()) body2->removePrevContact( ancestor->getMate() );
			body1->removePrevContact( ancestor );
		} else {
			ancestor = body2->checkContactInheritance(c2);
			if (ancestor){
				if (!ancestor->getMate())
					fprintf(stderr,"No mate for inherited contact!!\n");
				else		
					c1->inherit(ancestor->getMate());
				c2->inherit(ancestor);
				if (ancestor->getMate()) body1->removePrevContact( ancestor->getMate() );
				body2->removePrevContact( ancestor );
			} else {
//				fprintf(stderr,"New contact between %s and %s\n",body1->getName().latin1(), body2->getName().latin1() );
			}
		}
	}
}

/*! Adds a virtual contact on the body \a body1, which is assumed to
	be the \a l-th link on the \a f-th finger of a robot. Works by
	first creating a traditional contact, then converting it to a 
	virtual contact.
	
	The whole VirtualContact scheme is in bad need of an overhaul.
*/
void
addVirtualContacts(Body *body1, int f, int l, Body *body2, ContactReport &contactSet, 
				   bool softContactsOn )
{
	if ( softContactsOn && body1->isElastic() )	{
		findSoftNeighborhoods( body1, body2, contactSet );
		mergeSoftNeighborhoods(body1, body2, contactSet);
	}

	ContactReport::iterator cp;
	for (cp=contactSet.begin(); cp!=contactSet.end(); cp++) {
		Contact *c1;
		if ( softContactsOn && body1->isElastic() )	{
			c1 = new SoftContact( body1, body2, cp->b1_pos, cp->b1_normal, &(cp->nghbd1) );
			c1->setMate(NULL);
			((SoftContact *)c1)->setUpFrictionEdges();
		} else {
			c1 = new PointContact(body1, body2, cp->b1_pos, cp->b1_normal);
			c1->setMate(NULL);
		}
		
		VirtualContact *vc = new VirtualContact(f, l, c1);
		vc->setBody(body1);
		body1->addVirtualContact(vc);
		delete c1;
	}
}
