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
// Author(s):  Matei T. Ciocarlie
//
// $Id: collisionInterface.h,v 1.9 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#ifndef _collisioninterface_h_
#define _collisioninterface_h_

#include "collisionStructures.h"

#include <QMutex>
#include <QThreadStorage>

class BoundingBox;

/*! This class provides an interface between the GraspIt core 
	and the collision detection libraries. 
	
	Core code should never call collision detection libraries 
	directly, but use this abstract class instead. For any collision 
	detection library you want to use, implement this interface class 
	using that library.
*/
class CollisionInterface
{
public:
	//! Two contacts that are separated by less than this are considered the same
	/*! Do not confuse this with the contact threshold itself, which is usually
		on the order of 0.1mm. We generally use this one on the order of 3mm.
		Arguably, there should be a better spot to define this, maybe where
		the contact threshold itself is defined as well.

		Not all implementations of the collision interface use this. Some do a 
		clever removal of contact duplicates themselves and don't need it.
	*/
	static const int CONTACT_DUPLICATE_THRESHOLD;
protected:
	// The following two functions are needed for all collision detectors, so they have
	// been implemented at this level.
	/*! Finds all the groups of contacts that have the same normal and replaces them 
	with the contacts on the convex hull of the perimeter that they define. According to 
	theory on contact areas, this has no effect	on quality metric computations.*/
	void compactContactSet(ContactReport *contacts);

	//! Removes all the duplicate contacts from a contact report
	/*! Two contacts where both contact points are separated by less than the \a 
		duplicateThreshold are considered to be the same contact; only on of them
		is kept. We currently keep the one where the two bodies are closer 
		together. 

		Do not confuse this duplicateThreshold (usually on the order of millimeters)
		with the contact threshold (usually 0.1mm).

		This is not done when using VCollide, which has a more intelligent method of
		pruning contacts itself.
	*/
	void removeContactDuplicates(ContactReport *contacts, double duplicateThreshold);
	//! Mutex for synchronizing access to the threading mechanism
	static QMutex mMutex;
private:
	//! Here we keep the information on what thread we are currently running
	/*! This is used so that bodies in a given thread only collide with bodies
		from the same thread, or from the master thread (with id=0).
	*/
	QThreadStorage<int*> mThreadIdStorage;
	//! The id to be assigned to the next thread who requests one
	static int mNextThreadId;
	/*! Helper function for compactContactSet(). Given a single group of contacts that have
	the same normal, computed the perimeter that they define and keeps only those contacts
	that are on the perimeter. */
	void replaceContactSetWithPerimeter(ContactReport &contactSet);
public:
	//! Initializes the threading mechanism
	CollisionInterface();
	//! Stub, does nothing
	virtual ~CollisionInterface(){}
	/*! FAST_COLLISION is used to ask if any collision exists in the world. 
		ALL_COLLISIONS is used to ask for all the collisions in the world */
	enum DetectionType{FAST_COLLISION, ALL_COLLISIONS};

	//! Add a new body to the collision detection system
	virtual bool addBody(Body *body, bool ExpectEmpty = false) = 0;
        //! Updates just the geometry of a body that is already in the system
        /*! Things like disabled pairs for that body should stay intact */
        virtual bool updateBodyGeometry(Body* body, bool ExpectEmpty = false) = 0;
	//! Remove a body from the collision detection system.
	virtual void removeBody(Body *body) = 0;
	/*! Creates a clone of a body that shares the collision geometry hierarchy
		but has its own transform and can be moved and queried for collision 
		independently. WARNING: there are still problems with the cloning
		mechanism. If the original is deleted, ot if its collision structures
		are changed, the lingering clone is almost certain to cause a crash.
	*/
	virtual void cloneBody(Body *clone, const Body *original) = 0;
	//! Set the position of a body in the collision detection system.
	virtual void setBodyTransform(Body *body, const transf &t) = 0;

	//! Activates / deactivates a body inside the collision detection system.
	/*! If a body is deactivated, it is not removed from the system,
		but it stops producing either collisions or contacts. Can be
		re-activated any time desired, it is very cheap.
	*/
	virtual void activateBody(const Body *body, bool active) = 0;
	//! Toggles collision between a particular pair of bodies
	virtual void activatePair(const Body *body1, const Body *body2, bool active) = 0;
	//! Tells us if collisions for a body in general, or a particular pair of bodies, are active
	/*! If \a body2 is NULL, it returns whether \a body1 is active in the
		collision detection system. Otherwise, it returns whether collisions
		between this particular pair of bodies are active.
	*/
	virtual bool isActive(const Body *body1, const Body *body2 = NULL) = 0;

	/*! Goes through all the bodies in the world and returns collisions. If \a type
		is FAST_COLLISION, returns 1 as soon as any collision is found. If \a type is
		ALL_COLLISIONS, returns the total number of collisions in the world. 
		
		The list of all colliding bodies is placed in the \a report. 
		
		If an \interestList is passed, then it only queries for collisions involving
		at least one body in the list. */
	virtual int allCollisions(DetectionType type, CollisionReport *report, 
							  const std::vector<Body*> *interestList) = 0;

	/*! Finds all the contacts in the world. \a interestList is used the same way as in
		\a allCollisions(). */
	virtual int allContacts(CollisionReport *report, double threshold, 
							const std::vector<Body*> *interestList) = 0;
	/*! Finds the contacts between two specified bodies */
	virtual int contact(ContactReport *report, double threshold, 
						const Body *body1, const Body *body2) = 0;

	/*! Distance from a point \a point specified in world coordinates to a body. 
		On exit, \a closestPoint is the point on the body that is closest to the given 
		point, and \a closestNormal is the body normal at that point. Both \a closestPoint
		and \a closestNormal are in world coordinates */
	virtual double pointToBodyDistance(const Body *body1, position point,
									   position &closestPoint, vec3 &closestNormal) = 0;
	/*! Distance between two bodies, or -1 if the bodies interpenetrate. On exit, \a p1 
		and \a p2 are the two points on the bodies that are closest to each other, each in
		its own body coordinate frame */
	virtual double bodyToBodyDistance(const Body *body1, const Body *body2,
									  position &p1, position &p2) = 0;

	/*! Finds the neighborhood of a given point on the surface of a body. Given the point
		\a point, it will find all the vertices of the body that are with \a radius of 
		this point, as long as they belong to triangles that do not have normals pointing 
		in the opposite direction. This is done in order not to return vertices that are
		on the other side of a thin body.

		Assumes both \a point and \a normal are in the coordinate frame of \a body */
	virtual void bodyRegion(const Body *body, position point, vec3 normal, 
							double radius, Neighborhood *neighborhood)=0;

	//! Returns the bounding box hierarchy at a certin depth for a body
	virtual void getBoundingVolumes(const Body*, int, std::vector<BoundingBox>*){}

	/*! Informs the collision detection system that this is now a new thread. All bodies 
		or clones added from now on are specific to this thread. Bodies from a given thread 
		can collide ONLY with bodies from the same thread, or with bodies from the master 
		thread, which has ID 0. */
	virtual void newThread();
	/*! Returns the id of this thread. The master (original) thread has ID 0. If threads 
		are not used, this will always return 0 */
	virtual int getThreadId();
};

#endif
