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
// $Id: grasp.cpp,v 1.51 2009/08/14 18:09:44 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the grasp class, which analyzes grasps
 */

/* standard C includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <exception>
#include <typeinfo>

#include <QString>
#include <q3listbox.h>

//#include "mainWindow.h"
#include "grasp.h"
#include "world.h"
#include "robot.h"
#include "joint.h"
#include "body.h"
#include "contact.h"
#include "gws.h"
#include "quality.h"
#include "gwsprojection.h"
#include "matrix.h"
#include "matvec3D.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include <Inventor/Qt/SoQtComponent.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define G_ 9.81

/*! This replicates GWS::ALL_DIMENSIONS, but it was needed here too
	so that we don't have to give any caller access to GWS.
*/
const std::vector<int> Grasp::ALL_DIMENSIONS = std::vector<int>(6, 1);

const int Grasp::CONTACT_FORCE_EXISTENCE = 0;
const int Grasp::CONTACT_FORCE_OPTIMIZATION = 1;
const int Grasp::GRASP_FORCE_EXISTENCE = 2;
const int Grasp::GRASP_FORCE_OPTIMIZATION = 3;

 
/*!
  Initialize grasp object variables.  A grasp should only be created by a
  hand constructor, which passes a pointer to itself.
*/
Grasp::Grasp(Hand *h)
{
  int i;

  hand = h;
  object = NULL;
  numContacts=0;
  valid=true;
  useGravity = false;

  for (i=0;i<6;i++) minWrench[i]=0.0;

  numQM = 0;
}

/*!
  Deletes all quality measures, grasps wrench spaces, and projections
  associated with this grasp.  Frees all of the dynamically allocated
  vectors and matrics used for GFO.
 */
Grasp::~Grasp()
{
  int i;
  std::list<GWSprojection *>::iterator gpp;
  std::list<GWS *>::iterator gp;

  std::cout << "Deleting grasp" <<std::endl;

  for (i=0;i<numQM;i++) removeQM(0);
  for (gpp=projectionList.begin();gpp!=projectionList.end();gpp++)
    delete *gpp;
  for (gp=gwsList.begin();gp!=gwsList.end();gp++)
    delete *gp;
}

/*!
	Returns a pointer to the GWS of type \a type if one exists. If one doesn't
	it returns NULL
*/
GWS *
Grasp::getGWS(const char *type)
{
	GWS *gws = NULL;
	std::list<GWS *>::iterator gp;
	for (gp=gwsList.begin();gp!=gwsList.end();gp++) {
		if (!strcmp((*gp)->getType(),type)) {
			gws = *gp;
		}
	}
	if (gws) gws->ref();  // add 1 to the reference count
	return gws;
}

/*!
  Creates a new GWS of type \a type and adds it to the list of GWS's associated
  with this grasp if it does not exist already.  Current possible types of
  GWS's are "L1 Norm", and "LInfinity Norm".  Returns a pointer to a gws
  of the requested type that was either created or found.
*/
GWS *
Grasp::addGWS(const char *type)
{
  GWS *gws = NULL;
  std::list<GWS *>::iterator gp;

  //first check if we have the needed GWS already
  for (gp=gwsList.begin();gp!=gwsList.end();gp++)
    if (!strcmp((*gp)->getType(),type))
      gws = *gp;
    
  if (!gws) {
    gws = GWS::createInstance(type,this);
    
    gwsList.push_back(gws);
    printf("created new %s GWS.\n",type);
  }

  gws->ref();  // add 1 to the reference count
    
  return gws;
}

/*!
  Given a pointer to the GWS to remove, this decrements the reference count
  for that GWS.  If the reference count becomes 0, the GWS is deleted.
*/
void
Grasp::removeGWS(GWS *gws)
{
  DBGP("removing gws");
  gws->unref();
  if (gws->getRefCount() == 0) {
    DBGP("deleting gws");
    gwsList.remove(gws);
    delete gws;
  } else {
	  DBGP("gws refcount: "<<gws->getRefCount());
  }
}

/*!
  Adds the quality measure to the grasp's list of quality measures.
*/
void
Grasp::addQM(QualityMeasure *qm)
{
   qmList.push_back(qm);
   numQM++;
}

/*!
  Replaces an existing quality measure with a new one and deletes the old one.
  \a which selects which qm to replace where the index of the first qm in the
  list is 0.
*/
void
Grasp::replaceQM(int which,QualityMeasure *qm)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  qmList.insert(qp,qm);

  if (qp!=qmList.end()) {
    delete *qp;
    qmList.erase(qp);
  }
}

/*!
  Returns a pointer to the requested quality measure.  \a which is the index
  of the qm to return, where 0 is the first qm in the list.
*/
QualityMeasure *
Grasp::getQM(int which)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  if (qp!=qmList.end()) return *qp;
  return NULL;
}

/*!
  Removes a quality measure from the list and deletes it.  \a which is the
  index of the qm to remove, where 0 is the first qm in the list.
*/
void
Grasp::removeQM(int which)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  if (qp!=qmList.end()) {
    printf("Removing QM\n");
    delete *qp;
    qmList.erase(qp);
    numQM--;
  }
}

/*!
  Add a grasp wrench space projection to the grasp's list of projections.
*/
void
Grasp::addProjection(GWSprojection *gp)
{
  projectionList.push_back(gp);
  update();
}

/*!
  Removes a grasp wrench space projection from the list and deletes it.
*/
void
Grasp::removeProjection(GWSprojection *gp)
{
    projectionList.remove(gp);
    delete gp;
}

/*!
  This is the projection window close callback function.  When the projection
  window is closed, this removes the associated GWSprojection.
*/
void
Grasp::destroyProjection(void * user, SoQtComponent *)
{
  GWSprojection *gp = (GWSprojection *)user;
  gp->getGWS()->getGrasp()->removeProjection(gp);
}

/*!
  Updates the grasp by collecting all the current contacts on the hand and
  rebuilding any grasp wrench spaces and their projections.  This is usually
  called after the contacts on the hand have changed.

  Also collects and uses any contacts on robots that are attached to this
  one. See \a collectContacts() for details.

  You can ask for the GWS to be built using only a subset of the 6 dimensions 
  of force and torque normally available. These are, in this order: 
  
  fx, fy, fz, tx, ty, tz

  If you want only a subset to be used, pass a vector of 6 ints with 1 for the
  dimensions you want or 0 for those that you do not want. For example, if you
  want a GWS that only uses fx, fy and tz, pass the following vector:

  1, 1, 0, 0, 0, 1

  If you want to use all 6 dimensions, pass the default value of 
  Grasp::ALL_DIMENSIONS.

  Note that the dimensions along which the GWS is built affects any subsequent
  projections that are created
*/
void
Grasp::update(std::vector<int> useDimensions)
{
	if (hand) {
		std::vector<Robot*> robotVec;
		hand->getAllAttachedRobots(robotVec);
		for (std::vector<Robot*>::iterator it=robotVec.begin(); it!=robotVec.end(); it++) {
			(*it)->resetContactsChanged();
		}
		if (object) {
			collectContacts();
		} else {
			collectVirtualContacts();
		}
	} else if (object) {
		// contact only on the object: a kind of virtual contact
		collectVirtualContactsOnObject();
	}

	DBGP("numContacts: " << numContacts);
	updateWrenchSpaces(useDimensions);
}

/*! See Grasp::update() for details about \a useDimensions which affect the 
	dimensions that are used to build the GWS.
*/
void
Grasp::updateWrenchSpaces(std::vector<int> useDimensions)
{
  std::list<GWS *>::iterator gp; 
  std::list<GWSprojection *>::iterator pp;  

  //for tests with the online planner
  vec3 gravDirection(0,0,1);
  
  //SCALE gravity to some arbitrary value; here to 0.5 of what one contact can apply
  gravDirection = 0.5 * gravDirection;

  //compute the direction of world gravity forces relative to the object
  if (useGravity && object) {
	  gravDirection = gravDirection * object->getTran().inverse();
  }

  // rebuild grasp wrench spaces
  for (gp=gwsList.begin();gp!=gwsList.end();gp++) {
	  if (useGravity && object) {
		  (*gp)->setGravity(true, gravDirection);
	  } else {
		  (*gp)->setGravity(false);
	  }
      (*gp)->build(useDimensions);
  }

  // update the GWS projections
  for (pp=projectionList.begin();pp!=projectionList.end();pp++) {
    (*pp)->update();
  }

}

/*!
  Gathers the contacts on all links of the hand that are mated with contacts
  on the grasped object and adds them to the contactVec.

  It also collects all contacts from hands attached to this one, and treats all
  of them as one big grasp. This allows us to do grasp analysis for complex
  robots, such as the M7, which has manipulators attached at the end of the arms.
  If you want to do grasp analysis just for the contacts on one of the manipulators,
  use the instance of Grasp of that particular hand.
*/
void
Grasp::collectContacts()
{
	contactVec.clear();
	//get the contacts from this as well as all attached robots
	std::vector<Robot*> robotVec;
	hand->getAllAttachedRobots(robotVec);
	for (std::vector<Robot*>::iterator it=robotVec.begin(); it!=robotVec.end(); it++) {
		std::list<Contact*> contacts = (*it)->getContacts(object);
		contactVec.insert(contactVec.end(), contacts.begin(), contacts.end());
	}
	//update wrenches for all contacts
	for (std::vector<Contact *>::iterator cp=contactVec.begin(); cp!=contactVec.end(); cp++) {
		(*cp)->getMate()->computeWrenches();
	}
	numContacts = (int)contactVec.size();
	DBGP("Contacts: " << numContacts);
}

vec3 Grasp::virtualCentroid()
{
	vec3 cog(0,0,0);
	position pos;

	/*
	//COG AS CENTROID
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		cog = cog + vec3( pos.toSbVec3f() );
	}
	cog = ( 1.0 / (int)contactVec.size() ) * cog;
	//fprintf(stderr,"CoG: %f %f %f\n",cog.x(), cog.y(), cog.z());
	*/

	//COG as center of bounding box
	position topCorner(-1.0e5, -1.0e5, -1.0e5), bottomCorner(1.0e5, 1.0e5, 1.0e5);
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		if ( pos.x() > topCorner.x() ) topCorner.x() = pos.x();
		if ( pos.y() > topCorner.y() ) topCorner.y() = pos.y();
		if ( pos.z() > topCorner.z() ) topCorner.z() = pos.z();
		if ( pos.x() < bottomCorner.x() ) bottomCorner.x() = pos.x();
		if ( pos.y() < bottomCorner.y() ) bottomCorner.y() = pos.y();
		if ( pos.z() < bottomCorner.z() ) bottomCorner.z() = pos.z();
	}
	cog = 0.5 * (topCorner - bottomCorner);
	cog = vec3(bottomCorner.toSbVec3f()) + cog;

	return cog;
}

/*! Assumes all the contacts in the list are virtual, and computes the centroid
	and max radius for the grasp based on these virtual contacts.
*/
void
Grasp::setVirtualCentroid()
{
	vec3 cog = virtualCentroid();
	position pos;

	//VARIABLE RADIUS relative to cog location
	vec3 radius;
	double maxRadius = 0;
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		radius =  vec3( pos.toSbVec3f() ) - cog;
		if ( radius.len() > maxRadius) maxRadius = radius.len();
	}

	//FIXED radius to allow better inter-grasp comparison (exact value pulled out of thin air)
	maxRadius = 150;

	//fprintf(stderr,"Max radius: %f\n",maxRadius);

	for (int i=0; i<(int)contactVec.size(); i++) {
		((VirtualContact*)contactVec[i])->setCenter( position(cog.toSbVec3f()) );
		((VirtualContact*)contactVec[i])->setRadius(maxRadius);
		//((VirtualContact*)contactVec[i])->getWorldIndicator();
	}
}

/*!	Assumes that all contacts are virtual, but we do have an object so we 
	set its centroid  (IN WORLD COORDINATES) and maxradius to all virtual 
	contacts.
*/
void
Grasp::setRealCentroid(GraspableBody *body)
{
	position cog = body->getCoG() * body->getTran();
	double maxRadius = body->getMaxRadius();
	for (int i=0; i<(int)contactVec.size(); i++) {
		((VirtualContact*)contactVec[i])->setCenter(cog);
		((VirtualContact*)contactVec[i])->setRadius(maxRadius);
	}
}

/*!  Gathers the virtual contacts on all links of the hand and adds them to 
	the internal list in contactVec. Any GWS computations should then be 
	able to proceed regardless of the fact that these are virtual contacts.
	However, since we might not have an object, information about the 
	centroid to be used as reference point, and the max radius used for
	converting torques, are also computed and stored in the virtual
	contacts themselves.
*/
void
Grasp::collectVirtualContacts()
{
	int f,l;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;

	contactVec.clear();
	numContacts = 0;

	contactList = hand->getPalm()->getVirtualContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		contactVec.push_back(*cp);
		numContacts++;
	}

	for(f=0;f<hand->getNumFingers();f++) {
		for (l=0;l<hand->getFinger(f)->getNumLinks();l++) {
			contactList = hand->getFinger(f)->getLink(l)->getVirtualContacts();
			for (cp=contactList.begin();cp!=contactList.end();cp++){
				contactVec.push_back(*cp);
				numContacts++;
			}
		}
	}

	if (object == NULL) {
		setVirtualCentroid();
		for (int i=0; i<(int)contactVec.size(); i++) {
			((VirtualContact*)contactVec[i])->computeWrenches(false);
		}
	} else {
		setRealCentroid(object);
		//for (int i=0; i<(int)contactVec.size(); i++) {
			//((VirtualContact*)contactVec[i])->setObject(object);
			//((VirtualContact*)contactVec[i])->computeWrenches(true);
			//((VirtualContact*)contactVec[i])->getWorldIndicator(true);
		//}
	}
 }

/*!
	When we are dealing with an object but no hand, gathers the virtual contacts from the
object and adds them to the contactVec, doing the same thing as collectContacts() and
collectVirtualContacts() do for the hand-and-object and the hand-only cases.
*/
void
Grasp::collectVirtualContactsOnObject()
{
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;
	
	contactVec.clear();
	numContacts = 0;
	contactList = object->getVirtualContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++){
		contactVec.push_back(*cp);
		numContacts++;
	}
	for (int i=0; i<(int)contactVec.size(); i++) {
		// use computeWrenches defined in Contact class
		((Contact*)contactVec[i])->computeWrenches();
	}
#ifdef GRASPITDBG
		fprintf(stderr,"ContactOnObject has been pushed, number is %d\n",numContacts);
#endif
	setRealCentroid(object);
}

double
Grasp::getMaxRadius()
{
	if (object) return object->getMaxRadius();
	if (numContacts == 0) return 0;
	return ((VirtualContact*)contactVec[0])->getMaxRadius();
}

position
Grasp::getCoG()
{
	if (object) return object->getCoG();
	if (numContacts == 0) return position(0,0,0);
	return ((VirtualContact*)contactVec[0])->getCenter();
}

/*!
  Computes the jacobian for the base frame of link \a l on finger \a f
  relative to the base frame of the finger. This is wrt THE FRAME OF LINK!!!
  
  Distances USED TO BE converted to meters (check why this is...). Now they are
  kept in MILLIMETERS!
*/
double *
Grasp::getLinkJacobian(int f, int l)
{
  int j,col;
  Joint *jointPtr;
  int numDOF = hand->getNumDOF();
  double *jac = new double[6*numDOF];
  double k;
  mat3 m;
  vec3 p;
  transf T;
  double db0 = 0.0;

  dcopy(6*numDOF,&db0,0,jac,1);
  
  //I use f=-1 on virtual contacts to show that a contact is on the palm
  if (f < 0) return jac;

  for (j=hand->getFinger(f)->getLastJoint(l);j>=0;j--) {
    jointPtr = hand->getFinger(f)->getJoint(j);
    col = jointPtr->getDOFNum();
    
	k = hand->getDOF(jointPtr->getDOFNum())->getStaticRatio(jointPtr);
	T = T * jointPtr->getDH()->getTran();
    m = T.affine();
    p = T.translation();
    
    if (jointPtr->getType() == REVOLUTE) {
      jac[col*6]   += k*(-m.element(0,0)*p.y() + m.element(0,1)*p.x());
      jac[col*6+1] += k*(-m.element(1,0)*p.y() + m.element(1,1)*p.x());
      jac[col*6+2] += k*(-m.element(2,0)*p.y() + m.element(2,1)*p.x());
      jac[col*6+3] += k*m.element(0,2);
      jac[col*6+4] += k*m.element(1,2);
      jac[col*6+5] += k*m.element(2,2);
    } else {
      jac[col*6]   += k*m.element(0,2);
      jac[col*6+1] += k*m.element(1,2);
      jac[col*6+2] += k*m.element(2,2);
      jac[col*6+3] += 0.0;
      jac[col*6+4] += 0.0;
      jac[col*6+5] += 0.0;
    }
  }
  return jac;
}

/*! Computes the contact jacobian J. J relates joint angle motion to 
  contact motion. Its transpose, JT, relates contact forces to joint
  forces.
  
  The joints and the contacts must both be passed in. Their order in 
  the incoming vectors will determine their indices in the Jacobian.
  This function will make sure that only the right joints affect each
  contact: joints that come before the link in contact, on the same
  chain.
  
  The Jacobian is ALWAYS computed in the local coordinate system of each
  contact. When multiplied by joint torques, it will thus yield contact
  forces and torques in the local contact coordinate system of each contact.
  It is easy to do computations in world coordinates too, but then it
  is impossible to discards rows that correspond to directions that the
  contact can not apply force or torque in.
*/
Matrix 
Grasp::contactJacobian(const std::list<Joint*> &joints, 
                       const std::list< std::pair<transf, Link*> > &contact_locations)
{
  //compute the world locations of all the joints of the robot
  //this is overkill, as we might not need all of them
  std::vector< std::vector<transf> > jointTransf(hand->getNumChains());
  for (int c=0; c<hand->getNumChains(); c++) {
    jointTransf[c].resize(hand->getChain(c)->getNumJoints(), transf::IDENTITY);
    hand->getChain(c)->getJointLocations(NULL, jointTransf[c]);
  }
  
  Matrix J( Matrix::ZEROES<Matrix>(int(contact_locations.size()) * 6, (int)joints.size()) );
  std::list<Joint*>::const_iterator joint_it;
  int joint_count = 0;
  int numRows = 0;
  for (joint_it=joints.begin(); joint_it!=joints.end(); joint_it++) {
    std::list< std::pair<transf, Link*> >::const_iterator contact_it;
    numRows = 0;
    for(contact_it=contact_locations.begin(); contact_it!=contact_locations.end(); contact_it++, numRows+=6) {
      Link *link = contact_it->second;
      //check if the contact is on the same chain that this joint belongs too
      if ( (*joint_it)->getChainNum() != link->getChainNum() ) {
        continue;
      }
      KinematicChain *chain = hand->getChain( link->getChainNum() );
      //check that the joint comes before the contact in the chain
      int last_joint = chain->getLastJoint( link->getLinkNum() );
      //remember that the index of a joint in a chain is different from
      //the index of a joint in a robot
      int jointNumInChain = (*joint_it)->getNum() - chain->getFirstJointNum();
      assert(jointNumInChain >= 0);
      if ( jointNumInChain > last_joint) continue;
      //compute the individual jacobian
      transf joint_tran = jointTransf.at(link->getChainNum()).at(jointNumInChain);
      transf contact_tran = contact_it->first * link->getTran();
      //always get the individual jacobian in local coordinates
      Matrix indJ(Joint::jacobian(*joint_it, joint_tran, contact_tran, false));
      //place it at the correct spot in the global jacobian
      J.copySubMatrix(numRows, joint_count, indJ);
    }
    joint_count++;
  }
  return J;
}

/*! Simply gets the locations of all the contacts in the list and calls the
  more general version that takes in std::list< std::pair<transf, Link*> > &contact_locations */
Matrix 
Grasp::contactJacobian(const std::list<Joint*> &joints, 
                       const std::list<Contact*> &contacts)
{
  std::list< std::pair<transf, Link*> > contact_locations;
  std::list<Contact*>::const_iterator contact_it;
  for(contact_it=contacts.begin(); contact_it!=contacts.end(); contact_it++) {
    if ((*contact_it)->getBody1()->getOwner() != hand) {
      DBGA("Grasp jacobian: contact not on hand");
      continue;
    }
    Link *link = static_cast<Link*>((*contact_it)->getBody1());
    contact_locations.push_back( std::pair<transf, Link*>((*contact_it)->getContactFrame(), link) );
  }
  return contactJacobian(joints, contact_locations);
}

/*! Simply gets the locations of all the contacts in the list and calls the
  more general version that takes in std::list< std::pair<transf, Link*> > &contact_locations */
Matrix 
Grasp::contactJacobian(const std::list<Joint*> &joints, 
                       const std::list<VirtualContact*> &contacts)
{
  std::list< std::pair<transf, Link*> > contact_locations;
  std::list<VirtualContact*>::const_iterator contact_it;
  for(contact_it=contacts.begin(); contact_it!=contacts.end(); contact_it++) {
    if ((*contact_it)->getBody1()->getOwner() != hand) {
      DBGA("Grasp jacobian: contact not on hand");
      continue;
    }
    Link *link = static_cast<Link*>((*contact_it)->getBody1());
    contact_locations.push_back( std::pair<transf, Link*>((*contact_it)->getContactFrame(), link) );
  }
  return contactJacobian(joints, contact_locations);
}

/*! Computes the grasp map G. D is the matrix that relates friction edge
  amplitudes to contact force and R is the matrix that transorms contact
  forces to world coordinate system. We then need to sum all of them up
  by multiplication with a summation matrix to get the grasp map
  
  G = S R D
  
  G relates friction edge amplitudes from all contacts to resultant 
  object wrench.
*/
Matrix 
Grasp::graspMapMatrix(const Matrix &R, const Matrix &D)
{
  int numContacts = R.rows() / 6;
  assert(6 * numContacts == R.rows());
  //summation matrix that adds full 6D object wrenches
  Matrix S(6, 6*numContacts);
  for(int i=0; i<numContacts; i++) {
    S.copySubMatrix(0, 6*i, Matrix::EYE(6,6));
  }
  
  Matrix SR(S.rows(), R.cols());
  matrixMultiply(S, R, SR);
  Matrix G(S.rows(), D.cols());
  matrixMultiply(SR, D, G);
  return G;
}


/*! One possible version of the GFO problem.

	Given a matrix of joint torques applied to the robot joints, this will check
	if there exists a set of legal contact forces that balance them. If so, it
	will compute the set of contact forces that adds up to the wrench of least
	magnitude on the object.

	For now, this output of this function is to set the computed contact forces
	on each contact as dynamic forces, and also to accumulate the resulting
	wrench on the object in its external wrench accumulator.

	Return codes: 0 is success, >0 means finger slip, no legal contact forces 
	exist; <0 means error in computation 
*/
int 
Grasp::computeQuasistaticForces(const Matrix &robotTau)
{
	//WARNING: for now, this ignores contacts on the palm. That might change in the future

	//for now, if the hand is touching anything other then the object, abort
	for (int c=0; c<hand->getNumChains(); c++) {
		if ( hand->getChain(c)->getNumContacts(NULL) !=
			hand->getChain(c)->getNumContacts(object) ) {
				DBGA("Hand contacts not on object");
				return 1;
			}
	}

	std::list<Contact*> contacts;
	std::list<Joint*> joints;

	bool freeChainForces = false;
	for(int c=0; c<hand->getNumChains(); c++) {
		//for now, we look at all contacts
		std::list<Contact*> chainContacts = hand->getChain(c)->getContacts(object);
		contacts.insert(contacts.end(), chainContacts.begin(), chainContacts.end());
		if (!chainContacts.empty()) {
			std::list<Joint*> chainJoints = hand->getChain(c)->getJoints();
			joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
		} else {
			//the chain has no contacts
			//check if any joint forces are not 0
			Matrix chainTau = hand->getChain(c)->jointTorquesVector(robotTau);
			//torque units should be N * 1.0e6 * mm
			if (chainTau.absMax() > 1.0e3) {
				DBGA("Joint torque " << chainTau.absMax() << " on chain " << c 
									 << " with no contacts");
				freeChainForces = true;
			}
		}
	}
	//if there are no contacts, nothing to compute!
	if (contacts.empty()) return 0;

	//assemble the joint forces matrix
	Matrix tau((int)joints.size(), 1);
	int jc; std::list<Joint*>::iterator jit;
	for (jc=0, jit = joints.begin(); jit!=joints.end(); jc++,jit++) {
		tau.elem(jc,0) = robotTau.elem( (*jit)->getNum(), 0 );
	}
	//if all the joint forces we care about are zero, do an early exit 
	//as zero contact forces are guaranteed to balance the chain
	//we should probably be able to use a much larger threshold here, if
	//units are what I think they are
	if (tau.absMax() < 1.0e-3) {
		return 0;
	}

	//if there are forces on chains with no contacts, we have no hope to balance them
	if (freeChainForces) {
		return 1;
	}

	Matrix J(contactJacobian(joints, contacts));
	Matrix D(Contact::frictionForceBlockMatrix(contacts));
	Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
	Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));

	//grasp map G = S*R*D
	Matrix G(graspMapMatrix(R, D));

	//left-hand equality matrix JTD = JTran * D
	Matrix JTran(J.transposed());
	Matrix JTD(JTran.rows(), D.cols());
	matrixMultiply(JTran, D, JTD);

	//matrix of zeroes for right-hand of friction inequality
	Matrix zeroes(Matrix::ZEROES<Matrix>(F.rows(), 1));

	//matrix of unknowns
	Matrix c_beta(D.cols(), 1);

	//bounds: all variables greater than 0
	Matrix lowerBounds(Matrix::ZEROES<Matrix>(D.cols(),1));
	Matrix upperBounds(Matrix::MAX_VECTOR(D.cols()));

	//solve QP
	double objVal;
	int result = factorizedQPSolver(G, JTD, tau, F, zeroes, lowerBounds, upperBounds, 
									c_beta, &objVal);
	if (result) {
		if( result > 0) {
			DBGP("Grasp: problem unfeasible");
		} else {
			DBGA("Grasp: QP solver error");
		}
		return result;
	}

	//retrieve contact wrenchs in local contact coordinate systems
	Matrix cWrenches(D.rows(), 1);
	matrixMultiply(D, c_beta, cWrenches);
	DBGP("Contact wrenches:\n" << cWrenches);

	//compute wrenches relative to object origin and expressed in world coordinates
	Matrix objectWrenches(R.rows(), cWrenches.cols());
	matrixMultiply(R, cWrenches, objectWrenches);
	DBGP("Object wrenches:\n" << objectWrenches);

	//display them on the contacts and accumulate them on the object
	displayContactWrenches(&contacts, cWrenches);
	accumulateAndDisplayObjectWrenches(&contacts, objectWrenches);

	//simple sanity check: JT * c = tau
	Matrix fCheck(tau.rows(), 1);
	matrixMultiply(JTran, cWrenches, fCheck);
	for (int j=0; j<tau.rows(); j++) {
		//I am not sure this works well for universal and ball joints
		double err = fabs(tau.elem(j, 0) - fCheck.elem(j,0));
		//take error as a percentage of desired force, if force is non-zero
		if ( fabs(tau.elem(j,0)) > 1.0e-2) {
			err = err / fabs(tau.elem(j, 0));
		} else {
			//for zero desired torque, out of thin air we pull an error threshold of 1.0e2
			//which is 0.1% of the normal range of torques at 1.0e6
			if (err < 1.0e2) err = 0;
		}
		// 0.1% error is considered too much
		if (  err > 1.0e-1) {
			DBGA("Desired torque not obtained on joint " << j << ", error " << err << 
				" out of " << fabs(tau.elem(j, 0)) );
			return -1;
		}
	}

	//complex sanity check: is object force same as QP optimization result?
	//this is only expected to work if all contacts are on the same object
	double* extWrench = object->getExtWrenchAcc();
	vec3 force(extWrench[0], extWrench[1], extWrench[2]);
	vec3 torque(extWrench[3], extWrench[4], extWrench[5]);
	//take into account the scaling that has taken place
	double wrenchError = objVal*1.0e-6 - (force.len_sq() + torque.len_sq()) * 1.0e6;
	//units here are N * 1.0e-6; errors should be in the range on miliN
	if (wrenchError > 1.0e3) {
		DBGA("Wrench sanity check error: " << wrenchError);
		return -1;
	}
	return 0;
}

/*! One possible formulation of the core GFO problem. Checks if some
	combination of joint forces exists so that the resultant object
	wrench is 0. See inner comments for exact mathematical formulation.
	Not for standalone use; is called by the GFO functions in the 
	Grasp class.
*/
int
graspForceExistence(Matrix &JTD, Matrix &D, Matrix &F, Matrix &G, 
					Matrix &beta, Matrix &tau, double *objVal)
{
	// exact problem formulation
	// unknowns: [beta tau]			   (contact forces and joint forces)
	// minimize [beta tau]T*[G 0]T*[G 0]*[beta tau] (magnitude of resultant object wrench)
	// subject to:
	// [JTD -I] * [beta tau] = 0       (contact forces balance joint forces)
	// [0 sum] * [beta tau] = 1        (we are applying some joint forces)
	// [F 0] [beta tau] <= 0		   (all forces inside friction cones)
	// [beta tau] >= 0	  		       (all forces must be positive)
	// overall equality constraint:
	// | JTD -I |  | beta |   |0|
	// | 0   sum|  |  tau | = |1|

	int numJoints = tau.rows();
	Matrix beta_tau(beta.rows() + tau.rows(), 1);

	//right-hand side of equality constraint
	Matrix right_hand( JTD.rows() + 1, 1);
	right_hand.setAllElements(0.0);
	//actually, we use 1.0e9 here as units are in N * 1.0e-6 * mm
	//so we want a total joint torque of 1000 N mm
	right_hand.elem( right_hand.rows()-1, 0) = 1.0e10;

	//left-hand side of equality constraint
	Matrix LeftHand( JTD.rows() + 1, D.cols() + numJoints);
	LeftHand.setAllElements(0.0);
	LeftHand.copySubMatrix(0, 0, JTD);
	LeftHand.copySubMatrix(0, D.cols(), Matrix::NEGEYE(numJoints, numJoints) );
	for (int i=0; i<numJoints; i++) {
		LeftHand.elem( JTD.rows(), D.cols() + i) = 1.0;
	}

	//matrix F padded with zeroes for tau
	//left hand side of the inequality matrix
	Matrix FO(F.rows(), F.cols() + numJoints);
	FO.setAllElements(0.0);
	FO.copySubMatrix(0, 0, F);

	//right-hand side of inequality matrix
	Matrix inEqZeroes(FO.rows(), 1);
	inEqZeroes.setAllElements(0.0);

	//objective matrix: G padded with zeroes 
	Matrix GO(Matrix::ZEROES<Matrix>(G.rows(), G.cols() + numJoints));
	GO.copySubMatrix(0, 0, G);

	//bounds: all variables greater than 0
	// CHANGE: only beta >= 0, tau is not
	Matrix lowerBounds(Matrix::MIN_VECTOR(beta_tau.rows()));
	lowerBounds.copySubMatrix( 0, 0, Matrix::ZEROES<Matrix>(beta.rows(), 1) );
	Matrix upperBounds(Matrix::MAX_VECTOR(beta_tau.rows()));


	/*
	FILE *fp = fopen("gfo.txt","w");
	fprintf(fp,"left hand:\n");
	LeftHand.print(fp);
	fprintf(fp,"right hand:\n");
	right_hand.print(fp);
	fprintf(fp,"friction inequality:\n");
	FO.print(fp);
	fprintf(fp,"Objective:\n");
	GO.print(fp);
	fclose(fp);
	*/
	// assembled system:
	// minimize beta_tauT*QOT*QO*beta_tau subject to:
	// LeftHand * beta_tau = right_hand
	// FO * beta_tau <= inEqZeroes
	// beta_tau >= 0
	// CHANGE: only beta >= 0, tau is not
	int result = factorizedQPSolver(GO, LeftHand, right_hand, FO, inEqZeroes, 
									lowerBounds, upperBounds,
									beta_tau, objVal);
	beta.copySubBlock(0, 0, beta.rows(), 1, beta_tau, 0, 0);
	tau.copySubBlock(0, 0, tau.rows(), 1, beta_tau, beta.rows(), 0);
	return result;
}

/*! One possible formulation of the core GFO problem. Checks if some
	combination of contacts forces exists so that the resultant object
	wrench is 0. See inner comments for exact mathematical formulation.
	Not for standalone use; is called by the GFO functions in the 
	Grasp class.
*/
int
contactForceExistence(Matrix &F, Matrix &N, Matrix &Q, Matrix &beta, double *objVal)
{
	// exact problem formulation
	// unknowns: beta					(contact forces)
	// minimize betaT*QT*Q*beta			(magnitude of resultant object wrench)
	// subject to:
	// sum_normal * beta = 1			(we are applying some contact forces)
	// F * beta <= 0					(all forces inside friction cones)
	// beta >= 0	  				    (all forces must be positive)

	Matrix right_hand(1,1);
	//a total of 10N of normal force
	right_hand.elem(0,0) = 1.0e7;

	//right-hand side of inequality matrix
	Matrix inEqZeroes(F.rows(), 1);
	inEqZeroes.setAllElements(0.0);

	//bounds: all variables greater than 0
	Matrix lowerBounds(Matrix::ZEROES<Matrix>(beta.rows(),1));
	Matrix upperBounds(Matrix::MAX_VECTOR(beta.rows()));

	/*
	FILE *fp = fopen("gfo.txt","w");
	fprintf(fp,"N:\n");
	N.print(fp);
	fprintf(fp,"right hand:\n");
	right_hand.print(fp);
	fprintf(fp,"friction inequality:\n");
	F.print(fp);
	fprintf(fp,"Objective:\n");
	Q.print(fp);
	fclose(fp);
	*/
	int result = factorizedQPSolver(Q, N, right_hand, F, inEqZeroes, 
									lowerBounds, upperBounds,
									beta, objVal);
	return result;
}

/*! One possible formulation of the core GFO problem. Finds the contacts
	forces that result in 0 object wrench and are as far as possible 
	from the edges of the friction cones. Assumes that at least one set
	of contact forces that satisfy this criterion exist; see 
	contactForceExistence for that problem. See inner comments for exact 
	mathematical formulation. Not for standalone use; is called by the GFO 
	functions in the Grasp class.
*/
int
contactForceOptimization(Matrix &F, Matrix &N, Matrix &Q, Matrix &beta, double *objVal)
{
	// exact problem formulation
	// unknowns: beta					(contact forces)
	// minimize sum * F * beta			(crude measure of friction resistance abilities)
	// subject to:
	// Q * beta = 0						(0 resultant object wrench)
	// sum_normal * beta = 1			(we are applying some contact forces)
	// F * beta <= 0					(each individual forces inside friction cones)
	// beta >= 0	  				    (all forces must be positive)
	// overall equality constraint:
	// | Q | |beta|   |0|
	// | N |        = |1|

	//right hand of equality
	Matrix right_hand(Matrix::ZEROES<Matrix>(Q.rows()+1, 1));
	//a total of 10N of normal force
	right_hand.elem(Q.rows(),0) = 1.0e7;

	//left hand of equality
	Matrix LeftHand(Q.rows() + 1, Q.cols());
	LeftHand.copySubMatrix(0, 0, Q);
	LeftHand.copySubMatrix(Q.rows(), 0, N);

	//bounds: all variables greater than 0
	Matrix lowerBounds(Matrix::ZEROES<Matrix>(beta.rows(),1));
	Matrix upperBounds(Matrix::MAX_VECTOR(beta.rows()));

	//objective: sum of F
	Matrix FSum(1,F.rows());
	FSum.setAllElements(1.0);
	Matrix FObj(1,F.cols());
	matrixMultiply(FSum, F, FObj);
	/*
	FILE *fp = fopen("gfo.txt","w");
	fprintf(fp,"Left Hand:\n");
	LeftHand.print(fp);
	fprintf(fp,"right hand:\n");
	right_hand.print(fp);
	fprintf(fp,"friction inequality:\n");
	F.print(fp);
	fprintf(fp,"Objective:\n");
	Q.print(fp);
	fclose(fp);
	*/
	int result = LPSolver(FObj, LeftHand, right_hand, F, Matrix::ZEROES<Matrix>(F.rows(), 1), 
						  lowerBounds, upperBounds, 
						  beta, objVal);
	return result;
}

/*! One possible formulation of the core GFO problem. Finds the joint
	forces that result in 0 object wrench such that contact forces are as 
	far as possible from the edges of the friction cones. Assumes that at 
	least one set of contact forces that satisfy this criterion exist; see 
	contactForceExistence for that problem. See inner comments for exact 
	mathematical formulation. Not for standalone use; is called by the GFO 
	functions in the Grasp class.
*/
int
graspForceOptimization(Matrix &JTD, Matrix &D, Matrix &F, Matrix &G, 
                       Matrix &beta, Matrix &tau, double *objVal)
{
	// exact problem formulation
	// unknowns: [beta tau]            (contact forces and joint forces)
	// minimize [sum] [F 0] [beta tau] (sort of as far inside the friction cone as possible, not ideal)
	// subject to:
	// [JTD -I] * [beta tau] = 0       (contact forces balance joint forces)
	// [G 0]* [beta tau] = 0           (0 resultant object wrench)
	// [0 sum] * [beta tau] = 1        (we are applying some joint forces)
	// [F 0] [beta tau] <= 0		   (all forces inside friction cones)
	// [beta tau] >= 0	  		       (all forces must be positive)
	// overall equality constraint:
	// | JTD -I |  | beta |   |0|
	// | G    0 |  | tau  | = |0|
	// | 0   sum|		      |1|

	Matrix beta_tau(beta.rows() + tau.rows(), 1);
	int numJoints = tau.rows();

	//right-hand side of equality constraint
	Matrix right_hand( JTD.rows() + G.rows() + 1, 1);
	right_hand.setAllElements(0.0);
	//actually, we use 1.0e8 here as units are in N * 1.0e-6 * mm
	//so we want a total joint torque of 100 N mm
	right_hand.elem( right_hand.rows()-1, 0) = 1.0e10;

	//left-hand side of equality constraint
	Matrix LeftHand( JTD.rows() + G.rows() + 1, D.cols() + numJoints);
	LeftHand.setAllElements(0.0);
	LeftHand.copySubMatrix(0, 0, JTD);
	LeftHand.copySubMatrix(0, D.cols(), Matrix::NEGEYE(numJoints, numJoints) );
	LeftHand.copySubMatrix(JTD.rows(), 0, G);
	for (int i=0; i<numJoints; i++) {
		LeftHand.elem( JTD.rows() + G.rows(), D.cols() + i) = 1.0;
	}

	//objective matrix
	//matrix F padded with zeroes for tau
	//will also serve as the left hand side of the inequality matrix
	Matrix FO(F.rows(), F.cols() + numJoints);
	FO.setAllElements(0.0);
	FO.copySubMatrix(0, 0, F);
	//summing matrix and objective matrix
	Matrix FSum(1, F.rows());
	FSum.setAllElements(1.0);
	Matrix FObj(1, FO.cols());
	matrixMultiply(FSum, FO, FObj);

	//bounds: all variables greater than 0
	Matrix lowerBounds(Matrix::ZEROES<Matrix>(beta_tau.rows(),1));
	Matrix upperBounds(Matrix::MAX_VECTOR(beta_tau.rows()));

	//right-hand side of inequality matrix
	Matrix inEqZeroes(FO.rows(), 1);
	inEqZeroes.setAllElements(0.0);

	
	FILE *fp = fopen("gfo.txt","w");
	fprintf(fp,"left hand:\n");
	LeftHand.print(fp);
	fprintf(fp,"right hand:\n");
	right_hand.print(fp);
	fprintf(fp,"friction inequality:\n");
	FO.print(fp);
	fprintf(fp,"Objective:\n");
	FObj.print(fp);
	fclose(fp);
	

	// assembled system:
	// minimize FObj * beta_tau subject to:
	// LeftHand * beta_tau = right_hand
	// FO * beta_tau <= inEqZeroes
	// beta_tau >= 0
	int result = LPSolver(FObj, LeftHand, right_hand, FO, inEqZeroes,
						  lowerBounds, upperBounds, 
						  beta_tau, objVal);
	beta.copySubBlock(0, 0, beta.rows(), 1, beta_tau, 0, 0);
	tau.copySubBlock(0, 0, tau.rows(), 1, beta_tau, beta.rows(), 0);
	return result;
}

/*! Retrieve all the joints of the robot, but only if their chains have contacts on
  them. Joints on chains with no contact have a trivial 0 solution so they only
  make the problem larger for no good reason.
  we could go even further and only keep the joints that come *before* the contacts
  in the chain.
*/
std::list<Joint*> Grasp::getJointsOnContactChains()
{
	std::list<Joint*> joints;
	for (int c=0; c<hand->getNumChains(); c++) {
		if (hand->getChain(c)->getNumContacts(object) != 0) {
			std::list<Joint*> chainJoints = hand->getChain(c)->getJoints();
			joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
		}
	}
        return joints;
}

/*! This function is the equivalent of the Grasp Force Optimization, but done with
  the quasi-static formulation cast as a Quadratic Program.
  
  It can perform four types of computation:
  - contact force existence: are there contact forces that balance out on the object
  - contact force optimization: what are the optimal contact forces (as far as possible
    from the edges of the friction cones) that balance out on the object
  - grasp force existence: are there joint forces which produce contact forces which 
    balance out on the object
  - grasp force optimization: what are the optimal joint forces, producing contact
    forces that are as far as possible from the edges of the friction cones and 
    balance out on the object.
   See individual computation routines for more details.
  
  There might exist cases of grasps that are reported as form-closed where grasp force
  existence fails, as this function also asks that the contact forces that balance
  the object must be possible to apply from actuated DOF's.
  
  For now, this function displays the computed contact forces on the contacts, 
  rather than returning them. It also copies the computed joint forces in the
  matrix \a robotTau which is assumed to be large enough for all the joints of
  the robot and use the robot's numbering scheme.
  
  Return codes: 0 is success, >0 means problem is unfeasible, no equilibrium forces
  exist; <0 means error in computation 
*/
int 
Grasp::computeQuasistaticForcesAndTorques(Matrix *robotTau, int computation)
{
	//use the pre-set list of contacts. This includes contacts on the palm, but
	//not contacts with other objects or obstacles
	std::list<Contact*> contacts;
	contacts.insert(contacts.begin(),contactVec.begin(), contactVec.end());
	//if there are no contacts we are done
	if (contacts.empty()) return 0;

        //get only the joints on chains that make contact;
	std::list<Joint*> joints = getJointsOnContactChains();

	//build the Jacobian and the other matrices that are needed.
	//this is the same as in the equilibrium function above.
	Matrix J(contactJacobian(joints, contacts));
	Matrix D(Contact::frictionForceBlockMatrix(contacts));
	Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
	Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));

	Matrix N(Contact::normalForceSumMatrix(contacts));

	//grasp map that relates contact amplitudes to object wrench G = S*R*D
	Matrix G(graspMapMatrix(R,D));

	//matrix that relates contact forces to joint torques JTD = JTran * D
	Matrix JTran(J.transposed());
	Matrix JTD(JTran.rows(), D.cols());
	matrixMultiply(JTran, D, JTD);

	// vectors of unknowns
	Matrix beta( D.cols(), 1);
	Matrix tau( (int)joints.size(), 1);

	double objVal;
	/* This is the core computation. There are many ways of combining the 
	   optimization criterion and the constraints. Four of them are presented 
	   here, each encapsulated in its own helper function.
        */

        int result;
        switch(computation)
        {
        case GRASP_FORCE_EXISTENCE:
          result = graspForceExistence(JTD, D, F, G, beta, tau, &objVal);
          break;
        case GRASP_FORCE_OPTIMIZATION:
          result = graspForceOptimization(JTD, D, F, G, beta, tau, &objVal);
          break;
        case CONTACT_FORCE_EXISTENCE:
          result = contactForceExistence(F, N, G, beta, &objVal);
          matrixMultiply(JTD, beta, tau);
          break;
        case CONTACT_FORCE_OPTIMIZATION:
          result = contactForceOptimization(F, N, G, beta, &objVal);
          matrixMultiply(JTD, beta, tau);
          break;
        default:
          DBGA("Unknown computation type requested");
          return -1;
        }

	if (result) {
		if( result > 0) {
			DBGA("Grasp: problem unfeasible");
		} else {
			DBGA("Grasp: solver error");
		}
		return result;
	}
	DBGA("Optimization solved; objective: " << objVal);

	DBGP("beta:\n" << beta);
	DBGP("tau:\n" << tau);
	DBGP("Joint forces sum: " << tau.elementSum());

	Matrix Gbeta(G.rows(), beta.cols());
	matrixMultiply(G, beta, Gbeta);
	DBGP("Total object wrench:\n" << Gbeta);

	//retrieve contact wrenches in local contact coordinate systems
	Matrix cWrenches(D.rows(), 1);
	matrixMultiply(D, beta, cWrenches);
	DBGP("Contact forces:\n " << cWrenches);

	//compute object wrenches relative to object origin and expressed in world coordinates
	Matrix objectWrenches(R.rows(), cWrenches.cols());
	matrixMultiply(R, cWrenches, objectWrenches);
	DBGP("Object wrenches:\n" << objectWrenches);

	//display them on the contacts and accumulate them on the object
	displayContactWrenches(&contacts, cWrenches);
	accumulateAndDisplayObjectWrenches(&contacts, objectWrenches);

	//set the robot joint values for the return
	std::list<Joint*>::iterator it;
	int jc;
	for(it=joints.begin(), jc=0; it!=joints.end(); it++,jc++) {
		robotTau->elem( (*it)->getNum(), 0 ) = 1.0 * tau.elem(jc,0);
	}

	//sanity check: contact forces balance joint forces

	//sanity check: resultant object wrench is 0

	return 0;
}

/*! This is a helper function for grasp force optimization routines. Given a
	set of contacts and a matrix of contact wrenches expressed in *local* 
	contact coordinates, it will set these wrenches in the dynamic wrench slot
	of the contacts for rendering purposes.
*/
void
Grasp::displayContactWrenches(std::list<Contact*> *contacts, 
						      const Matrix &contactWrenches)
{
	int count = 0;
	std::list<Contact*>::iterator it;
	for (it=contacts->begin(); it!=contacts->end(); it++, count++) {
		Contact *contact = *it;
		if (contact->getBody2()->isDynamic()) {
			//wrench is also scaled down for now for rendering and output purposes
			//we also transform the wrench to the mate's coordinate system, which
			//usually involves negating the x and z axes
			double dynWrench[6];
			for (int i=0; i<6; i++) {
				dynWrench[i] = -1.0 * 1.0e-6 * contactWrenches.elem(6*count+i,0);
			}
			//the y axis is not negated
			dynWrench[1] = -1.0 * dynWrench[1];
			dynWrench[4] = -1.0 * dynWrench[4];
			contact->getMate()->setDynamicContactWrench(dynWrench);
		}
	}
}


/*! This is a helper function for grasp force optimization routines. Given a
	set of contacts, and a matrix of contact wrenches expressed in *world*
	coordinates and relative to object origin (as computed in gfo routines)
	this function will convert them to object coordinate system and accumulate
	them in the object's external wrench accumulator. This can serve as an
	output of the gfo functions and also for rendering purposes, as the 
	external wrench accumulator can then be rendered.
*/
void
Grasp::accumulateAndDisplayObjectWrenches(std::list<Contact*> *contacts, 
										  const Matrix &objectWrenches)
{
	int count = 0;
	std::list<Contact*>::iterator it;
	for (it=contacts->begin(); it!=contacts->end(); it++, count++) {
		Contact *contact = *it;
		vec3 force(objectWrenches.elem(6*count+0,0), 
					objectWrenches.elem(6*count+1,0), 
					objectWrenches.elem(6*count+2,0));
		vec3 torque(objectWrenches.elem(6*count+3,0), 
					objectWrenches.elem(6*count+4,0), 
					objectWrenches.elem(6*count+5,0));
		if (contact->getBody2()->isDynamic()) {
			DynamicBody *object = (DynamicBody*)(contact->getBody2());
			//compute force and torque in body reference frame
			//and scale them down for now for rendering and output purposes
			force = 1.0e-6 * force * object->getTran().inverse();
			//torque is also scaled by maxRadius in conversion matrix
			torque = 1.0e-6 * torque * object->getTran().inverse();
			//accumulate them on object
			object->addForce(force);
			object->addTorque(torque);
		}
	}
}
