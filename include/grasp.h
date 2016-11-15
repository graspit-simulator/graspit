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
// $Id: grasp.h,v 1.26 2009/07/21 19:30:27 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the grasp class, which analyzes grasps
 */
#ifndef GRASP_HXX

#include <list>
#include <vector>
#include <set>
#include <QObject>

//Inventor includes
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
class SoTransform;
class SoCoordinate3;
class SoIndexedFaceSet;

class vec3;
class transf;
class position;
class Hand;
class GraspableBody;
class Contact;
class VirtualContact;
class Link;
class GWS;
class QualityMeasure;
class QMDlg;
class GWSprojection;
class Matrix;
class Joint;

//! Max iteration steps in a feasibility phase
#define MAX_FEAS_LOOPS	100 
//! Max iteration steps in a optimization phase
#define MAX_OPTM_LOOPS  100 

extern bool saveSetup;
extern int saveCounter;

//! A grasp occurs between a hand and an object and has quality measures associated with it.  
//  It also has methods to optimize grasping forces.
/*! Each hand object has a grasp associated with it, and the grasp occurs
    between the hand and a graspableBody.  Each grasp is incharge of
	maintaining	a set of computed grasp wrench spaces, which are then used
	by various quality measures.  When contacts change between the two, the 
	grasp is updated, meaning each grasp wrench space associated with the 
	grasp is rebuilt, as well as any grasp wrench space projections.  All
    grasp quality measures are also re-evaluated. 

	The grasp wrench spaces are designed to be shared by different quality 
	metrics. When a new metric is added, if a GWS of the needed type already
	exists in the list maintained by thhis grasp, the new QM will use it, 
	and a reference count on the GWS is increased. GWS are automatically 
	deleted when no QM that needs them exist anymore.
	
	A grasp can also perform many of the same services on set of 
	VirtualCotacts (see documentation of that class for details). This part
	however has not neen competely tested, and might produce behavior that
	is not entirely consistent with how it handles traditional contacts.

	This class also provides the mechanism for performing Grasp Force
	Optimization (GFO) computations using a Quadratic Program (QP) formulation.
	There are many variants of this computation; some of them are already
	written. You can generally mix and match between optimization criteria
	and constraints in any way you want. A good starting point is the
	computeQuasistaticForcesAndTorques(...) function.
*/
class Grasp : public QObject{
  Q_OBJECT

public:
  //! Default option for building a GWS on all 6 dimensions
  static const std::vector<int> ALL_DIMENSIONS;

protected:
  friend class LMIOptimizer;

  //! A pointer to the hand that owns this grasp object
  Hand *hand;

  //! A pointer to the object that is the focus of this grasp
  GraspableBody *object;

  //! \c TRUE if the grasp has been updated since the last time the contacts changed
  bool valid;

  //! A list of pointers to the associated grasp wrench spaces
  std::list<GWS *> gwsList;

  //! A list of pointers to the associated quality measures
  std::list<QualityMeasure *> qmList;

  //! A list of pointer to the associated grasp wrench space projections
  std::list<GWSprojection *> projectionList;

  //! Number of quality meausre in the list
  int numQM;

  //! A vector of pointers to the contacts on the object where it touches the hand 
  std::vector<Contact *> contactVec;

  //! Number of grasp contacts
  int numContacts;

  //! Minimum grasp wrench that can be applied given contact forces that sum to 1
  double minWrench[6];  

  //! Tells us if quality metrics should take into account gravity
  bool useGravity;

  //! Computes the Jacobian of a link wrt the base of the finger, in link coordinates
  double *getLinkJacobian(int f, int l);

  //! Sets the reference point that is used for grasp wrench computations as the center of the virtual contacts
  void setVirtualCentroid();

  //! Computes the virtual center of the internally assembled list of virtual contacts
  vec3 virtualCentroid();

  //! Sets the reference points of all virtual contacts using the c.o.g of the given object 
  void setRealCentroid(GraspableBody *body);

  friend class QMDlg;

signals:
  //! Called when contacts have changes and the wrench spaces need to be updated
  void graspUpdated();

public:
  Grasp(Hand *h);

  ~Grasp();

  /*! Returns whether the grasp has been updated since the last time grasp
    contacts have changed. */
  bool                    isValid() const {return valid;}

  /*! Returns whether the grasp force optimization problem is feasible. */
  //bool                    isFeasible() const {return feasible;}

  /*! Returns the number of quality measures defined for this grasp. */
  int                     getNumQM() const {return numQM;}

  /*! Return a pointer to the object that is the focus of this grasp. */
  GraspableBody *         getObject() const {return object;}

  /*! Return the number of grasp contacts. */
  int                     getNumContacts() const {return numContacts;}

  /*! Return a pointer to the i-th grasp contact on the object. */
  Contact *               getContact(int i) const {return contactVec[i];}

  void                    getMinWrench(double *w) const
  {
    if (w) memcpy(w,minWrench,6*sizeof(double));
  }

  void                    setMinWrench(double *w)
  {
    if (w) memcpy(minWrench,w,6*sizeof(double));
  }

  /*! Sets graspableBody \a g to be the new focus of the grasp and updates the
    grasp. */
  void                    setObject(GraspableBody *g) {object = g; update();}

  /* this is a hack; I had to do it due to some bug I was never able to trace down*/
  void                    setObjectNoUpdate(GraspableBody *g) {object = g;}

  //! Collects all the contacts between the hand and the object in an internal list
  void collectContacts();
  //! Collects all the virtual contacts on the hand n an internal list
  void collectVirtualContacts();
  //! Collect all the virtual contacts on the object
  void collectVirtualContactsOnObject();

  //! Collects all the contacts in the internal list and updates the wrench spaces
  void update(std::vector<int> useDimensions = ALL_DIMENSIONS);
  //! Updates (re-computes) the wrench spaces of this grasp and all of their projections
  void updateWrenchSpaces(std::vector<int> useDimensions = ALL_DIMENSIONS);

  //! Returns the max radius used in GWS computations, either from the object of from virtual contacts
  double getMaxRadius();
  //! Returns the c.o.g. used in GWS computations, either from the object of from virtual contacts
  position getCoG();

  //! Adds a GWS of a given type to the grasp, unless one exists already
  GWS *addGWS(const char *type);
  GWS *getGWS(const char *type);
  //! Decrements the reference count on a GWS of the given type, and deletes it if the ref count reaches 0
  void removeGWS(GWS *gws);
  //! Adds a quality measure to this grasp
  void addQM(QualityMeasure *qm);
  //! Replaces a QM in the list associated with this grasp with another QM
  void replaceQM(int which,QualityMeasure *qm);
  //! Returns one of the quality measures that have been associated with this grasp
  QualityMeasure *getQM(int which);
  //! Removes a quality measure that has been associated with this grasp
  void removeQM(int which);
  
  void addProjection(GWSprojection *gp);
  void removeProjection(GWSprojection *gp);
  static void destroyProjection(void * user, SoQtComponent * component);

  //! Sets whether QM's should take gravity into account; not very thoroughly tested yet
  void setGravity(bool g){useGravity = g;}
  bool isGravitySet(){return useGravity;}

  //------------------- Grasp Force Optimization (GFO) routines --------------------------

  static const int CONTACT_FORCE_EXISTENCE;
  static const int CONTACT_FORCE_OPTIMIZATION;
  static const int GRASP_FORCE_EXISTENCE;
  static const int GRASP_FORCE_OPTIMIZATION;

  //! Computes the contact forces and joint torques that give the most robust equilibrium
  int computeQuasistaticForcesAndTorques(Matrix *robotTau, int computation);

  //! Given a vector of joint torques, computes the contact forces that balance the system
  int computeQuasistaticForces(const Matrix &robotTau);

  //! Gets a list with only the joints on chains that have contacts on them
  std::list<Joint*> getJointsOnContactChains();

  //! A version of the contact grasp Jacobian, used for GFO routines
  Matrix contactJacobian(const std::list<Joint*> &joints, 
                         const std::list< std::pair<transf, Link*> > &contact_locations);

  //! A convenience version of the Jacobian function that takes in a list of contacts
  Matrix contactJacobian(const std::list<Joint*> &joints, 
                         const std::list<Contact*> &contacts);

  //! A convenience version of the Jacobian function that takes in a list of virtual contacts
  Matrix contactJacobian(const std::list<Joint*> &joints, 
                         const std::list<VirtualContact*> &contacts);

  //! Computes the grasp map matrix G from friction and normal force matrices R and D
  static Matrix graspMapMatrix(const Matrix &R, const Matrix &D);

  //! Sets local contact wrenches into the contact wrench slots so they can be rendered
  void displayContactWrenches(std::list<Contact*> *contacts, const Matrix &contactWrenches);
  //! Accumulates object wrenches in the external wrench accumulator for the objects
  void accumulateAndDisplayObjectWrenches(std::list<Contact*> *contacts, 
										  const Matrix &objectWrenches);
};

#define GRASP_HXX
#endif
