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
// $Id: contact.h,v 1.37 2009/06/16 22:53:24 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the contact class hierarchy
 */
#ifndef CONTACT_HXX
#define CONTACT_HXX

#include <Inventor/nodes/SoSeparator.h>
#include <list>
#include <vector>

#include "graspit/Collision/collisionStructures.h"

class transf;
class Body;
class SoSeparator;
class SoMaterial;
class Matrix;
class VirtualContact;

#define DISPLAY_CONE_SCALE 20.0
#define MAX_FRICTION_EDGES 100


//! Friction type: Frictionless, Point contact with friction, Soft finger contact with elliptic approximation, Soft finger contact with linearized elliptic approximation
enum frictionT {FL, PCWF, SFCE, SFCL};

//!  A wrench is a 6-vector containing 2 3-vectors for the force and torque components
struct Wrench {
  vec3 force;
  vec3 torque;
};

//! A contact between 2 bodies
/*! Contacts always come in pairs.  When two bodies touch, a contact is defined
  for each of them.  They have the same (or very close to the same) global
  positions, opposite inward pointing normals, and share the same coefficients
  of friction. Any instance of this class will have as mate its reciprocal
  contact on the other body.

  A contact is defined in places where two bodies are separated by less then
  the contact threshold (usually set to 0.1 mm). In GraspIt, two bodies are
  NEVER allowed to interpenetrate; they are considered in contact if they are
  apart, but separated by less than the threshold.

  We always define contacts to occur precisely at a point (as opposed to a small
  area). This makes computations much easier, as we don't have to compute
  deformations. The Grasp class will however consider the case of objects that
  match geometry so that many point contacts can occur on the same patch. For
  soft bodies, we have the SoftContact class which inherits from this one and
  tries to approximate the frictional effects of having a contact patch (while
  not computing such a patch explicitly).

  Contacts also define their own linearized friction models. This means that,
  for each contact, friction is constrained to lie within some convex polyhedron.
  The edges that define this polyhedron are different for different types of
  contacts, which inherit from this class. However, once the edges of the friction
  polyhedron are defined, all contact behave identically.

  You will also find here code for two projects which are not used right now. The
  first one if Grasp Force Optimization (GFO). See the grasp class for that code;
  it is finished but has never been tested. Also, the GFO code has never been updated
  to work with the new Contact hierarchy of PointContact, SoftContact etc, and uses a
  parallel mechanism to keep track of friction models. The second one concerns the
  dynamics engine: in theory, we can try to save contact information from a time step
  to help the solver during the next time step. The framework is in place, but we have
  never used it.
 */
class Contact
{

    friend class VirtualContact;

  protected:
    //! Pointer to the body this contact is on
    Body *body1;

    //! Pointer to the other body involved in this contact
    Body *body2;

    //! Points to the other contact in this pair
    Contact *mate;

    //! Coefficient of static friction
    double sCof;

    //! Coefficient of kinetic friction
    double kcof;

    //! The type of friction model at this contact
    frictionT frictionType;

    //! Contact dimension (num basisvecs): 1 for FL, 3 for PCWF, 4 for SFCE, 4 for SFCL
    int contactDim;

    //! LMI dimension: 1 for FL, 3 for PCWF, 4 for SFCE, 7 for SFCL
    int lmiDim;

    //! Coordinates of the contact with respect to body 1 base frame
    position loc;

    //! Contact normal with respect to body 1 base frame
    vec3 normal;

    //! Pose of the contact frame with respect to the body1 frame.
    transf frame;

    //! Pose of body 1 in world space at time of contact
    transf body1Tran;

    //! Pose of body 2 in world space at time of contact
    transf body2Tran;

    //! Root Inventor node stores arrow geometry representing current contact force
    SoSeparator *contactForcePointers;

    //! The maximum normal force for this contact point due to hand torque limits
    double normalForceLimit;

    //! A contactDim vector containing the optimal values to multiply with basisVecs. This will produce the contact forces necessary to generate the needed object wrench.
    double *optimalCoeffs;

    //! The current dynamic force acting at this contact
    double dynamicForce[6];

    //! Dynamics LCP information from the previous time step;
    double prevCn;
    //! Dynamics LCP information from the previous time step;
    double prevLambda;
    //! Dynamics LCP information from the previous time step;
    double *prevBetas;
    //! Tells us wether this contact has inherited some information from the previous dynamic time step.
    bool inheritanceInfo;

    //! Based on LCP solution, this flag shows if the contact is slipping or not. Does not work well.
    bool mSlip;

    //! computes a single contact wrench from a friction edge
    void wrenchFromFrictionEdge(double *edge, const vec3 &radius, Wrench *wr);

    //! Sets up frictional forces at this contact using a linearized ellipsoid
    int setUpFrictionEllipsoid(int numLatitudes, int numDirs[], double phi[], double eccen[]);

  public:

    //! 6 x numFrictionEdges matrix of friction cone boundary wrenches used in dynmaics
    /*! Friction edges contain "normalised" friction information: just the
      frictional component, and without reference to normal force or coeff
      of friction (c.o.f.). They only care about the relationship between
      frictional components.
    */
    double frictionEdges[6 * MAX_FRICTION_EDGES];

    //! number of friction edges defining the frictional component of contact wrenches
    int numFrictionEdges;

    //! Array of wrenches bounding the Contact Wrench Space (CWS)
    /*! The CWS is a convex polyhedron that defines the space of wrenches that
      can be applied at this contact. Also encapsulates normal forces (usually
      normalised to 1N) and the relationship between normal and frictional
      components (usually encapsulated in the coefficient of friction).
    */
    Wrench *wrench;

    //! Number of total friction wrenches.
    int numFCWrenches;

    //! Initializes an empty contact (not really used)
    Contact() : body1(NULL), body2(NULL), mate(NULL), sCof(0.0),
      contactForcePointers(NULL), optimalCoeffs(NULL), prevBetas(NULL), wrench(NULL) {}

    //! Constructs a contact between two bodies
    Contact(Body *b1, Body *b2, position pos, vec3 norm);

    //! Destructor
    virtual ~Contact();

    //! Converts pure friction edges into full contact wrenches by considering normal force
    virtual void computeWrenches();

    //! Main function for defining the space of friction that can be applied at this contact.
    virtual int setUpFrictionEdges(bool dynamicsOn = false) = 0;

    /*! Connects the mate contact to this one */
    void setMate(Contact *m) {mate = m;}

    /*! Returns contact position wrt Body1 frame. */
    position getPosition() {return loc;}

    /*! Return contact normal wrt Body1 frame. */
    vec3 getNormal() {return normal;}

    /*! Returns a pointer to the other half of this contact pair. */
    Contact *getMate() const {return mate;}

    /*! Returns a pointer to body this contact is on. */
    Body *getBody1() const {return body1;}

    /*! Returns a pointer to the other body invovled in this contact. */
    Body *getBody2() const {return body2;}

    /*! Returns the pose of body1 when this contact was formed. */
    transf getBody1Tran() const {return body1Tran;}

    /*! Returns the pose of body2 when this contact was formed. */
    transf getBody2Tran() const {return body2Tran;}

    //! Gets the coefficient of friction for this contact
    double getCof() const;

    //! Gets the location
    position getLocation() const {return loc;}

    //! Gets the frame
    transf getFrame() const { return frame;}

    //! Updates the coefficient of friction (called when the body materials have changed)
    void updateCof();

    /*! Returns the type of friction modeled at this contact. */
    frictionT getFrictionType() const {return frictionType;}

    /*! Returns pose of the contact frame relative to the base frame of body1. */
    transf getContactFrame() const {return frame;}

    /*! Returns the current dynamic force acting at this contact. */
    double *getDynamicContactWrench() {return dynamicForce;}

    /*! Returns the Inventor root of the pointer geometry for this contact */
    SoSeparator *getContactForcePointers() const {return contactForcePointers;}

    /*! Determines whether this contact will prevent motion of body 1 as expressed in local body 1 coordinates */
    bool preventsMotion(const transf &motion) const;

    //! Called by GFO routine to set optimal contact force
    void setContactForce(double *optmx);

    /*! Returns the contact dimension (number of basis wrenches). */
    int getContactDim() const {return contactDim;}

    /*! Returns the dimension of the LMI block for this contact. (used in GFO) */
    int getLmiDim() const {return lmiDim;}

    /*! Sets the normal force limit based on hand torque limits as computed during GFO. */
    void setNormalForceLimit(double nfl) {normalForceLimit = nfl;}

    /*! Returns the maximum normal force due to torque limits in the hand computed by GFO.  */
    double getNormalForceLimit() const {return normalForceLimit;}

    //------------------- QuasiStatic equilibrium matrices and helper functions ---------------------

    //! Returns a matrix for friction constraints at this contact that can be used in an LCP
    Matrix frictionConstraintsMatrix() const;
    //! Returns a block matrix composed of individual constraint matrices for the contacts in the list
    static Matrix frictionConstraintsBlockMatrix(const std::list<Contact *> &contacts);

    //! Returns the matrix that relates friction edge amplitudes to normal and frictional force
    Matrix frictionForceMatrix() const;
    //! Returns a block matrix made up of individual force matrices from the contacts in the list
    static Matrix frictionForceBlockMatrix(const std::list<Contact *> &contacts);

    //! Returns the matrix that transforms a force on this contact into a wrench on the other body
    Matrix localToWorldWrenchMatrix() const;
    //! Returns a block matrix made up of individual force to world wrench conversion matrices
    static Matrix localToWorldWrenchBlockMatrix(const std::list<Contact *> &contacts);

    //! Returns a matrix that adds just the normal force components of an amplitudes vector
    static Matrix normalForceSumMatrix(const std::list<Contact *> &contacts);

    //---------------------------------

    /*! Sets the dynamic force acting at this contact during the current time step. Used when drawing contact forces.*/
    void setDynamicContactWrench(double f[6])
    {memcpy(dynamicForce, f, 6 * sizeof(double));}

    /*! Sets just the force part of a dynamic wrench using a vec3*/
    void setDynamicContactForce(const vec3 &force) {
      dynamicForce[0] = force.x(); dynamicForce[1] = force.y(); dynamicForce[2] = force.z();
      dynamicForce[3] = dynamicForce[4] = dynamicForce[5] = 0;
    }

    /*! This contact inherits some properties from a contact from a previous dynamic time step.  */
    void inherit(Contact *c);

    //! The error that shows that this contact constraints are violated during dynamic simulation
    double getConstraintError();

    //! Maximum separation distance (in mm) between two bodies that are considered to be in contact
    static const double THRESHOLD;

    //! Maximum linear distance for which two contacts at consecutive time steps are considered to be the same contact
    static const double INHERITANCE_THRESHOLD;

    //! Maximum angular distance for which two contacts at consecutive time steps are considered to be the same contact
    static const double INHERITANCE_ANGULAR_THRESHOLD;

    //! Returns the IV root of the visual markers that shows the location of this contact
    virtual SoSeparator *getVisualIndicator() {return NULL;}

    virtual mat3 getRot() {return mat3::Identity();}

    //! Get dynamic sovler LCP information from the previous time step
    double getPrevCn() {return prevCn;}
    //! Get dynamic sovler LCP information from the previous time step
    double getPrevLambda() {return prevLambda;}
    //! Get dynamic sovler LCP information from the previous time step
    double *getPrevBetas() {return prevBetas;}
    //! Set dynamic sovler LCP information which might be used at the next time step
    void setLCPInfo(double cn, double l, double *betas);

    //! Returns the slip flag which does not work very well.
    bool isSlipping() const {return mSlip;}

    //! Tells us if this contact has inherited some information from previous time step
    bool inherits() {return inheritanceInfo;}

    //! A debug tool to see that contact inheritance works right
    SoMaterial *coneMat;
    //! A debug tool to see that contact inheritance works right
    float coneR, coneG, coneB;

    //! Testing static force output.  Default implementation for baseclass is trivial
    virtual void getStaticContactInfo(std::vector<position> &pVec, std::vector<double> &floatVec) {pVec.push_back(loc); floatVec.push_back(1);}
    //! Only used in SoftFinger contact
    virtual mat3 getCommonFrameRot() { return mat3::Identity(); }

};
#endif
