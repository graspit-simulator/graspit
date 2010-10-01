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

#include <Inventor/nodes/SoSeparator.h>
#include <list>
#include <vector>

#include "collisionStructures.h"

class transf;
class Body;
class SoSeparator;
class SoMaterial;
class Matrix;

#ifdef ARIZONA_PROJECT_ENABLED
#include <arizona/Arizona_Raw_Exp.h>
#endif

#define DISPLAY_CONE_SCALE 20.0
#define MAX_FRICTION_EDGES 100
  

//! Friction type: Frictionless, Point contact with friction, Soft finger contact with elliptic approximation, Soft finger contact with linearized elliptic approximation
enum frictionT {FL,PCWF,SFCE,SFCL};

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
  double cof;

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
  int setUpFrictionEllipsoid(int numLatitudes,int numDirs[], double phi[],double eccen[]);

public:

  //! 6 x numFrictionEdges matrix of friction cone boundary wrenches used in dynmaics
  /*! Friction edges contain "normalised" friction information: just the 
	  frictional component, and without reference to normal force or coeff
	  of friction (c.o.f.). They only care about the relationship between 
	  frictional components.
  */
  double frictionEdges[6*MAX_FRICTION_EDGES];

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
  Contact() : body1(NULL),body2(NULL),mate(NULL),cof(0.0),
	contactForcePointers(NULL), optimalCoeffs(NULL), prevBetas(NULL), wrench(NULL) {}

  //! Constructs a contact between two bodies 
  Contact(Body *b1,Body *b2, position pos, vec3 norm);

  //! Destructor
  virtual ~Contact();

  //! Converts pure friction edges into full contact wrenches by considering normal force
  virtual void computeWrenches();

  //! Main function for defining the space of friction that can be applied at this contact.
  virtual int setUpFrictionEdges(bool dynamicsOn = false)=0;

  /*! Connects the mate contact to this one */
  void setMate(Contact *m) {mate = m;}

  /*! Returns contact position wrt Body1 frame. */
  position getPosition(){return loc;}

  /*! Return contact normal wrt Body1 frame. */
  vec3 getNormal(){return normal;}

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
  bool preventsMotion(const transf& motion) const;

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
  static Matrix frictionConstraintsBlockMatrix(const std::list<Contact*> &contacts);

  //! Returns the matrix that relates friction edge amplitudes to normal and frictional force
  Matrix frictionForceMatrix() const;
  //! Returns a block matrix made up of individual force matrices from the contacts in the list
  static Matrix frictionForceBlockMatrix(const std::list<Contact*> &contacts);

  //! Returns the matrix that transforms a force on this contact into a wrench on the other body
  Matrix localToWorldWrenchMatrix() const;
  //! Returns a block matrix made up of individual force to world wrench conversion matrices
  static Matrix localToWorldWrenchBlockMatrix(const std::list<Contact*> &contacts);

  //! Returns a matrix that adds just the normal force components of an amplitudes vector
  static Matrix normalForceSumMatrix(const std::list<Contact*> &contacts);

  //---------------------------------

  /*! Sets the dynamic force acting at this contact during the current time step. Used when drawing contact forces.*/
  void setDynamicContactWrench(double f[6])
    {memcpy(dynamicForce,f,6*sizeof(double));}

  /*! Sets just the force part of a dynamic wrench using a vec3*/
  void setDynamicContactForce(const vec3 &force) {
    dynamicForce[0] = force.x(); dynamicForce[1] = force.y();dynamicForce[2] = force.z();
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
  virtual SoSeparator* getVisualIndicator(){return NULL;}

  //! Get dynamic sovler LCP information from the previous time step
  double getPrevCn(){return prevCn;}
  //! Get dynamic sovler LCP information from the previous time step
  double getPrevLambda(){return prevLambda;}
  //! Get dynamic sovler LCP information from the previous time step
  double *getPrevBetas(){return prevBetas;}
  //! Set dynamic sovler LCP information which might be used at the next time step
  void setLCPInfo(double cn, double l, double *betas);

  //! Returns the slip flag which does not work very well.
  bool isSlipping() const {return mSlip;}

  //! Tells us if this contact has inherited some information from previous time step
  bool inherits(){return inheritanceInfo;}

  //! A debug tool to see that contact inheritance works right
  SoMaterial *coneMat;
  //! A debug tool to see that contact inheritance works right
  float coneR, coneG, coneB;
};

//! A Point Contact With Friction (PCWF) implementing a Coulomb friction model
/*! The Point Contact simulates rigid bodies in contact. As such, its wrench
	space is a 3D cone and its friction space is a 2D circle with just 
	tangential friction. Its visual indicator is the cone itself. The cone's
	angle is equal to the tangent of the friction coefficient.
*/
class PointContact : public Contact
{
public:
	//! Also sets up friction edges according to Coulomb model
	PointContact(Body *b1, Body *b2, position pos, vec3 norm);
	//! Stub destructor
	~PointContact();
	//! Defines a 2D friction circle with tangential friction only.
	int setUpFrictionEdges(bool dynamicsOn = false);
	//! Returns the visual indicator which is the wrench space cone itself
	SoSeparator* getVisualIndicator();
};

//! Soft Contact implements an SFC model for contacts between soft bodies
/*! The SoftContact attempts to capture some of the frictional effects between
	soft bodies without explicitly computing the deformation at the contacts.
	It locally fits analytical surfaces to both bodies involved by computing
	their local radii of curvature. After this, it uses analytical models
	to compute the expected contact area and pressure distribution.
	
	Based on the contact area, it then tries to approximate the relationship
	between tangential friction and frictional torque using the limit surface,
	which becomes a 3D friction ellipsoid. The friction space is thus 3D, and
	the wrench space becomes 4D.
	
	Its visual indicator shows the analytical surface that has been fit to both
	bodies involved, as a small patch around the contact location.
*/
class SoftContact : public Contact
{
protected:
	//! A list of points from body1 that surround the contact in the frame of body1
	vec3 *bodyNghbd;

	//! The number of points in the current fit for analytical surfaces
	int numPts;

	//! Second order fit of points from body1 in the form r1x^2 + r2y^2
	double r1, r2;
	//! Parameters of unrotated fit in form z= ax^2+by^2+cxy
	double a, b, c;

	//! The relative angle between r1 and the mate's r1
	double relPhi;

	//! The rotation from the contact frame to the frame in which the fit is given by the radii of curvature, r1 and r2
	mat3 fitRot;

	//! The angle of the fitRot rotation
	double fitRotAngle;

	//! The major axis of the ellipse of contact
	double majorAxis;
	//! The minor axis of the ellipse of contact
	double minorAxis;

	//! The relative radii of curvature of the two bodies
	double r1prime;
	//! The relative radii of curvature of the two bodies
	double r2prime;

	//calculates the relative angle between the frames with 2 radii of curvature between
	//the contact and its mate
	int CalcRelPhi();

	//! Calculate the relative curvatures of the two bodies for contact dynamics
	int CalcRprimes();

	//! Fits an analytical surface to a local patch on body1 around the contact
	void FitPoints( );

	//! Calculates friction characteristics using a Mattress model
	double CalcContact_Mattress( double nForce );

public:
	//! Also takes a local neighborhood of points around body 1
	SoftContact( Body *b1, Body *b2, position pos, vec3 norm, Neighborhood *bn );

	//! Deletes its local record of the body neighborhood
	~SoftContact( );

	//! Friction model is a 3D friction ellipsoid also containing frictional torque
	int setUpFrictionEdges(bool dynamicsOn = false );

	//! Visual indicator is a small patch of the fit analytical surface on the body
	SoSeparator* getVisualIndicator();

	//! Also attempt to apply some torques in the contact plane; currently disabled.
	virtual void computeWrenches();
};

//! A contact that exists even when a hand is not perfectly touching another object
/*!	This class is meant for studing grasp quality when there is really no grasp, as
	the hand is not exactly touching an object. The high-level purpose is to convert
	the grasp quality function to a continous function that exists in all hand
	configurations, rather then a highly discrete function that only is non-zero
	when the hand is touching an object in the right way. However, this line of
	work is not completed and this class is in bad need of an overhaul.
	
	A virtual contact does NOT come in a pair - there is no mate. As such, any instance
	of this will have the mate set to NULL, a distinct sign of a virtual contact. 
	Also, virtual contacts have the normals pointing outward, instead of the usual
	contacts whose normals point inward. This is confusing and should probably be
	changed at some point.
	
	The VirtualContact does not always need a body2. In fact, most of its design can
	work if the body2 is NULL. However, it can have a body2, in which case some
	distance functions are computed based on the body2. This is not very well
	implemented right now and subject to change.
	
	A virtual contact can be instantiated from any kind of contact as it pretty much 
	only copies the friction edges and location. It also needs a separate 
	computeWrenches to eliminate the need for the object.
	
	All computations for a virtual contact (like wrenches, etc) are performed in the
	world coordinate system, unlike the usual contact which is in the object's 
	coordinate system. The reason is that we can have a grasp defined my many virtual
	contacts on the hand (rather then meny traditional contacts on an object). In this 
	case, each link of the hand has its own coordinate frame, so in order to have a 
	coherent frame for the grasp we use world coordinates.
*/
class VirtualContact : public Contact 
{
private:
	//! Meant to replace the object CoG when a virtual contact has no object
	position mCenter;
	//! Meant to replace the object max radius when a virtual contact has no object
	float mMaxRadius;
	
	//! We can hook this up directly to the world root to see the contact; for debug purposes
	SoSeparator *mWorldInd;
	//! We keep a pointer so we can change this contact's appearance on the fly; for debug
	SoMaterial *mZaxisMat;

	//! Which finger (kinematic chain) of the robot this virtual contact is on. -1 means the palm.
	int mFingerNum;
	//! Which link of the chain this virtual contact is on.
	int mLinkNum;

	//! Empty contact initialization shared by all constructors; sets the contact as its own mate
	void init();
public:
	
	//! The constructor initializes an empty virtual contact, setting it as its own mate
	VirtualContact();
	//! Also flips the normal of the original contact, to make it point outward
	VirtualContact(int f, int l, Contact *original);
	//! Copy constructor copies friction edges and location
	VirtualContact(const VirtualContact *original);
	//! Destructor
	~VirtualContact();

	//! Sets the body that this contact is on, presumably to a robot link
	void setBody(Body *b){body1 = b;}
	//! Sets an object that this contact is not exactly on, but we use in calculations.
	void setObject(Body *b){body2 = b;}

	//! Writes this contact, including friction edges, to a file
	void writeToFile(FILE *fp);
	//! Loads this contact, including friction edges, from a file
	void readFromFile(FILE *fp);

	//! Wrench computation is done in world coordinates considers the fact that we have no object
	void computeWrenches(bool useObjectData = false, bool simply = false);
	
	//! Scales the wrench space of this contact by a given factor
	void scaleWrenches(double factor);

	//! The virtual contact can not set up its own friction edges.
	int setUpFrictionEdges(bool dynamicsOn = false);

	//! Updates mObjectDistance and mObjectNormal based on current world locations.
	void updateContact(Body* object);

	//! Sets the max radius that is to be used in wrench computations
	void setRadius(float r){mMaxRadius = r;}
	//! Sets the c.o.g that is to be used in wrench computations
	void setCenter(position c){mCenter = c;}
	//! Gets the max radius that is used in wrench computations
	double getMaxRadius(){return mMaxRadius;}
	//! Gets the center that is used in wrench computations
	position getCenter(){return mCenter;}

	//! The visual indicator is just a thin red cylinder
	SoSeparator* getVisualIndicator();
	//! Another indicator that can be atatched directly to the world root, for debug
	void getWorldIndicator(bool useObjectData = false);

	//! The location of this contact in world coordinates
	position getWorldLocation();
	//! The normal of this contact (pointing outwards) in world coordinates
	vec3 getWorldNormal();
	//! Computes the distance to the object along the contact normal and the object surface normal at the closest point
	void getObjectDistanceAndNormal(Body *body, vec3 *objDistance, vec3 *objNormal);

	//! Changes the visual marker to show that this contact is marked; for debug purposes.
	void mark(bool m);

	//! Returns the number of the finger that this contact is on
	int getFingerNum(){return mFingerNum;}
	//! Returns the number of the link that this contact is on
	int getLinkNum(){return mLinkNum;}
};
/* just like what VirtualContact class has done, VirtualContactOnObject Class is nothing weird except that
it changes the virtual contacts' loaction from the finger to an object imported before.
with these virtual contacts, we continue to study the grasp quality. Hao 10.04
*/
class VirtualContactOnObject : public VirtualContact
{
public:
	VirtualContactOnObject ();
	~VirtualContactOnObject ();
	void readFromFile(FILE * fp);
#ifdef ARIZONA_PROJECT_ENABLED
	void readFromRawData(ArizonaRawExp* are, QString file, int index, bool flipNormal = false);
#endif
	void writeToFile(FILE * fp);
};
#define CONTACT_HXX
#endif
