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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie 
//
// $Id: body.h,v 1.46 2010/08/10 17:23:59 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the body class hierarchy.
 */
#ifndef BODY_HXX

#include <QTextStream>
#include <QString>
#include <list>
#include <vector>

#include "worldElement.h"
#include "contact.h"

#ifdef CGDB_ENABLED
#include "DBase/graspit_db_model.h"
#endif

class SoCoordinate3;
class SoGroup;
class SoIndexedFaceSet;
class SoMaterial;
class SoScale;
class SoSeparator;
class SoSwitch;
class SoTranslation;
class SoTransform;

class Contact;
class Robot;
class DynJoint;
class World;
class BoundingBox;
class Triangle;
class TiXmlElement;

//! A triangular body face used in computing mass properties.
/*!
  For use with Mirtich's mass property code
*/
typedef struct {
  double norm[3];
  double w;
  double *verts[3];
} FACE;

//! A generic simulation body.
/*! A generic body is defined by its geometry, its material, and a transform.
    Body instances are considered obstacles--they can form contacts with other
    objects but are not part of of the dynamics system.
*/
class Body : public WorldElement {
  Q_OBJECT
public:
  //! Parameter to control the height of friction cones
  static const float CONE_HEIGHT;

protected:
  //! The surface material of the body specified as an index to the world material list
  int material; 

  //! Tells us whether this is a rigid body or not (affects friction models)
  bool mIsElastic;
	
  //! The Young's Modulus of the material, it describes its elasticity
  double youngMod;

  //! The file that geometry was loaded from, if any
  QString mGeometryFilename;
	
  //! Type of geometry file, for now either "Inventor" or "off"
  QString mGeometryFileType;

  //! The body's world position (translations are in mm)
  transf Tran;  

  //! When this is un-checked, changing the transform of this body will not trigger a redraw
  bool mRenderGeometry;

  //! The number of contacts on the body
  int numContacts;

  //! The current contacts on the body
  std::list<Contact *> contactList;

  //! The contacts on the body at the previous time step
  std::list<Contact *> prevContactList;

  //! Virtual Contacts on this body. See the Virtual Contact class for explanation
  std::list<Contact *> virtualContactList;

  //! This flag determines whether the body's friction cones should be shown
  bool showFC;

  //! This flag determines whether the body's virtual contacts should be shown
  bool showVC;

  //! A pointer to the root node of the geometry of this model
  SoSeparator *IVGeomRoot;

  //! A pointer to a node that scales the geometry of this model
  SoTransform* IVScaleTran;
  
  //! A pointer to a node that offsets the geometry of this model
  SoTransform *IVOffsetTran;

  //! A pointer to a node that can hold the geometry of the bounding volume struture
  SoSeparator *IVBVRoot;

#ifdef GEOMETRY_LIB
  //! If we are using the geometry library, here we will show the primitives that approximate this body
  SoSeparator *IVPrimitiveRoot;
#endif

  //! A pointer to the Inventor transform for the body
  SoTransform *IVTran;  

  //! A pointer to the material node that controls this body's transparency
  SoMaterial *IVMat;

  //! A pointer to the root of the friction cones on this body
  SoSeparator *IVContactIndicators;

  //! This flag tells us if this body follows the Flock of Birds tracker
  bool mUsesFlock;

  //! This tells us which bird in the flock the object is using
  int mBirdNumber;

  //! The relative tranform used for the Flock of Birds
  FlockTransf mFlockTran;

  //! Inventor root of the axes in the body subtree
  SoSwitch *IVAxes;

  //! Inventor root of the worst case disturbance wrench indicator
  SoSeparator *IVWorstCase;

  //! Inventor transform from body frame to center of gravity
  SoTranslation *axesTranToCOG;

  //! Inventor scale for axes so that they extend outside the body
  SoScale *axesScale;

  //! Copy constructor
  Body (const Body &b);

  //! Creates the axes for display
  void createAxesGeometry();

  //! Initialize an empty scene graph structure with just the needed roots 
  void initializeIV();

  //! Adds itself to the given vector of Bodies
  virtual void getBodyList(std::vector<Body*> *bodies) {bodies->push_back(this);}

  /////////////////////////////// PUBLIC /////////////////////////////////
public:
  //! Empty body with invalid material and no geometry
  Body(World *w,const char *name=0);

  //! Breaks all contacts; does not remove body from collision detection or scene graph
  virtual ~Body();

  //! Load the body information from a file
  virtual int load(const QString &filename);
  
  //! Convert the body information to xml and link the xml to the given geometry file
  int convert2xml(QString filename);
  
  //! Loads the body information from an XML structure
  virtual int loadFromXml(const TiXmlElement *root, QString rootPath);

  //! Loads the geometry of the body from an Inventor file that can be read by Coin (IV or VRML)
  int loadGeometryIV(const QString &filename);

  //! Loads the geometry of the body from an .off file
  int loadGeometryOFF(const QString &filename);

  //! Loads the geometry of the body from a .ply file
  int loadGeometryPLY(const QString &filename);

  //! Loads the geometry from a vector of vertices and one of triangles
  int loadGeometryMemory(const std::vector<position> &vertices, const std::vector<int> &triangles);

 //! Saves the body information to an XML structure
 virtual int saveToXml(QTextStream& xml);

  //! Make this body a clone of another body
  virtual void cloneFrom(const Body* original);

  //! Adds additional material nodes so we can control the transparency of the body
  void addIVMat(bool clone = false);

  //! Adds this body to the collision detection system
  virtual void addToIvc(bool ExpectEmpty = false);

  //! Adds this body to the collision detection system as a clone of another body
  virtual void cloneToIvc(const Body* original);

  //! Changes the scale of the geometry. Collision geometry gets updated as well (might be slow)
  virtual void setGeometryScaling(double x, double y, double z);

  //! Changes the offset of the geometry. Collision geometry gets updates as well (might be slow)
  virtual void setGeometryOffset(transf tr);

  //! Sets the body to another location in the world. Collisions are not checked.
  virtual int setTran(transf const& newTr);

  //! Default parameters for transparency, show friction cones, etc.
  virtual void setDefaultViewingParameters();

  //! Enables / disabled automatic render requests when this body is moved
  void setRenderGeometry(bool s);

  //! Gets the current rendering requests state
  bool getRenderGeometry() const {return mRenderGeometry;}

  //! Tells us if this body's position is controlled by the Flock of Birds
  bool usesFlock(){return mUsesFlock;}

  //! Which bird in the flock controlls this body
  int getBirdNumber(){return mBirdNumber;}

  //! Where on the body the Flock of Birds sensor is mounted
  FlockTransf *getFlockTran(){return &mFlockTran;}

  //! Individual bodies belong to themselves.  Links override this and return their robot.
  virtual WorldElement *getOwner() {return (WorldElement *)this;}

  //! Determines whether instance is dynamic (overridden in DynamicBody)
  virtual bool isDynamic() const {return false;}

  //! Returns whether this body is soft (elastic) or not. Affect contact models.
  bool isElastic(){return mIsElastic;}

  /*! Returns the current material of the body. 
   * \sa setMaterial()
   */
  int getMaterial() const {return material;}
  //! Sets the material of this body
  void setMaterial(int mat);

  /*! Returns the Inventor material for the body. */
  SoMaterial *getIVMat() const {return IVMat;}

  //! Returns the Young's modulus for this body
  double getYoungs() { return youngMod; }

  /*! Returns a pointer to the root of the Inventor geometry that was loaded.*/
  SoSeparator *getIVGeomRoot() const {return IVGeomRoot;}

  /*! Returns a pointer ot the scaling transform of this body */
  SoTransform* getIVScaleTran() {return IVScaleTran;}

#ifdef GEOMETRY_LIB
  //! The root of the scene graph where we can put primitive geometry
  SoSeparator *getIVPrimitiveRoot() const {return IVPrimitiveRoot;}
#endif

  /*! Returns a pointer to the Inventor transform node for the body. */
  SoTransform *getIVTran() const {return IVTran;}

  /*! Returns a pointer to the root of the Inventor subtree containing the 
    friction cones.
  */
  SoSeparator *getIVContactIndicators() const {return IVContactIndicators;}

  /*! Asks the body to display a number of bounding boxes as part of its
	  collision detection bbox hierarchy. Used only for debugging purposes.
  */
  void setBVGeometry(const std::vector<BoundingBox> &bvs);

  /*! Returns the current pose of the body. */
  transf const& getTran() const {return Tran;}

  /*! Returns the number of contacts on the body, against another given body. */
  int getNumContacts(Body *b = NULL) const;

  /*! Returns the number of virtual contacts on the body. */
  int getNumVirtualContacts() const {return (int)virtualContactList.size();}

  /*! Returns a copy of the body's contact list against a given body. */
  std::list<Contact *>getContacts(Body *b = NULL) const;

  /*! Returns a copy of the body's virtual contact list. */
  std::list<Contact *>getVirtualContacts() const {return virtualContactList;}

  /*! Returns the value of the flag determining whether friction cones are
    shown on this body
  */
  bool frictionConesShown() const {return showFC;}

  //! Returns the transparency of this body for rendering
  float getTransparency() const;
  //! Sets the transparency of this body for rendering
  void setTransparency(float t);

  //! Shows or hides the friction cones for this body
  void showFrictionCones(bool on, int vc=0);
  //! Asks all contacts to recompute their friction cones
  virtual void redrawFrictionCones();

  //! Adds a contact to this body
  virtual void addContact(Contact *c);
  //! Adds a virtual contact to this body
  virtual void addVirtualContact(Contact *c);
  //! Removed a contact from this body
  virtual void removeContact(Contact *c);
  //! Removes a contact from the list of contacts at the previous time step
  virtual void removePrevContact(Contact *c);
  //! Removes all the cotacts on this body (usually after a move)
  virtual void breakContacts();
  //! Removes all virtual contacts on this body
  virtual void breakVirtualContacts();
  //! Load in virtual contacts specified in file fn
  virtual int loadContactData(QString fn);

  //! Moves all current contacts to the list of previous contacts
  virtual void resetContactList();
  //! Checks if a current contact is the same as a contact from the previous time step
  Contact* checkContactInheritance(Contact *c);

  //! Returns true if current contacts prevent motion in the given direction
  virtual bool contactsPreventMotion(const transf& motion) const;

  //! Prints the name and material of this body to a stream
  friend QTextStream& operator<<(QTextStream &os, const Body &b);

  /*! Returns all the triangles of the scene graph geometry of this object */
  void getGeometryTriangles(std::vector<Triangle> *triangles) const;

  /*! Returns all the vertices of the scene graph geometry of this object*/
  void getGeometryVertices(std::vector<position> *vertices) const;
};

//! The superclass for all bodies that take part in the dynamics.
/*! A dynamic body adds mass parameters to the generic body description.
    It also includes the state variables q and v, which encode the position
    and velocity of the body at the current time step of the dynamics.
*/
class DynamicBody : public Body {
  Q_OBJECT

  //! Is this body affected by dynamics?
  bool useDynamics;

  //! projection integrals used in mass properties computations
  double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

  //! face integrals used in mass properties computations
  double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

 protected:
  //! Center of gravity position (in millimeters)
  position CoG;

  //! Maximum radius of the body from the center of gravity (in millimeters)
  double maxRadius;

  //! Mass of the body (in grams)
  double mass;

  //! Is this body fixed in the world frame?
  bool fixed;

  //! World pose a fixed body should maintain
  transf fixedTran;

  //! Points to the dynamic joint connecting this body to its parent
  DynJoint *dynJoint;

  //! Corners of the box bounding the possible world positions of the object
  vec3 bbox_max, bbox_min;

  //! Flag determining whether axes should be shown (origin at COG)
  bool showAx;

  //! Flag determining whether dynamic contact forces for this body should be drawn
  bool showDynCF;

  //! Dynamic acceleration, in mm/sec^2
  double a[6];
  //! Dynamic velocity, in mm/sec
  double v[6];
  //! Dynamic pose (translation in mm and quaternion)
  double q[7];
  //! Stack for saved velocity states
  std::list<double*> vStack;
  //! Stack for saved position states
  std::list<double*> qStack;
  //! A single saved velocity state
  double markedV[6];
  //! A single saved position state
  double markedQ[7];

  //! The inertia tensor of this body; remember to multiply by mass to get the actual inertia
  double I[9];
  
  //! Tells us if the motion for this body has been computed at the current time step
  bool dynamicsComputedFlag;

  //! Accumulates external wrenches in world coordinates applied to this body
  double extWrenchAcc[6];

  //! Used for computing mass properties if not supplied in file
  void compProjectionIntegrals(FACE &f,int A,int B);

  //! Used for computing mass properties if not supplied in file
  void compFaceIntegrals(FACE &f,int A,int B,int C);

  //! Uses Brian Mirtich's code to compute cog and inertia matrix based on geometry
  int computeDefaultInertiaMatrix(std::vector<Triangle> &triangles, double *defaultI);

  //! Initializes an empty dynamic body, common to all constructors
  void init();

public:
  //! An empty dynamic body with no meaningful properties
  DynamicBody(World *w, const char *name=0);

  //! Creates a dynamic body from a static one
  DynamicBody(const Body &b, double m);

  virtual ~DynamicBody();

  //! Initializes dynamic parameters after copy constructor or load method
  void resetDynamics();

  //! Clones another dynamic body
  void cloneFrom(const DynamicBody *newBody);

  //! Also looks for properties specific to the dynamic body
  virtual int loadFromXml(const TiXmlElement *root, QString rootPath);

  //! Saves the DynamicBody information to an XML structure
  virtual int saveToXml(QTextStream& xml);

  //! Also updates the dynamic state of the body.
  virtual int setTran(transf const& newTr);

  //! Sets the center of gravity and reinitializes the state vector to match it
  void setCoG(const position& newCoG);
  
  //! Sets the inertia matrix from \a newI
  void setInertiaMatrix(const double *newI);

  //! Computes and sets the default center of gravity and inertia matrix
  void setDefaultDynamicParameters();

  //! Sets the maximum radius of the object, used for scaling and grasp computations
  void setMaxRadius(double maxRad);

  //! Attempts to compute mass and inertia matrix based only on geometry
  void computeDefaultMassProp(position &cog, double *I);

  //! Attempts to compute the max radius based only on geometry
  double computeDefaultMaxRadius();

  //! Hides the axes by default
  virtual void setDefaultViewingParameters();

  /*! Returns the center of gravity position. */
  position const& getCoG() const {return CoG;}

  /*! Returns the max radius of the body. */
  double getMaxRadius() const {return maxRadius;}

  /*! Returns the mass of the body (in grams).
   * \sa setMass()
   */
  double getMass() const {return mass;}

  //! Returns a pointer to the body's 3x3 inertia tensor (stored as an array).
  const double *getInertia() const {return I;} 

  /*! Returns a pointer to the body's 6x1 velocity vector. */
  const double *getVelocity() {return v;}

  /*! Returns a pointer to the body's 6x1 acceleration vector (simple first
    order approximation).
   */
  const double *getAccel() {return a;}

  /*! Returns a pointer to the body's 7x1 position vector [3x1 translation , 
     4x1 quaternion]
   */
  const double *getPos() {return q;} 

  /*! The returns a pointer to the Inventor subgraph containing the worst case
    indicator */
  SoSeparator *getIVWorstCase() const {return IVWorstCase;}

  /*! Returns whether the body is fixed within the world */
  bool isFixed() const {return fixed;}

  /*! Returns whether this body is affected by dynamics
    \sa setUseDynamics()
   */
  bool isDynamic() const {return useDynamics;}

  /*! Returns the world pose a fixed body should maintain */
  const transf& getFixedTran() const {return fixedTran;}

  /*! Returns the dynamic joint connecting this body to its parent or NULL
   * if there is none
   */
  virtual DynJoint *getDynJoint() {return dynJoint;}

  /*! Returns whether axes are drawn for this body */
  bool axesShown() const {return showAx;}

  /*! Returns whether dynamic contact forces should be drawn for this body */
  bool dynContactForcesShown() const {return showDynCF;}

  /*! Sets whether this body should be affected by dynamics.  Rather than
      creating a new static body when a body is set to be static, we just set
      this flag to false.  That way if the body is made dynamic again, many\
      parameters won't have to be recomputed.
  */
  void setUseDynamics(bool dyn) {useDynamics = dyn;}

  /*! Returns whether the dynamics have been computed for this body during
    the current dynamics iteration.
    \sa setDynamicsFlag()
    \sa resetDynamicsFlag()
  */
  bool dynamicsComputed() const {return dynamicsComputedFlag;}  

  /*! At the end of the dynamics iteration, this is used to reset the
    dynamicsComputed flag
   */
  void resetDynamicsFlag() {dynamicsComputedFlag = false;}
  
  /*! This is called to set the dynamicsComputed flag after the motions for
    this body have been computed during the current iteration of the dynamics.
   */
  void setDynamicsFlag() {dynamicsComputedFlag = true;}

  /*! Sets whether axes should be drawn for this body */
  void showAxes(bool on);

  /*! Sets whether dynamic contact forces should be drawn for this body */
  void showDynContactForces(bool on);

  /*! Sets the current 7x1 position vector */
  bool setPos(const double *new_q);

  /*! Sets the current 6x1 velocity vector [vx vy vz vrx vry vrz] */
  void setVelocity(double *new_v)  {memcpy(v,new_v,6*sizeof(double));}

  /*! Sets the current 6x1 acceleration vector */
  void setAccel(double *new_a)  {memcpy(a,new_a,6*sizeof(double));}

  /*! Sets the world boundaries for this body.  This is so a body can't fall forever.*/
  void setBounds(vec3 minBounds,vec3 maxBounds)
    {bbox_min = minBounds; bbox_max = maxBounds;}
  
  /*! Sets the body's mass in grams to m.
    \sa getMass()
   */
  void setMass(double m) {mass = m;}

  /*! Returns the value of the external wrench accumulator. */
  double *getExtWrenchAcc() {return extWrenchAcc;}

  //! Saves the current state on the stack
  void pushState();
  //! Pops and sets the current state from the stack
  bool popState();
  //! Clears the state stack
  void clearState();
  //! Saves the current state in a one-slot-only location
  void markState();
  //! Returns to previously saved state
  void returnToMarkedState();

  //! Clears the external wrench accumulator
  void resetExtWrenchAcc();
  //! Adds an external wrench expressed in world coordinates
  void addExtWrench(double *extW);
  //! Adds a pure force expressed in world coordinates
  void addForce(vec3 force);
  //! Adds an external torque expressed in world coordinates
  void addTorque(vec3 torque);
  //! Adds a torque expressed in body coordinates
  void addRelTorque(vec3 torque);
  //! Adds the wrench resulting from a force applied at some location on the object in world coords
  void addForceAtPos(vec3 force,position pos);
  //! Adds the wrench resulting from a force applied at some location on the object in body coords
  void addForceAtRelPos(vec3 force,position pos);
  
  //! Makes this body fixed for dynamic simulations
  void fix();
  //! Allows this body to move in dynamic simulations
  void unfix();
  //! Connects a dynamic joint (usually a fixed one) to this body
  virtual void setDynJoint(DynJoint *dj);

  /*! Holds a default mass in case a body file doesn't specify one.
   *  The other mass properties can be computed assuming uniform density
   */
  static double defaultMass;
};

//! Used for bodies that are part of a robot.
/*! A link is a dynamic body that notifies its owner robot when contacts have
    changed.
*/
class Link : public DynamicBody {
  Q_OBJECT

  friend class Robot;
  friend class KinematicChain;

 protected:

  //! A pointer to the robot that this link is a part of
  Robot *owner;

  //! Identifies what part of the robot this link is
  int chainNum,linkNum;

 public:
  Link(Robot *r,int c, int l,World *w,const char *name=0);
  virtual ~Link();

  /*! Returns a pointer to the robot owning this link. */
  virtual WorldElement *getOwner() {return (WorldElement *)owner;}

  /*! Returns which chain this link is a part of. */
  int getChainNum() const {return chainNum;}

  /*! Returns which link in the chain this link is. */
  int getLinkNum() const {return linkNum;}
  
  /*! Check if contact against a body not belonging to the same robot prevents motion.*/
  bool externalContactsPreventMotion(const transf& motion);

  //! Sets the flag that indicates that contacts have changed
  virtual void setContactsChanged();

  /*! Returns the location of the distal joint in this link's coordinate system */
  position getDistalJointLocation();

  /*! Returns the location of the proximal joint in this link's coordinate system */
  position getProximalJointLocation();

  /*! Returns the z axis of the proximal joint in link's coordinate system */
  vec3 getProximalJointAxis();
};



#ifdef CGDB_ENABLED
class GraspitDBModel;
#endif

//! Used for dynamic bodies that are not part of a robot.
/*! A Graspable body is partially transparent by default and shows the
    locations of any contacts on its surface.
*/
class GraspableBody : public DynamicBody {
  Q_OBJECT

  //! A pointer to the Inventor root of the shape primitives for this body
  SoSeparator *IVGeomPrimitives;

  //! When using the CGDB, here we store the CGDB model that this body comes from, if any
#ifdef CGDB_ENABLED
  GraspitDBModel* mGraspitDBModel;
#endif

 public:
  GraspableBody(World *w, const char *name=0);
  virtual ~GraspableBody();

  virtual void setDefaultViewingParameters();

  virtual void cloneFrom(const GraspableBody *newBody);

  /*! If the body has shape primitives defined (i.e. they were found when the
    body was loaded), this returns a pointer to the Inventor root node of
    that tree.  Otherwise, it returns NULL. */
  SoSeparator *getIVGeomPrimitives() const {return IVGeomPrimitives;}

  friend QTextStream& operator<<(QTextStream &os, const GraspableBody &gb);

#ifdef CGDB_ENABLED
  //! When using the CGDB, returns the database model that this body comes from, if any
  GraspitDBModel *getDBModel(){
    return mGraspitDBModel;
  }
  //! When using the CGDB, sets the database model that this body comes from, if any
  void setDBModel(GraspitDBModel* model) {mGraspitDBModel = model;}
#endif

};


#define BODY_HXX
#endif
