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
// $Id: robot.h,v 1.55 2010/07/09 02:33:45 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the robot class hierarchy.
 */

#ifndef ROBOT_H

#include <vector>
#include <map>
#include <QTextStream>
#include <QString>

#include "collisionStructures.h"
#include "mytools.h"
#include "kinematicChain.h"
#include "dof.h"
#include "worldElement.h"
#include "body.h"
#include "joint.h"

class Grasp;
class GloveInterface;
class CyberGlove;
class EigenGraspInterface;
class Matrix;
class TiXmlElement;

//! Base class for all robots which are collections of links connected by moveable joints
/*! A robot is collection of link bodies orgainized around a base link.
   Connected to the base link may be 1 or more kinematic chains of links and
   joints.  When a robot is attached to another robot an additional mount
   piece that connects to the base of the child robot may also be defined.
   The structure of a robot is defined in a text configuration file
   [see user documentation] that is parsed by the load method. 
   
   Each robot has a collection of DOF's (degrees of freedom) that are
   linked to the individual joints of a kinematic chain.  In general, a robot
   can not set its joints directly. All joint motion has to be obtained from
   the DOF's then set using the kinematic chains. In general, we will refer to
   the position of the robot's base in world coordinate systems as the robot's
   "pose" while the value of the joints (and implicitly the DOF's) is the robot's
   "posture".
   
   To set the robot pose, think of it as a general \a WorldElement; nothing 
   complicated here. Posture is more involved, and depends on whether you are
   in static or dynamic mode.
   
   In static mode, you can have the robot move to some desired *DOF* values. The
   DOF's will then compute appropriate joint values, and the robot will have the
   chains execute those. The settings come in two flavors:
   
   \a moveDOFToContacts(...) will perform the move in small steps and detect 
   contacts along the way. It is guaranteed to leave your robot in a legal 
   configuration, and detect all contacts that occur.
   
   \a forceDOFVals(...) will force the final joint values and perform no collision
   detection. Therefore, it might leave the robot in inter-penetration with some 
   other object.
   
   If dynamics are on, then new desired DOF positions are set with a call
   to \a setDesiredDOFVals, and a number of intermediate set points are generated
   for each joint that will ensure a smooth velocity and acceleration profile.
   Built in PD DOF controllers use gains specified in the robot configuration
   file to apply approriate forces to each joint to correct error between the
   current joint position and the current set point.
   
   If a linear cartesian moves are desired, a trajectory generator can smoothly
   interpolate between the desired poses by creating a number of intermediate
   poses.  The inverse kinematics for each of these positions is computed and
   and the resulting joint positions are used as set points for the PD
   controllers.   
*/
class Robot : public WorldElement {
  Q_OBJECT

signals:
  //! This signal informs that dof values have changed
  void configurationChanged();

  //! This signal informs that user has begun interaction by clicking on a robot handler
  void userInteractionStart();

  //! This signal is the complement of userInteractionStart().
  void userInteractionEnd();

  //! Emitted by moveDOFToCOntacts each time a step is taken
  void moveDOFStepTaken(int numCols, bool &stopRequest);

private:
  // Connection to a parent robot 
  //! Points to a parent robot that this robot is attached to
  Robot *parent;
  //! Records which chain of the parent robot this robot is attached to
  int parentChainNum;
  //! The inverse of the offset transform from parent chain end to this robot's base
  transf tranToParentEnd;
  //! A pointer to an optional mount piece link
  Link *mountPiece;

protected:

  // The structure of the robot itself	
  //! The number of kinematic chains (or fingers) this robot has
  int numChains;
  //! The number of degrees of freedom this robot has
  int numDOF;
  //! The total number of joints of this robot. Set when robot is loaded
  int numJoints;
  //! A vector of pointers to this robot's kinematic chains
  std::vector<KinematicChain *> chainVec;
  //! A vector of pointers to this robot's DOF's
  std::vector<DOF *> dofVec;
  //! A pointer to the base link (or palm) of the robot
  Link *base;

  // Save and restore state
  //! Is used to save the current transform if we want to restore it later
  transf savedTran;
  //! Is used to store the states of the DOF; a stream is convenient as it can pack all dofs
  QString savedDOF;
  //! Tells us whether the state has been previously saved or not
  bool savedState;

  //! Whether changes to the position / geometry of the robot should trigger a scene graph redraw
  bool mRenderGeometry;

  // Dynamic simulation: parameters and trajectory generation
  //! The simulation time of when the next change of DOF setpoints should occur
  double dofUpdateTime;
  //! The default translational velocity (mm/sec) to use when generating cartesian trajectories
  double defaultTranslVel;
  //! The default rotational velocity (rad/sec) to use when generating cartesian trajectories
  double defaultRotVel;

  // Input from external hardware
  //! Shows if this robot is to be controlled via a CyberGlove
  bool mUseCyberGlove;
  //! Provides translation between the CyberGlove and this robot's DOFs
  GloveInterface *mGloveInterface;
  //! Shows if this robot is to be controlled by the Flock of Birds
  bool mUsesFlock;
  //! If robot is controlled by Flock of Birds, shows which bird it is attached to
  int mBirdNumber;
  //! Geometry for a visual model that shows where the bird is attached to the robot
  SoSeparator *IVFlockRoot;
  //! The relative tranform used for the Flock of Birds
  FlockTransf mFlockTran;

  //!Holds all information about this robot's eigengrasps
  EigenGraspInterface* mEigenGrasps;

  //! Pre-specified information on how to best approach an object for grasping
  /*! This transform sets the origin and the z axis to match the best approach 
	  direction for this hand */
  transf approachTran;
  //! Geometry for the visual model of the approach direction
  SoSeparator *IVApproachRoot;

  //! Adds a visual image of the Flock of Birds sensor to the base of the robot
  void addFlockSensorGeometry();
  //! Adds a visual indicator of the pre-specified approach direction for this hand
  void addApproachGeometry();

  //! Recurses to all chains of this robot and any attached robots
  virtual bool simpleContactsPreventMotion(const transf& motion) const;

  //! An internal method called by setTran.  
  inline virtual void simpleSetTran(transf const& tr);

  //! Asks all chains to set the given joint values
  virtual void setJointValues(const double* jointVals);
  //! Asks all chains to set the given joint values, then update the position of all links
  virtual void setJointValuesAndUpdate(const double* jointVals);
  //! Gets the current joint values from the chains
  inline void getJointValues(double* jointVals) const;
  //! Informs the dof's that certain values have been set.
  inline void updateDofVals(double *dofVals);
  //! Main function for obtaining joint values from the dofs given desired dof values
  virtual bool getJointValuesFromDOF(const double *desireddofVals, double *actualDofVals,
									 double *jointVals, int *stoppedJoints);
  //! Gets the Jacobian matrix of Joints w.r.t. DOF
  Matrix getJacobianJointToDOF(int chainNum);

  //! Attempts to set the desired dof values, detecting contacts along the way
  bool jumpDOFToContact(double *desiredVals, int *stoppedJoints, int *numCols = NULL);
  //! Finds the contact time between a collision and a collision-free posture
  int interpolateJoints(double *initialVals, double *finalVals, CollisionReport *colReport, 
						double *interpolationTime);
  //! Stops all the joints that affect a link that is in contact
  void stopJointsFromLink(Link *link, double *desiredJointVals, int *stoppedJoints);

  //! Adds all of the bodies that make up this robot to the given vector
  virtual void getBodyList(std::vector<Body*> *bodies);

  friend void KinematicChain::updateLinkPoses();
  friend void KinematicChain::attachRobot(Robot *r,const transf &offsetTr);

 public:

  /*! Simply initializes an empty robot within world w.  The load method must be called to read a 
	  configuration file and give structure to the robot. */
  Robot(World *w,const char *name) : WorldElement(w,name) {
    parent=NULL; parentChainNum = -1;  mountPiece=NULL;
    numChains = numDOF = 0; base = NULL; dofUpdateTime=0.0;
	mUseCyberGlove = false; mGloveInterface = NULL; mUsesFlock = false;
	mEigenGrasps = NULL; mRenderGeometry = true; savedState = false;
	approachTran = transf::IDENTITY;
	// temporary
	defaultTranslVel = 50; defaultRotVel = M_PI/4.0;
  }
  
  //! Deletes all kinematic chains, the base and mount piece, etc.
  virtual ~Robot();

  //--------------------------load and populate a robot-------------------------------

  //! The main load function that loads all the information from XML
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

  //! Makes this robot into a clone of the original
  virtual void cloneFrom(Robot *original);

  //--------------------------statics-------------------------------------------------

  //! The main way to move robot dofs IN STATICS. Checks collisions and finds contacts.
  bool moveDOFToContacts(double *desiredVals, double *desiredSteps, bool stopAtContact, 
						 bool renderIt=false);

  //! Sets the location (pose) of the base of this robot in the world coordinate system
  virtual int setTran(transf const& tr);

  //! Sets the given DOF to the given value and updates the link poses. Collisions are NOT checked.
  inline void forceDOFVal(int dofNum,double val);

  //! Sets the values of the DOF's to the values in the array dofVals. Collisions are NOT checked.
  inline void forceDOFVals(double *dofVals);

  //-------------------------dynamics-------------------------------------------------

  //! The main way to move the robot dofs IN DYNAMICS mode. 
  void setDesiredDOFVals(double *dofVals);

  //! Returns true if any of the contacts on the fingers are slipping during dynamics
  bool contactSlip();

  //! Attempt to check if the dynamic autograsp is complete
  bool dynamicAutograspComplete();

  //! Computes the joint angles after a dynamic step has been completed
  virtual void updateJointValuesFromDynamics();

  //! Applies internal joint forces (if any), such as friction or joint springs
  void applyJointPassiveInternalWrenches();

  //! Calls the DOF controllers which set dof forces based on current and desired posture
  virtual void DOFController(double timeStep);

  //! Builds dynamic constraints for all the coupled dof's of this robot
  virtual void buildDOFCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
										   double* Nu,double *eps,int &ncn);

  //! Builds dynamic dof limit constraints for all the dof's of this robot.
  virtual void buildDOFLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
									    double* H, double *g, int &hcn);

  //-------------------------get position and pose information------------------------

  /*! Return the transform which describes the world pose of the robot base
      frame with respect to the world coordinate system. */
  transf const &getTran() const {return base->getTran();}

  //! Computes the transform of all links in a chain for a given set of dof vals
  virtual void fwdKinematics(double *dofVals,std::vector<transf>& trVec,int chainNum);

  //! Computes the dofvals that achieve a desired end pose for a chain.
  virtual int invKinematics(const transf& endTran,double* dofVals, int chainNum); 

  //! Returns the current values of all of the DOF's of the robot.
  inline void getDOFVals(double *dofVals) const;

  //! Returns the values of the DOFs that can be used to save the current state
  inline void storeDOFVals(double *dofVals) const;

  //------------------------- static checks and range of motion

  //! Returns true if all specified dof vals are within their respective legal ranges
  inline bool checkDOFVals(double *dofVals) const;

  //! Clamps the given dofVals to be within their respective legal ranges
  inline bool checkSetDOFVals(double *dofVals) const;

  //! Returns true of contacts on the palm, a link or a parent robot prevent given motion
  virtual bool contactsPreventMotion(const transf& motion) const;

  //! Checks if the path between current and a desired posture is free of collisions
  bool checkDOFPath(double *desiredVals, double desiredStep);

  //! Sets the DOF values based on the current values of the joints
  inline void updateDOFFromJoints(double *jointVals);

  //! Returns the closest distance between a joint value and its limit
  inline double jointLimitDist() const;

  //-------------------------connections to real input devices (CyberGlove, FLock etc.)

  //! Returns true if this robot is controlled by a CyberGlove
  bool useCyberGlove(){return mUseCyberGlove;}

  //! Sets the instance of the class which translates CyberGlove informration for this robot
  void setGlove(CyberGlove *glove);

  //! Processes a new reading from the CyberGlove
  void processCyberGlove();

  //! Returns the class that interfaces between the CyberGlove and this robot
  GloveInterface* getGloveInterface(){return mGloveInterface;}

  //! Returns the number of the bird that this robot is connected to
  int getBirdNumber(){return mBirdNumber;}

  //! Returns true if this robot is connected to a Flock of Birds
  bool usesFlock(){return mUsesFlock;}

  //! Returns the transform that indicates where the flock sensor is mounted on this robot
  const FlockTransf* getFlockTran() const {return &mFlockTran;}

  //! Returns the class that keeps track of flock transforms for this robot
  FlockTransf* getFlockTran(){return &mFlockTran;}

  //-------------------------eigengrasps----------------------------------------------

  //! Looks for and loads eigengrasp information from a given file
  int loadEigenData(QString filename);

  //! Sets the trivial eigengrasp set, where we have 1 eg per dof
  int useIdentityEigenData();

  //! Returns the eigengrasps of this robot
  const EigenGraspInterface* getEigenGrasps() const {return mEigenGrasps;}

  //! Returns the eigengrasps of this robot
  EigenGraspInterface* getEigenGrasps() {return mEigenGrasps;}

  //-------------------------save and load the state----------------------------------

  //! Reads the values of all dofs from a text stream, then updates posture accordingly
  virtual QTextStream &readDOFVals(QTextStream &is);

  //! Writes the values of all dofs to a text stream
  virtual QTextStream &writeDOFVals(QTextStream &os);

  //! Saves the state of the robot (pose and posture). Overwrites any previously saved state.
  virtual void saveState();

  //! Restores the previously saved state (if any).
  virtual void restoreState();

  //-------------------------contacts-------------------------------------------------

  //! Returns the total number of contacts on this robot against a given body
  int getNumContacts(Body* body = NULL);

  //! Returns all the contacts on this robot against a given body
  std::list<Contact*> getContacts(Body* body = NULL);

  //! Breaks all the contacts on this robot
  void breakContacts();

  //! Returns the total number of virtual contacts on this robot
  int getNumVirtualContacts();

  //! Shows or hides virtual contacts
  void showVirtualContacts(bool on);

  //! Looks for and loads virtual contact data from a given file
  int loadContactData(QString filename);

  //-------------------------attached robots------------------------------------------

  //! Given a chain or tree of connected robots, returns a pointer to the root or base robot.
  Robot *getBaseRobot() {if (parent) return parent->getBaseRobot(); return this;}

  //! Returns a pointer to the parent robot (whose chain this robot is connected to).
  Robot *getParent() {return parent;}

  //! Returns the index of the chain that this robot is connected to.
  int getParentChainNum() {return parentChainNum;}

  /*! Returns the transform from the base of this robot to end frame of the
      parent's kinematic chain that this robot is connected to. */
  const transf &getTranToParentEnd() {return tranToParentEnd;}

  //! Returns a vector of all robots attached to this one
  void getAllAttachedRobots(std::vector<Robot *> &robotVec);

  //! Attaches another robot to the end of a chain of this robot
  void attachRobot(Robot *r,int chainNum,const transf &offsetTr);

  //! Detaches a previously attached robot
  void detachRobot(Robot *r);

  //! Loads a mount piece from a file, if this robot is atatched to another
  Link *importMountPiece(QString filename);

  //! Returns a point to the mountpiece link (NULL if none exists)
  Link *getMountPiece() const {return mountPiece;}

  //-------------------------other functions and accessors----------------------------

  bool snapChainToContacts(int chainNum, CollisionReport colReport);

//! Sets the name of this robot
  void setName(QString newName);

  //! Return the number of links for this robot (including palm and mount piece).
  int getNumLinks() const;

  //! Returns the number of degrees of freedom for this robot. 
  int getNumDOF() const {return numDOF;}

  //! Returns a pointer to the i-th DOF. 
  DOF *getDOF(int i) const {return dofVec[i];}

  //! Returns the scale of the i-th DOF dragger.
  float getDOFDraggerScale(int i) const {return dofVec[i]->getDraggerScale();}

  //! Returns the number of kinematic chains in this robot.
  int getNumChains() const {return numChains;}

  //! Returns the number of joints in this robot, as set when robot was loaded
  int getNumJoints() const {return numJoints;}

  //! Returns a pointer to the cnumber of kinematic chains in this robot
  KinematicChain *getChain(int i) const {return chainVec[i];}

  //! Return a pointer to the base link of this robot
  Link *getBase() const {return base;}

  //! Sets the transparency of the entire robot to the given value
  void setTransparency(float t);

   //! Returns the default translational velocity (mm/sec) for the robot.
  double getDefaultTranslVel() const {return defaultTranslVel;}

  //! Returns the default rotational velocity (rad/sec) for the robot.
  double getDefaultRotVel() const  {return defaultRotVel;}

  //! Sets the default translational velocity (mm/sec) for the robot.
  void setDefaultTranslVel(double v) {defaultTranslVel = v;}

  //! Sets the default rotational velocity (rad/sec) for the robot.
  void setDefaultRotVel(double v) {defaultRotVel = v;}

  //! Enables or disables the automatic render when the pose or porture of the robot are changed
  void setRenderGeometry(bool s);

  //! Returns whether the automatic render flag is set
  bool getRenderGeometry() const {return mRenderGeometry;}

  //! Returns a vector of all links associated with this robot
  void getAllLinks(std::vector<DynamicBody *> &allLinkVec);

  //! Computes a smooth trajectory so that a given chain goes through a set of poses
  void setChainEndTrajectory(std::vector<transf>& traj,int chainNum);

  //! Computes a trajectory that interpolates linearly between a start and end poses
  void generateCartesianTrajectory(const transf &startTr, const transf &endTr, 
								   std::vector<transf> &traj,
								   double startVel, double endVel=0.0, double timeNeeded=-1.0);

  //! Accumulated the total static torques applied at each joint by all dof's
  Matrix staticJointTorques(bool useDynamicDofForce);
	
  //! Returns the pre-defined approach direction for this robot
  transf getApproachTran() const {return approachTran;}

  //! Tells us how far along the approach direction a given object is, within a certain limit
  double getApproachDistance(Body *object, double maxDist);

  //---------------------------emit signals-----------------------------------------

  //! Emits the configuration changed signal
  void emitConfigChange(){emit configurationChanged();}
  //! Emits the user interaction start signal
  void emitUserInteractionStart(){emit userInteractionStart();}
  //! Emits the user interaction ended signal
  void emitUserInteractionEnd(){emit userInteractionEnd();}

  static const double AUTO_GRASP_TIME_STEP;
};

/*! Informs the dofs of their new values in \a dofVals. Simply passes through
	to the similar function of each dof
*/
void Robot::updateDofVals(double *dofVals)
{
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->updateVal(dofVals[d]);
	}
}

/*! Gets the current joint values in \a jointVals. It is expected that
	this vector is of size equal to the number of joints of this robot.
*/
void Robot::getJointValues(double *jointVals) const
{
	for (int c=0; c<numChains; c++) {
		chainVec[c]->getJointValues(jointVals);
	}
}

/*! Returns the closest distance between a joint value and its limit; positive 
	if the  joint is inside its legal range and negative if it's outside
*/
double Robot::jointLimitDist() const
{
	double dist, minDist = 1.0e10;
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<chainVec[c]->getNumJoints(); j++) {
			double val = chainVec[c]->getJoint(j)->getVal();
			dist = std::min( chainVec[c]->getJoint(j)->getMax() - val, 
							 val - chainVec[c]->getJoint(j)->getMin() );
			minDist = std::min(dist, minDist);
		}
	}
	return minDist;
}

/*! Sets the transform of the base link, the mount piece (if it exists), and tells each
	kinematic chain to update the link poses, which also updates the pose of each 
	attached robot. 
*/
void Robot::simpleSetTran(transf const& tr) {
    base->setTran(tr);
    if (mountPiece) mountPiece->setTran(tr);
    for (int f=0;f<numChains;f++) chainVec[f]->updateLinkPoses();
}

/*! \a dofVals should point to a double array with a length at least equal to
    the number of DOF's of this robot.
*/
void Robot::getDOFVals(double *dofVals) const {
    for (int d=0;d<numDOF;d++) dofVals[d]=dofVec[d]->getVal();
}

/*! The values of the DOFs that can be used for saving the current state
	are not always the current values of the DOF's. This in not ideal, but
	was necessary in the case of tha Barrett hand: when saving the current
	state, more often we want the *breakaway* value of the DOF rather than
	its current value. 

	Neither solution is ideal:
	- if we later restore the state using the current value of the DOF, we
	have lost breakaway information and that might result in a collision
	- if we do it using the breakaway value, we will lose some contacts on
	the distal link. However, these can usually be recovered using an 
	autoGrasp(...).

	The correct way is to use writeDOFVals(...) and readDOFVals(...) which
	save the entire state of the DOFs, including all breakaway information.
	However, those functions work with QStrings, while we needed to save 
	the state as one value per DOF for external reasons. Hence the need for
	this hack-ish function.
*/
void Robot::storeDOFVals(double *dofVals) const {
    for (int d=0;d<numDOF;d++) dofVals[d]=dofVec[d]->getSaveVal();
}

/*! Forces a single dof to assume a current value. Gets the appropriate joint
	values from the dofs, then forces the joints to that position and updates
	the posture. Does NOT check for collisions
*/
void Robot::forceDOFVal(int dofNum,double val) {
	double *jointVals = new double[numJoints];
	getJointValues(jointVals);
	dofVec[dofNum]->reset();
	dofVec[dofNum]->accumulateMove(val, jointVals, NULL);
	setJointValuesAndUpdate(jointVals);
	dofVec[dofNum]->updateVal(val);
	delete [] jointVals;
}

/*! Sets the values of the dofs of this robot to the values in the array 
	\a dofVals. Then it updates the link poses, but collisions are NOT checked.
*/
void Robot::forceDOFVals(double *dofVals) {
	double *jointVals = new double[numJoints];
	getJointValues(jointVals);
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->reset();
		dofVec[d]->accumulateMove(dofVals[d], jointVals, NULL);
	}
	setJointValuesAndUpdate(jointVals);
	updateDofVals(dofVals);
	delete [] jointVals;
}

/*! Asks each dof to update its value based on the joint values supplied in
	\a jointVals 
*/
void Robot::updateDOFFromJoints(double *jointVals) {
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->updateFromJointValues(jointVals);
	}
}

/*! Returns true if all the dof values in \a dofVals are within their limits */
bool Robot::checkDOFVals(double *dofVals) const {
	  for (int d=0;d<numDOF;d++) {
		  if ( dofVals[d] > dofVec[d]->getMax()) return false;
		  if ( dofVals[d] < dofVec[d]->getMin()) return false;
	  }
	  return true;
}

/*! Clamps the dof values in \a dofVals to be within the dof limits. Returns true
	if at least one the values in \a dofVals was within its limit before clamping
*/
bool Robot::checkSetDOFVals(double *dofVals) const {
	  bool atLeastOneValid = false;
	  for (int d=0;d<numDOF;d++) {
		  if ( dofVals[d] > dofVec[d]->getMax()) {
			  dofVals[d] = dofVec[d]->getMax();
		  } else if ( dofVals[d] < dofVec[d]->getMin()) {
			  dofVals[d] = dofVec[d]->getMin();
		  }
		  else atLeastOneValid = true;
	  }
	  return atLeastOneValid;
}


//! A hand is a special type of robot that can have a grasp associated with it.
/*! A hand is a special type of a more generic robot.  Generally it consists of
    a palm as the base link and one or more fingers, which are kinematic
    chains.  One instance of a grasp is associated with each hand, and it is
    usually analyzed each time contacts between the hand and the object to be
    grasped change.  The hand class also provides an autograsp method which 
    closes the fingers at fixed rates until further motion is prevented by
    contacts.

	This class is more the results of legacy architecture than any conceptual 
	differences. Arguably, any robot is also a "hand", especially since we allow
	any robot to have multiple kinematic chains. It is possible that we will get rid
	of the Hand class altogether at some point.
*/
class Hand : public Robot {
  Q_OBJECT

protected:
  //! A pointer to the grasp associated with this hand
  Grasp *grasp;

public:
  //! Constructs the instance of the Grasp, then calls the Robot constructor
  Hand(World *w,const char *name);
  //! Also deletes the instance of the Grasp
  virtual ~Hand();

  //! Clones this hand from another
  virtual void cloneFrom(Hand *original);

  /*! Returns the number of fingers in the hand.  Fingers are just another name
    for kinematic chains, so this is just a convenience function */
  int getNumFingers() const {return numChains;}

  /*! Returns a pointer to the i-th kinematic chain. */
  KinematicChain *getFinger(int i) const {return chainVec[i];}

  /*! Returns a pointer to the base link of the hand. */
  Link *getPalm() const {return base;}

  /*! Returns a pointer to the associated grasp object. */
  Grasp *getGrasp() const {return grasp;}

  //! Closes all fingers in a direction pre-specified (usually in config file)
  virtual bool autoGrasp(bool renderIt, double speedFactor = 1.0, bool stopAtContact = false);

  //! Opens the fingers in the opposite direction of \a autoGrasp
  virtual bool quickOpen(double speedFactor = 0.1);

  //! Moves the hand in the pre-specified approach direction until contact is made
  virtual bool approachToContact(double moveDist, bool oneStep = true);

  //! Moves the hand back until it is out of collision, then forward until a contact is made.
  virtual bool findInitialContact(double moveDist);
};

#define ROBOT_H
#endif

