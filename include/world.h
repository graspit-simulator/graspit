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
// $Id: world.h,v 1.45 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the simulation world that controls interactions between the world elements
 */
#ifndef WORLD_HXX

#include <list>
#include <vector>
#include <utility>
#include <set>
#include <QString>
#include <QObject>

#include "material.h"

#include <Inventor/SoType.h>

class vec3;
class position;
class mat3;
class Quaternion;
class transf;

class IVmgr;
class Hand;
class Robot;
class HumanHand;
class Body;
class DynamicBody;
class Link;
class GraspableBody;
class WorldElement;
class Tendon;

typedef std::vector<position> Neighborhood;

struct ContactDataS;
typedef struct ContactDataS ContactData;
typedef std::vector<ContactData> ContactReport;

struct CollisionDataS;
typedef struct CollisionDataS CollisionData;
typedef std::vector<CollisionData> CollisionReport;

class CollisionInterface;
class BoundingBox;
class TiXmlElement; 
class SoGroup;
class SoSeparator;
class SoIdleSensor;
class SoSensor;

//! The simulation world holds the world elements and handles their static and dynamic interactions
/*! A world object keeps track of all of the world elements (bodies and robots)
    contained within it.  It allows collisions to be turned on or off,
    completely, for a single body, or just between a pair of bodies.  It has
    several parameters that effect body interactions such as the table of
    coefficient of friction tables.  The world also contains methods advance
    the dynamic simulation of body motions within the world.  The state of
    a world may be written out to a simple text file which may be reloaded
    later.
 */
class World : public QObject {
  Q_OBJECT ;

protected:
  //! Pointer to the IVmgr who controls the simulation
  IVmgr *myIVmgr;

  //! Keeps track of the current simulation time.
  double worldTime;

  //! A vector of pointers to ALL of the bodies in the world
  std::vector<Body *> bodyVec;

  //! A vector of pointers to the graspable bodies in the world
  std::vector<GraspableBody *> GBVec;

  //! A vector of pointers to the robots defined within this world
  std::vector<Robot *> robotVec;        

  //! A vector of pointers to the hands defined within this world
  std::vector<Hand *> handVec;

  //! The number of bodies currently in this world
  int numBodies;

  //! The number of graspable bodies currently in this world
  int numGB;

  //! The number of robots currently in this world
  int numRobots;

  //! The number of hands currently in this world
  int numHands;

  //! The number of currently selected elements
  int numSelectedElements;

  //! The number of currently selected body elements
  int numSelectedBodyElements;

  //! The number of currently selected body elements
  int numSelectedRobotElements;

  //! The number of currently selected bodies
  int numSelectedBodies;

  //! Keeps track of whether the world has been modified since the last save
  bool modified;

  //! Keeps track of which hand is currently the focus of menu commands
  Hand *currentHand;

  //! A list of pointers to the currently selected worldElements
  std::list<WorldElement *> selectedElementList;

  //! A vector of pointers to the currently selected bodies
  std::vector<Body *> selectedBodyVec;
  
  //! keeps track is a tendon is selected or not
  bool isTendonSelected;

  //! pointer to selected tendon
  Tendon* selectedTendon;

  //! A flag to determine if collions checking is on or off
  bool allCollisionsOFF;

  //! Turns on or off soft contacts, needs a switch in the Iv environment
  bool softContactsON;

  //! A pointer to the collision detection instance
  CollisionInterface *mCollisionInterface;

  //! A pointer to the root of the world's Inventor scene graph
  SoSeparator *IVRoot;

  //! The number of materials defined within this world
  int numMaterials;
  
  //! A vector of strings, one for each material name
  std::vector<QString> materialNames;

  //! A numMaterials x numMaterials array of static coefficient of friction values
  double **cofTable;

  //! A numMaterials x numMaterials array of kinetic coefficient of friction values
  double **kcofTable;

  //! A flag to determine whether dynamics is currently being used or not
  bool dynamicsOn;

  //! A pointer to the Inventor idle sensor that is used when dynamics is on
  SoIdleSensor *idleSensor;

  //! The length of the default dynamics time step
  double dynamicsTimeStep;  

  //! Reads the user preferences for the world settings.  Under
  void readSettings();

  //! Saves the user preferences for the world settings.  
  void saveSettings();

  //! Static callback routine for the dynamic operations idle sensor.
  static void dynamicsCB(void *data,SoSensor *sensor);
  
  friend class Body;
  friend class DynamicBody;
  friend class MainWindow;

signals:
  //! Signal that a dynamic step has been completed
  void dynamicStepTaken();

  //! Signal a dynamics error has occured with an error string
  void dynamicsError(const char *errMsg);

  //! Signal that selections have changed
  void selectionsChanged();

  //! Signal that the number of world elements has changed
  void numElementsChanged();

  //! Signal that a hand was removed from the world
  void handRemoved();

  //! Signal that all grasps have been updated
  void graspsUpdated();

  //! Signal we want have changed the currently selected hand from within the code
  void handSelectionChanged();

  //! Signal that a tendon has been (de)selected
  void tendonSelectionChanged();

  //! Signal that the selected tendon's status has changed
  void tendonDetailsChanged();

public:	
  //! public constructor
  World(QObject *parent=0,const char *name=0, IVmgr *mgr=NULL);

  //! Saves the current user settings in the registry and clears the world
  ~World();
  
  //! Returns the number of bodies in this world
  int getNumBodies() const {return numBodies;}

  //! Returns the number of graspable bodies in this world
  int getNumGB() const {return numGB;}

  //! Returns the number of robots in this world
  int getNumRobots() const {return numRobots;}

  //! Returns the number of hands in this world
  int getNumHands() const {return numHands;}

  //! Returns the number of materials defined in this world
  int getNumMaterials() const {return numMaterials;}

  //! Returns the material index of the material named by matName
  int getMaterialIdx(const QString &matName) const; 

  //! Returns the name of the material with index i 
  QString getMaterialName(int i) const {return materialNames[i];}

  //! Returns the number of selected body elements 
  int getNumSelectedBodyElements() const {return numSelectedBodyElements;}
  
  //! Returns the number of selected robot elements 
  int getNumSelectedRobotElements() const {return numSelectedRobotElements;}

  //! Returns the number of selected world elements 
  int getNumSelectedElements() const {return numSelectedElements;}

  //! Returns the number of selected bodies 
  int getNumSelectedBodies() const {return numSelectedBodies;}

  //! Returns a pointer to the i-th selected body in the world 
  Body *getSelectedBody(int i) const {return selectedBodyVec[i];}

  //! returns the number of tendons in the currently selected hand 
  int getCurrentHandNumberTendons();

  //! Returns the currently selected tendond 
  Tendon* getSelectedTendon(){return selectedTendon;}

  //! returns the name of the i-th tendon of the currently selected hand 
  QString getSelectedHandTendonName(int i);

  //! query whether a tendon is selected or not 
  bool queryTendonSelected(){return isTendonSelected;}

  //! Set selected tendon 
  void selectTendon(Tendon *t);

  //! Sets the selected tendon, receives an index into the selected hand's list of tendons 
  void selectTendon(int i);

  //!deselect tendon 
  void deselectTendon();

  //! signals that a tendon has changed status, we need to update the tendon status bar 
  void tendonChange(){emit tendonDetailsChanged();}

  //! Returns the static coefficient of friction between two materials
  double getCOF(int mat1,int mat2) {return cofTable[mat1][mat2];}

  //! Returns the kinetic coefficient of friction between two materials
  double getKCOF(int mat1,int mat2) {return kcofTable[mat1][mat2];}

  //! Returns the current simulation time for this world 
  double getWorldTime() const {return worldTime;}

  //! Returns the default timestep for this world 
  double getTimeStep() const {return dynamicsTimeStep;}
  
  //! Returns whether this world has been modified since the last save. 
  bool wasModified() const {return modified;}

  //! Returns the root of the Inventor scene graph for this world 
  SoSeparator *getIVRoot() const {return IVRoot;}

  //! Returns a pointer to the i-th body defined in this world 
  Body *getBody(int i) const {return bodyVec[i];}

  //! Returns a pointer to the i-th graspable body defined in this world 
  GraspableBody *getGB(int i) const {return GBVec[i];}

  //! Returns a pointer to the i-th hand defined in this world 
  Hand *getHand(int i) const {return handVec[i];}

  //! Returns a pointer to the i-th robot defined in this world 
  Robot *getRobot(int i) const {return robotVec[i];}

  //! Returns whether the world element pointed to by e is currently selected or not
  bool isSelected(WorldElement *e) const;

  //! Returns a list of pointers to the currently selected world elements 
  const std::list<WorldElement *>& getSelectedElementList() const {return selectedElementList;}

  //! Returns a pointer to the hand that is the focus of menu commands etc. 
  Hand *getCurrentHand() const { return currentHand;}

  //! Returns whether dynamics are currently running or not. 
  bool dynamicsAreOn() const {return dynamicsOn;}

  //! Returns whether  soft contacts are on 
  bool softContactsAreOn() { return softContactsON; }

  //! Sets all world settings to their original default values. 
  void setDefaults();

  //! Sets the world modified flag.  Should be done when a change has since the last save. 
  void setModified() {modified = true;}

  //! Loads the saved world from the file in filename. 
  int load(const QString &filename);

  //! Loads the saved world from the XML file in filename.
  int loadFromXml(const TiXmlElement* root,QString rootPath);

  //! Save the current state of the world to a file.
  int save(const QString &filename);
  
  //! Creates a new body of type \a bodyType, then calls the load routine of that body
  Body *importBody(QString bodyType,QString filename);

  //! Creates a new body of type \a bodyType, then calls the loadFromXml routine of that body
  Body *importBodyFromXml(QString bodyType, const TiXmlElement* child, QString rootPath);

  //! Adds an already populated body to the world, but not to collision detection system
  void addBody(Body *body);

  //! Links are loaded in during the robot load method, but are added to the world with this routine
  void addLink(Link *newLink);

  //! Called when the user selects a new hand from the drop down menu.
  void setCurrentHand(Hand *hand) {currentHand = hand; emit handSelectionChanged();}

  //! Creates a robot, loads it from a file and adds it to the world
  Robot *importRobot(QString filename);
	
  //! Adds an already populated robot to the world,and possibly to the scene graph
  void addRobot(Robot *robot, bool addToScene = true);

  //! Removes the robot pointed to by robot from the world, then deletes it.
  void removeRobot(Robot *robot);

  //! Creates a new dynamic body from the data in the body pointed to by b.
  DynamicBody *makeBodyDynamic(Body *b, double mass);

  //! Deselects all currently selected world elements.
  void deselectAll();

  //! Adds the worldElement pointed to by e to the list of selected elements.
  void selectElement(WorldElement *e);

  //! Removes the worldElement pointed to by e from the list of selected elements.
  void deselectElement(WorldElement *e);

  //! Removes the element pointed to by e from the world 
  void destroyElement(WorldElement *e, bool deleteElement = true);
  
  //! Turns the collision detection system on or off
  void toggleAllCollisions(bool on);

  //! Toggle collisions for a world element (robot or body)
  void toggleCollisions(bool on, WorldElement *e1,WorldElement *e2=NULL);

  //! Check if collisions are enabled for a world element (robot or body)
  bool collisionsAreOff(WorldElement *e1=NULL, WorldElement *e2=NULL);

  //! Checks if collision between an entire robot and another element are on
  bool robotCollisionsAreOff(Robot *r, WorldElement *e);

  //! Answers true if there are no collisions in the world, potentially involving a specified element.
  bool noCollision(WorldElement *e=NULL);

  //! Queries the collision detection system for a report of which pairs of bodies are colliding.
  int getCollisionReport(CollisionReport *colReport, const std::vector<Body*> *interestList = NULL);

  //! Returns the minimum distance in mm between the two bodies; deprecated, use version on world elements
  // double getDist(Body *b1,Body *b2);

  //! Returns the minimum distance in mm between the two world elements
  double getDist(WorldElement *e1, WorldElement *e2);

  //! Returns the minimum distance in mm between the two bodies as well as the points on the bodies that are closest.
  double getDist(Body *b1,Body *b2, position &p1, position &p2);

  //! Finds every contact occurring between the pairs of bodies listed in the colReport.
  void findContacts(CollisionReport &colReport);

  //! Finds all contacts occurring on body b.  
  void findContacts(Body *b);

  //! Finds all the contacts occuring in the world.
  void findAllContacts();

  //! Sets up virtual contacts on all links at the closest points to a given body.
  void findVirtualContacts(Hand *hand, Body *object);

  //! Helper function that finds the closest point on a link to an object
  ContactData findVirtualContact(Link *link, Body* object);

  //! Returns the bounding boxes from the collision detection bounding box hierarchy of a body.
  void getBvs(Body *b, int depth, std::vector<BoundingBox> *bvs);

  //! Updates the grasp for any hand that has had contacts change since its grasp was last updated.
  void updateGrasps();

  //! Resets the external wrench accumulator for all the dynamic bodies.
  void resetDynamicWrenches();

  //! Starts the dynamic simulation for this world.
  void turnOnDynamics();

  //! Stops the dynamic simulation process.
  void turnOffDynamics();

  //! Resets dynamic simulation. Clears the dynamic stack, fixes all robot bases
  void resetDynamics();

  //! One complete step of the dynamics simulation. 
  void stepDynamics();

  //! Moves all dynamic bodies based on their velocity and the length of the current timestep.
  double moveDynamicBodies(double timeStep);

  //! Calls the dynamics routine to build and solve the LCP to find the velocities of the bodies
  int computeNewVelocities(double timeStep);

  //! Calls upon each dynamic body to save its current dynamic state to the top of its stack of states.
  void pushDynamicState();

  //! Restores the most recent dynamic state of each dynamic body by popping the state of each body's local stack.
  void popDynamicState();

  //! Finds a region around a given point on a body through the collision detection system
  void FindRegion( const Body *body, position point, vec3 normal, double radius,
	  			   Neighborhood *neighborhood);

  //! Returns the collision interface being used
  CollisionInterface *getCollisionInterface() {return mCollisionInterface;}

  //! Computes the distance between a point and a body
  vec3 pointDistanceToBody(position p, Body *b, vec3* normal = NULL);

  //! Adds back to the scene graph an element that has been removed
  void addElementToSceneGraph(WorldElement *e);
  //! Removes from the scene graph an element that is already part of this world
  void removeElementFromSceneGraph(WorldElement *e);

  //! Emits the signal that informs that grasps have been updated
  void emitGraspsUpdated(){emit graspsUpdated();}
};

#define WORLD_HXX
#endif


