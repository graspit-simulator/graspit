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
// Author(s): Matei T. Ciocarlie
//
// $Id: humanHand.h,v 1.15 2009/08/17 22:16:52 cmatei Exp $
//
//######################################################################

#ifndef HUMAN_HAND_H
#define HUMAN_HAND_H

/*! \file
	This file in intended to define a HumanHand subclass from Hand, to 
	encapsulate the specific characteristics for building an accurate 
	model of a human hand. However, this is not complete, and not really
	used yet.

	The main feature that is actually implemented here are tendons. We 
	define all the frameworks for defining tendons in a hand, as a list
	if insertion points and wrappers. You can visualize tendons and also
	use them to actuate the hand, but only in the dynamics engine. 
	However, this work is not complete. The human hand model that is 
	delivered with GraspIt actually does not use this (yet). 

	Other features, such as soft fingers, have found their way out of
	the human hand as they are used for robotic hands as well.
*/

#include <vector>
#include <Inventor/SoType.h>
#include "robot.h"

class SoSeparator;
class SoSphere;
class SoCylinder;
class SoTransform;
class SoDrawStyle;
class Tendon;

/*! An insertion point is a point where a tendon inserts into, or
  changes direction on a link of a robot. Currently it is the 
  only way to change the routing of a tendon, as intersection 
  of tendons and general geometry are not calculated. Insertion 
  points are also the only points where a tendon can apply force
  to the link. They are also called "pulleys" on occasion in the
  literature.
*/
class TendonInsertionPoint {
private:
  //! This tells wether the insertion point is permanent, or dynamic
  /*! Dynamic insertion points are those created when a tendon wraps 
    around a wrapper.*/
  bool mPermanent;
  
  //! Robot chain nnumber that tendon is attached to
  /*! -1 means insertion point is on the base of the robot*/
  int mAttachChainNr;
  //! Link number for the link that the insertion point is on
  int mAttachLinkNr;
  
  SoSeparator *mIVInsertion;
  SoMaterial *mIVInsertionMaterial;
  SoTransform *mIVInsertionTran;
  SoSphere *mIVInsertionGeom;
  
  SoSeparator *mIVConnector;
  SoMaterial *mIVConnectorMaterial;
  SoTransform *mIVConnectorTran;
  SoCylinder *mIVConnectorGeom;
  
  Tendon* mOwner;
public:
  
  //! The location of the insertion point, in link coordinates.
  vec3 mAttachPoint;
  //! The force applied by the tendon at this insertion point, in world coordinates.
  vec3 mInsertionForce;
  
  TendonInsertionPoint(Tendon* myOwner,int chain,int link,vec3 point,bool isPerm=true);
  
  void createInsertionGeometry();
  void createConnectorGeometry();
  void removeAllGeometry();
  Tendon* getTendon(){return mOwner;}
  void setPermanent(bool p){mPermanent=p;}
  bool isPermanent(){return mPermanent;}
  SbVec3f getWorldPosition();
  
  //! Use this function to get the link the insertion point is attached to.
  /*! Handles the case where tendon is attached to base correctly*/
  Link* getAttachedLink();
  
  SoSeparator* getIVInsertion(){return mIVInsertion;}
  SoSphere* getIVInsertionGeom(){return mIVInsertionGeom;}
  SoMaterial* getIVInsertionMaterial(){return mIVInsertionMaterial;}
  
  SoSeparator* getIVConnector(){return mIVConnector;}
  SoMaterial* getIVConnectorMaterial(){return mIVConnectorMaterial;}
  SoTransform* getIVConnectorTran(){return mIVConnectorTran;}
  SoCylinder* getIVConnectorGeom(){return mIVConnectorGeom;}
};

/*! The TendonWrapper is a cylindrical shape that a tendon is not alowed 
  to penetrate and must wrap around. Like an insertion point, it is 
  attached to a robot link, and defined in the link's coordinate system.
*/
class TendonWrapper{
private:
  int attachLinkNr;
  int attachChainNr;
  Robot *owner;

  SoSeparator *IVWrapper;
  SoMaterial *IVWrapperMaterial;
  SoTransform *IVWrapperTran;
  SoCylinder *IVWrapperGeom;
  
public:
  vec3 location,orientation;
  double radius;
  
  TendonWrapper(Robot* myOwner);
  Link* getAttachedLink();
  Robot* getRobot(){return owner;}
  
  void createGeometry();
  SoSeparator* getIVRoot(){return IVWrapper;}
  int getChainNr(){return attachChainNr;}
  int getLinkNr(){return attachLinkNr;}
  bool loadFromXml(const TiXmlElement* root);
};

//! Defines a tendon geometry by listing its insertion points
class Tendon{
private:
  Robot* mOwner;

  SoSeparator *mIVRoot;
  
  //! Used to toggle wether the tendon is visible or not; is inserted as first child of the root.
  SoDrawStyle* mIVVisibleToggle;

  //! Root for holding all related to force indicators rendering
  SoSeparator* mIVForceIndRoot;

  //! Used to toggle wether force indicators are visible or not
  SoDrawStyle* mIVForceIndToggle;

  //! Material for force indicators
  SoMaterial *mIVForceIndMaterial;
  
  //! Root for the arrows themselves showing forces at insertion points
  SoSeparator *mIVForceIndicators;

  //! Force applied as a result of voluntary muscle contraction
  float mActiveForce;
  
  //! Force applied as a result of tendon / muscle elongation
  float mPassiveForce;
  
  //! Used to flip passive force application on or off
  bool mApplyPassiveForce;
  
  //! The spring constant of this tendon, if thought of as a linear spring
  double mK;
  
  //! The list of insertion points that defines the geometry of the tendon
  /*! This has to be a list because we create ins points dynamically when wrapping around wrappers. */
  std::list<TendonInsertionPoint *> mInsPointList;
  
  QString mTendonName;

  bool mVisible;

  bool mForcesVisible;

  bool mSelected;
  
  //! The length of the tendon at the resting position
  float mRestLength;
  
  //! The current length of the tendon
  float mCurrentLength;

  int mNrInsPoints;

  void updateForceIndicators();

public:
  
  Tendon(Robot* myOwner);

  Robot *getRobot(){return mOwner;}

  SoSeparator* getIVRoot(){return mIVRoot;}
  
  //! Adds an insertion point at the end of the insPointList
  void addInsertionPoint(int chain,int link,vec3 point, bool isPerm);
  
  //! Inserts an insertion point at a given position in insPointList
  std::list<TendonInsertionPoint*>::iterator 
  insertInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos, 
                       int chain, int link, vec3 point, bool isPerm);
  
  //! Removes the insertion point pointed to by the given iterator
  void removeInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos);
  
  //! Updates connector geometry based on movement of links that tendon inserts into
  void updateGeometry();
  
  //! Updates insertion point forces based on movement of links that tendon inserts into
  void updateInsertionForces();
  
  //! Computes a naive version of passive forces based on tendon excursion; more of a placeholder.
  void computeSimplePassiveForces();
  
  //! Checks if tendon intersect a wrapper and adds insertion points around the wrapper if needed
  void checkWrapperIntersections();
  
  //! Removes wrapper intersections if they are no longer needed
  void removeWrapperIntersections();
  
  void select();
  void deselect();
  bool isSelected(){return mSelected;}
  
  void setActiveForce(float f);

  void setPassiveForce(float f);

  float getActiveForce(){return mActiveForce;}

  float getPassiveForce(){return mPassiveForce;}

  float getTotalForce(){if (mApplyPassiveForce) return mActiveForce+mPassiveForce; else return mActiveForce;}

  void setStiffness(double k){mK = k;}

  double getStiffness() {return mK;}
  
  void setName(QString name){mTendonName=name;}

  QString getName(){return mTendonName;}
  
  void setVisible(bool v);

  bool isVisible(){return mVisible;}

  void setForcesVisible(bool v);

  bool forcesVisible() {return mForcesVisible;}
  
  //! Applies previously computed forces at links that have insertion points. 
  /*! Forces MUST have been updated by updateGeometry() and updateInsertionForces()*/
  void applyForces();

  void setApplyPassiveForce(bool b){mApplyPassiveForce=b;}
  
  /*sets the current state of the tendon as rest state*/
  void setRestPosition();
  
  //! Returns the excursion of the end of the tendon compared to its rest position.
  /*! For the moment we consider that tendon elongation is actually 
    excursion, as if the origin was free to move. */
  float getExcursion(){return mCurrentLength - mRestLength;}

  bool loadFromXml(const TiXmlElement *root);

  transf getInsertionPointWorldTransform(std::list<TendonInsertionPoint*>::iterator insPt);

  //! Returns the locations of all insertion points in world coordinates
  /*! The transform is such that the resultant tendon force, if any, points along the z axis
    of the local insertion point coordinate frame.
  */
  void getInsertionPointTransforms(std::vector<transf> &insPointTrans);
  
  //! Returns the magnitudes of the forces at each insertion point, assuming a total tendon force of 1.0
  void getInsertionPointForceMagnitudes(std::vector<double> &magnitudes);

  //! Returns pairs of insertion points and their links, with ins. pt. transforms relative to their links
  void getInsertionPointLinkTransforms(std::list< std::pair<transf, Link*> > &insPointLinkTrans);
  
};

//! A hand with tendon information
/*! Currently, the only thing that a human hand has different from a 
  robotic hand is a list of tendons. It will load these tendons from
  the .cfg file and display them, and the GUI can be used to visualize
  the tendon geometry, excursion, etc. In the future, the human hand
  might gain more features.
*/
class HumanHand : public Hand {
  Q_OBJECT
protected:
  std::vector<Tendon *> mTendonVec;

  std::vector<TendonWrapper *> mTendonWrapperVec;

  //! Applies dynamic tendon forces, to be called during an interation of the dynamic engine
  void applyTendonForces();

public:
  HumanHand(World*,const char*);

  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

  void updateTendonGeometry();

  int getNumTendons(){return mTendonVec.size();}

  int getNumTendonWrappers(){return mTendonWrapperVec.size();}

  Tendon* getTendon(int i){return mTendonVec[i];}

  TendonWrapper* getTendonWrapper(int i){return mTendonWrapperVec[i];}

  void selectTendon(int i){mTendonVec[i]->select();}

  void deselectTendon(int i){mTendonVec[i]->deselect();}

  //! Applies tendon forces rather than calling the PD Controller
  virtual void DOFController(double timeStep);

  //! Computes if tendon forces are self-balanced around all joints of the hand
  int tendonEquilibrium();
};

#endif
