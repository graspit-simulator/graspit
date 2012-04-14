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
// $Id: humanHand.cpp,v 1.23 2009/08/17 22:14:44 cmatei Exp $
//
//######################################################################

#include "humanHand.h"

#include <QFile>
#include <QTextStream>
#include <list>

#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>

#include "SoArrow.h"

#include "grasp.h"
#include "matrix.h"
#include "tinyxml.h"
#include "debug.h"
#include "world.h"

#define WRAPPER_TOLERANCE 0.995

//#define PROF_ENABLED
#include "profiling.h"

//const double TendonInsertionPoint::INSERTION_POINT_RADIUS = 1.5;
//const double TendonInsertionPoint::CONNECTOR_RADIUS = 0.8;

const double TendonInsertionPoint::INSERTION_POINT_RADIUS = 0.45;
const double TendonInsertionPoint::CONNECTOR_RADIUS = 0.24;

/*! Returns the distance between an infinite line defined by l1 and l2 and a point p0.
  Modified from http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
*/
double pointLineDistance(vec3 p0, vec3 l1, vec3 l2)
{
  vec3 x01 = p0 - l1;
  vec3 x02 = p0 - l2;
  vec3 x21 = l2 - l1;
  return (x01 * x02).len() / x21.len();
}

PROF_DECLARE(ROTATE_SO_TRANSFORM);
void rotateSoTransform(SoTransform *tran, vec3 axis, double angle)
{
  PROF_TIMER_FUNC(ROTATE_SO_TRANSFORM);
  transf tr(tran);
  Quaternion quat(angle, axis);
  transf rot(quat, vec3(0,0,0));
  tr = rot * tr;
  tr.toSoTransform(tran);
}

TendonInsertionPoint::TendonInsertionPoint(Tendon *myOwner, int chain, int link, vec3 point, double mu, bool isPerm) : 
  mAttachPoint(point),
  mPermanent(isPerm),
  mAttachChainNr(chain),
  mAttachLinkNr(link),
  mMu(mu),
  mOwner(myOwner)
{
  createInsertionGeometry();
  createConnectorGeometry();
}

void TendonInsertionPoint::setAttachPoint(vec3 attachPoint)
{
  mAttachPoint = attachPoint;
  mIVInsertionTran->translation.setValue(mAttachPoint.x() , mAttachPoint.y() , mAttachPoint.z() );
}
	
/*! Use this function to get the link the insertion point is 
  attached to. If chain is -1 this means it is attached to 
  the base of the robot this function will handle this 
  correctly.
*/
Link* TendonInsertionPoint::getAttachedLink()
{  
  if (mAttachChainNr == -1 || mAttachLinkNr == -1) {
    // insertion point is attached to robot base
    return mOwner->getRobot()->getBase();
  } else {
    return mOwner->getRobot()->getChain(mAttachChainNr)->getLink(mAttachLinkNr);
  }
}

//PROF_DECLARE(TIP_GET_WORLD_POSITION);
SbVec3f TendonInsertionPoint::getWorldPosition()
{
  //PROF_TIMER_FUNC(TIP_GET_WORLD_POSITION);
  position worldPos;
  worldPos = position(mAttachPoint.toSbVec3f()) * ( getAttachedLink()->getTran() );
  return worldPos.toSbVec3f();
}

void TendonInsertionPoint::createInsertionGeometry()
{
  mIVInsertion = new SoSeparator;
  mIVInsertionMaterial = new SoMaterial;
  mIVInsertionTran = new SoTransform;
  mIVInsertionGeom = new SoSphere;
  
  /*insert a pointer to the transform of the link this insertion point is attached to
    like this, we don't have to worry about updating anything.
    One problem: cleanup. This reference might prevent link's tranform IV node from 
    being deleted when it should be. */
  mIVInsertion->addChild(getAttachedLink()->getIVTran());
  
  /* insertion point's location relative to link's origin*/
  mIVInsertionTran->translation.setValue(mAttachPoint.x() , mAttachPoint.y() , mAttachPoint.z() );
  mIVInsertion->addChild(mIVInsertionTran);
  
  if ( isPermanent() )
  {
    mIVInsertionMaterial->diffuseColor.setValue( (float)0.7 , (float)0.2 , (float)0.2);
    mIVInsertionGeom->radius=(float)INSERTION_POINT_RADIUS;
  }
  else
  {
    if (mOwner->isSelected())
      mIVInsertionMaterial->diffuseColor.setValue( (float)1.0 , (float)0.5 , (float)0.5);
    else
      mIVInsertionMaterial->diffuseColor.setValue( (float)0.5 , (float)0.5 , (float)0.5);
    mIVInsertionGeom->radius=(float)CONNECTOR_RADIUS;
  }
  mIVInsertion->addChild(mIVInsertionMaterial);
  mIVInsertion->addChild(mIVInsertionGeom);
}

void TendonInsertionPoint::createConnectorGeometry()
{
  mIVConnector = new SoSeparator;
  mIVConnectorTran = new SoTransform;
  mIVConnectorMaterial = new SoMaterial;
  mIVConnectorGeom = new SoCylinder;
  
  if ( mOwner->isSelected() )
    mIVConnectorMaterial->diffuseColor.setValue(1.0 , 0.5 , 0.5);
  else
    mIVConnectorMaterial->diffuseColor.setValue(0.5 , 0.5 , 0.5);
  
  mIVConnector->addChild( mIVConnectorTran );
  mIVConnector->addChild( mIVConnectorMaterial );
  mIVConnector->addChild( mIVConnectorGeom);
}

void TendonInsertionPoint::removeAllGeometry()
{
  mIVConnector->removeAllChildren();
  mIVInsertion->removeAllChildren();
}

Tendon::Tendon(Robot *myOwner)
{
  mOwner = myOwner;
  mActiveForce = 0;
  mIVRoot = new SoSeparator;
  mIVVisibleToggle = new SoDrawStyle();
  mIVVisibleToggle->style = SoDrawStyle::FILLED;
  mIVRoot->addChild(mIVVisibleToggle);
  mTendonName = "unnamed";
  mVisible = true;
  mForcesVisible = false;
  mSelected = false;
  mRestLength = 0;
  mCurrentLength = 0;
  mDefaultRestLength = -1;
  mPreTensionLength = -1;
  mApplyPassiveForce = true;
  mPassiveForce = 0;
  mK = 0.0;

  mIVForceIndRoot = new SoSeparator;
  mIVRoot->addChild(mIVForceIndRoot);
  mIVForceIndToggle = new SoDrawStyle; 
  mIVForceIndToggle->style = SoDrawStyle::INVISIBLE;
  mIVForceIndRoot->addChild(mIVForceIndToggle);
  mIVForceIndMaterial = new SoMaterial;
  mIVForceIndMaterial->diffuseColor.setValue(0.2f , 0.4f , 0.4f);
  mIVForceIndRoot->addChild(mIVForceIndMaterial);
  mIVForceIndicators = new SoSeparator;
  mIVForceIndRoot->addChild(mIVForceIndicators);
}

void Tendon::setActiveForce(float f)
{
  if (f>=0) mActiveForce = f;
  else mActiveForce = 0;
  updateInsertionForces();
  if (mVisible && mForcesVisible) updateForceIndicators();
}
void Tendon::setPassiveForce(float f)
{
  if (f>=0) mPassiveForce = f;
  else mPassiveForce = 0;
  updateInsertionForces();
  if (mVisible && mForcesVisible) updateForceIndicators();
}

PROF_DECLARE(TENDON_REMOVE_TEMP_INSERTION_POINTS);
void Tendon::removeTemporaryInsertionPoints()
{
  PROF_TIMER_FUNC(TENDON_REMOVE_TEMP_INSERTION_POINTS);
  std::list<TendonInsertionPoint*>::iterator it = mInsPointList.begin();
  while (it != mInsPointList.end())
  {
    if ( (*it)->isPermanent() ) it++;
    else it = removeInsertionPoint(it);
  }
}

bool Tendon::insPointInsideWrapper()
{
  int i=0;
  for (std::list<TendonInsertionPoint*>::iterator insPt = mInsPointList.begin();
       insPt != mInsPointList.end();
       insPt++)
  {
    if (!(*insPt)->isPermanent()) continue;
    vec3 p0((*insPt)->getWorldPosition());
    for (int j=0; j< ((HumanHand*)getRobot())->getNumTendonWrappers(); j++) 
    {
      if ( ((HumanHand*)getRobot())->getTendonWrapper(j)->isExempt(mTendonName) ) continue;
      TendonWrapper *wrapper = ((HumanHand*)getRobot())->getTendonWrapper(j);
      //two points along axis of tendon wrapper
      vec3 l1 = wrapper->location;
      vec3 l2 = l1 + wrapper->orientation;
      //convert them to world coordinates 
      Link* link = wrapper->getAttachedLink();
      position tmpPos = position (l1.toSbVec3f()) * ( link->getTran());
      l1 = vec3 ( tmpPos.toSbVec3f() );
      tmpPos = position (l2.toSbVec3f()) * ( link->getTran());
      l2 = vec3 ( tmpPos.toSbVec3f() );

      double d = pointLineDistance(p0, l1, l2);
      if (d < wrapper->radius)
      {
        //std::cerr << "Ins point " << i << "  inside wrapper " << j;
        return true;
      }
    }
    i++;
  }
  return false;
}

double Tendon::minInsPointDistance()
{
  double minDist = std::numeric_limits<double>::max();
  for (std::list<TendonInsertionPoint*>::iterator thisInsPt = mInsPointList.begin();
       thisInsPt != mInsPointList.end();
       thisInsPt++)
  {
    if (!(*thisInsPt)->isPermanent()) continue;

    std::list<TendonInsertionPoint*>::iterator nextInsPt = thisInsPt;
    nextInsPt++;
    while( nextInsPt!=mInsPointList.end() && !(*nextInsPt)->isPermanent() ) nextInsPt++;
    if (nextInsPt == mInsPointList.end()) continue;

    minDist = std::min( minDist, ( vec3((*thisInsPt)->getWorldPosition()) - 
                                   vec3((*nextInsPt)->getWorldPosition()) ).len() );
  }
 return minDist;
}

inline bool wrapperIntersection(TendonWrapper *wrapper, QString tendonName, 
                                vec3 pPrev, vec3 pNext, position &newInsPtPos)
{
  if ( wrapper->isExempt(tendonName) ) return false;
  //two points at the two ends of the tendon wrapper
  vec3 P3 = wrapper->location;
  vec3 P4 = P3 + 0.5 * wrapper->length * wrapper->orientation;
  P3 = P3 - 0.5 * wrapper->length * wrapper->orientation;
  //convert them to world coordinates 
  Link* link = wrapper->getAttachedLink();
  position tmpPos = position (P3.toSbVec3f()) * ( link->getTran());
  P3 = vec3 ( tmpPos.toSbVec3f() );
  tmpPos = position (P4.toSbVec3f()) * ( link->getTran());
  P4 = vec3 ( tmpPos.toSbVec3f() );
  
  vec3 Pa, Pb;
  double mua, mub;
  LineLineIntersect( pPrev , pNext , P3,P4 , &Pa, &Pb, &mua, &mub);

  // if closest point is not between wrapper edges we are done
  if (mub<0 || mub>1) return false;

  // if  closest point is not between insertion point we are done
  // this also exits if one of the insertion points is closest, which warrants a closer look
  if (mua<=0 || mua>=1) return false;
  
  /*
  //check if tendon is on wrong side of wrapper
  vec3 wrapping_direction;
  if (wrapper->wrappingSide(tendonName, wrapping_direction))
  {
    //convert closest points to link coordinates
    position Pa_link = position(Pa.toSbVec3f()) * ( link->getTran().inverse());
    position Pb_link = position(Pb.toSbVec3f()) * ( link->getTran().inverse());
    vec3 dPrevLink = Pa_link - Pb_link;
    double length = dPrevLink.len();
    dPrevLink = normalise(dPrevLink);
    //DBGA("Tendon " << tendonName.latin1() << " has wrapping side; direction is " << dPrevLink);
    if ( dPrevLink % wrapping_direction < 0 ) {
      DBGA("Tendon " << tendonName.latin1() << " on the wrong side of a wrapper at mua " << mua);
      //new insertion point is in opposite direction
      vec3 dRes = vec3(Pb_link.toSbVec3f() ) - ( wrapper->radius) * dPrevLink;
      newInsPtPos = position ( dRes.toSbVec3f() );
      return true;
    }    
  }
  */
  // check if tendon is too close to wrapper
  vec3 dPrev = Pa - Pb;
  if (dPrev.len() < WRAPPER_TOLERANCE * wrapper->radius)
  {
    // compute location of new insertion point - on cylinder edge 
    dPrev = normalise(dPrev);
    vec3 dRes = Pb + ( wrapper->radius ) * dPrev;
    // transform it to coordinate system of wrapper */          
    newInsPtPos = position ( dRes.toSbVec3f() ) * ( link->getTran().inverse() );
    return true;
  }
  return false;
}

PROF_DECLARE(TENDON_REMOVE_INTERSECTIONS);
void Tendon::removeWrapperIntersections()
{
  PROF_TIMER_FUNC(TENDON_REMOVE_INTERSECTIONS);  
  for (std::list<TendonInsertionPoint*>::iterator insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    std::list<TendonInsertionPoint*>::iterator nextInsPt = insPt;
    nextInsPt ++;
    if ( !(*insPt)->isPermanent() && insPt!=mInsPointList.begin() && nextInsPt!=mInsPointList.end() )
    {
      std::list<TendonInsertionPoint*>::iterator prevInsPt = insPt;    
      prevInsPt--;      
      vec3 pPrev = vec3( (*prevInsPt)->getWorldPosition() );
      vec3 pNext = vec3( (*nextInsPt)->getWorldPosition() );
      bool needed = false;
      for (int j=0; j< ((HumanHand*)getRobot())->getNumTendonWrappers(); j++)
      {
        TendonWrapper *wrapper = ((HumanHand*)getRobot())->getTendonWrapper(j);
        position foo;
        if (wrapperIntersection(wrapper, mTendonName, pPrev, pNext, foo))
        {
          needed = true;
          break;
        }        
      }
      if (!needed)
      {
        removeInsertionPoint(insPt);
        insPt = prevInsPt;
      }
    }
  }
}

/*! Checks if a connector penetrates a cylindrical wrapper by more than the 
	tolerance value. If so, it adds a temporary insertion point on the edge 
	of the cylider.	Problems:
	- creates a small "jump" in the connector as it goes from being INSIDE 
	  the wrapper (acc. to tolerance) to passing through a point ON THE EDGE 
	  of the wrapper. This might create discontinuities between time steps
	  for dynamics' numerical integration
	- does not allow for lateral "sliding" of the tendon on the wrapper
*/
PROF_DECLARE(TENDON_CHECK_INTERSECTIONS);
void Tendon::checkWrapperIntersections()
{
  PROF_TIMER_FUNC(TENDON_CHECK_INTERSECTIONS);
  std::list<TendonInsertionPoint*>::iterator insPt = mInsPointList.begin();  
  int num_insertions = 0;
  while (insPt!=mInsPointList.end()) 
  {
    bool new_insertion = false;
    std::list<TendonInsertionPoint*>::iterator nextInsPt = insPt;
    nextInsPt ++;
    if (nextInsPt != mInsPointList.end() ) {
      vec3 pCur = vec3( (*insPt)->getWorldPosition() );
      vec3 pNext = vec3 ( (*nextInsPt)->getWorldPosition() );  
      for (int j=0; j< ((HumanHand*)getRobot())->getNumTendonWrappers(); j++) 
      {
        TendonWrapper *wrapper = ((HumanHand*)getRobot())->getTendonWrapper(j);
        if ( wrapper->isExempt(mTendonName)) continue;
        position newInsPtPos;
        if ( wrapperIntersection(wrapper, mTendonName, pCur, pNext, newInsPtPos) ) {          
          int chainNr = wrapper->getChainNr();
          int linkNr = wrapper->getLinkNr();          
          insertInsertionPoint( nextInsPt, chainNr, linkNr, vec3(newInsPtPos.toSbVec3f()), wrapper->getMu(), false );
          new_insertion = true;
        }
      }
    }
    if (new_insertion && num_insertions++ > 10) 
    {
      DBGA("More than 10 new tendon insertions in a single point; loop might be stuck, forcing continuation.");
      new_insertion = false;
    }
    if (!new_insertion) 
    {
      insPt++;
      num_insertions = 0;
    }
  }
}

/*!	Updates the geometry of the connectors between insertion points, which
	need to move together with the robot's links. However, some of them 
	depend on two links, so we can not use link transforms directly (like 
	we do for the geometry of the insertion points).Rather, this geometry 
	needs to be recomputed after every change in link status.
*/
PROF_DECLARE(TENDON_UPDATE_GEOMETRY);
PROF_DECLARE(TENDON_COMPUTE_GEOMETRY_CORE);
void Tendon::updateGeometry()
{
  PROF_TIMER_FUNC(TENDON_UPDATE_GEOMETRY);
  /*first we wrap tendon around wrappers*/
  checkWrapperIntersections();
  
  /*and we remove unnecessary ones*/
  removeWrapperIntersections();
  
  /* we also compute the length of the tendon as the sum of connector lengths*/
  mCurrentLength = 0;
  
  PROF_START_TIMER(TENDON_COMPUTE_GEOMETRY_CORE);
  std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, newInsPt;  
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    prevInsPt = insPt;
    if (insPt!=mInsPointList.begin())
    {
      prevInsPt--;
      SbVec3f pPrev = (*prevInsPt)->getWorldPosition();
      SbVec3f pCur = (*insPt)->getWorldPosition();
      vec3 dPrev = vec3(pCur) - vec3(pPrev);
      // distance between the two
      float m = dPrev.len();
      // add it to tendon length
      mCurrentLength += m;

      //updates to IV geometry are made only if tendon is visible
      //this should be consolidated in the future
      if (mVisible)
      {
        // midpoint between the two 
        vec3 c = vec3(pPrev) + dPrev*0.5;
        SoTransform* tran = (*insPt)->getIVConnectorTran();
        SoCylinder* geom = (*insPt)->getIVConnectorGeom();
        geom->radius=(float)TendonInsertionPoint::CONNECTOR_RADIUS;
        geom->height = m;
        tran->pointAt(c.toSbVec3f(),pPrev);
        //neg 90 x
        rotateSoTransform(tran, vec3(1.0, 0.0, 0.0), -1.5707);
      }
    } 
    else if (mVisible)
    {
      // make the cylinder tiny so that it is not visible
      SoCylinder* geom = (*insPt)->getIVConnectorGeom();
      geom->radius=0.1f;
      geom->height = 0.1f;
    }
  }
  PROF_STOP_TIMER(TENDON_COMPUTE_GEOMETRY_CORE);
  
  /*after geometry has been updated, go ahead and update forces*/
  computeSimplePassiveForces();
  updateInsertionForces();
  if (mVisible && mForcesVisible) updateForceIndicators();
}

PROF_DECLARE(TENDON_UPDATE_FORCE_INDICATORS);
void Tendon::updateForceIndicators()
{
  PROF_TIMER_FUNC(TENDON_UPDATE_FORCE_INDICATORS);
  mIVForceIndicators->removeAllChildren();
  std::vector<transf> insPointTrans;
  std::vector<double> insPointMagn;
  getInsertionPointTransforms(insPointTrans);
  getInsertionPointForceMagnitudes(insPointMagn);

  /*
  if (mInsPointList.size() != insPointTrans.size()) DBGA("Error 1");
  size_t i=0;
  std::list<TendonInsertionPoint*>::iterator insPt;
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    Link* link = (*insPt)->getAttachedLink();	
    SbVec3f tmp = ( (*insPt)->getAttachPoint() ).toSbVec3f();
    position pos = position(tmp) * ( link->getTran() );
    
    vec3 force = (*insPt)->mInsertionForce; 

    DBGA("Transform:\n" << insPointTrans[i]);
    mat3 mat(insPointTrans[i].rotation());
    DBGA("Rotation:\n " << mat);
    DBGA("Position: " << pos);
    DBGA("Magnitudes, from arrows: " << insPointMagn[i]* getTotalForce() << " and from forces: " << force.len());
    DBGA("Force direction: " << normalise(force));
    DBGA("\n-----------------------------------\n");
    i++;
  }
*/

  if (insPointTrans.size() != insPointMagn.size())
  {
    DBGA("Error: number of ins point trans does not match number of ins point magn");
    return;
  }
  for (size_t i=0; i<insPointTrans.size(); i++)
  {
    SoTransform* forceTrans = new SoTransform;
    insPointTrans[i].toSoTransform(forceTrans);
    //pos 90 x
    rotateSoTransform(forceTrans, vec3(1.0, 0.0, 0.0), 1.5707);
    SoArrow *arrow = new SoArrow;
    arrow->height = 10.0 * 1.0e-6 * insPointMagn[i] * getTotalForce();    
    arrow->cylRadius = 0.25;
    arrow->coneRadius = 0.5;
    if (arrow->height.getValue() < arrow->coneHeight.getValue()) {
      arrow->coneHeight = arrow->height.getValue() / 2.0;
    }     
    SoSeparator* forceIndSep = new SoSeparator;
    forceIndSep->addChild(forceTrans);
    forceIndSep->addChild(arrow);
    mIVForceIndicators->addChild(forceIndSep);
    DBGP("Active force: " << mActiveForce << "; Passive force: " << mPassiveForce);
    DBGP("Added indicator of length" << 10.0 * 1.0e-6 * insPointMagn[i] * getTotalForce());
  }
}

transf Tendon::getInsertionPointWorldTransform(std::list<TendonInsertionPoint*>::iterator insPt)
{
  SbVec3f pCur = (*insPt)->getWorldPosition();
  std::list<TendonInsertionPoint*>::iterator prevInsPt = insPt; prevInsPt--;
  std::list<TendonInsertionPoint*>::iterator nextInsPt = insPt; nextInsPt++;
  vec3 dPrev(0,0,0), dNext(0,0,0);
  
  if (insPt != mInsPointList.begin()) dPrev = normalise(vec3((*prevInsPt)->getWorldPosition()) - vec3(pCur));
  if (nextInsPt != mInsPointList.end()) dNext = normalise(vec3((*nextInsPt)->getWorldPosition()) - vec3(pCur));
  
  SoTransform* tran = new SoTransform;
  tran->pointAt(pCur,(vec3(pCur)+dPrev + dNext).toSbVec3f());
  //neg 180 x
  rotateSoTransform(tran, vec3(1.0, 0.0, 0.0), -3.14159);
  transf tr(tran);
  tran->ref();
  tran->unref();
  return tr;
}

PROF_DECLARE(TENDON_GET_INS_POINT_TRANSFORMS);
void Tendon::getInsertionPointTransforms(std::vector<transf> &insPointTrans)
{
  PROF_TIMER_FUNC(TENDON_GET_INS_POINT_TRANSFORMS);
  if (mInsPointList.size() <= 1) {
    DBGA("Insertion point transforms ill-defined, not enough insertion points");
    return;
  }
  std::list<TendonInsertionPoint*>::iterator insPt;  
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    insPointTrans.push_back( getInsertionPointWorldTransform(insPt) );
  }
}

/*! The result of this function can be fed directly into the Grasp::Jacobian computation */
void Tendon::getInsertionPointLinkTransforms(std::list< std::pair<transf, Link*> > &insPointLinkTrans)
{
  if (mInsPointList.size() <= 1) {
    DBGA("Insertion point transforms ill-defined, not enough insertion points");
    return;
  }
  std::list<TendonInsertionPoint*>::iterator insPt;  
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    //make the transform relative to the link and insert it in list
    transf linkTrans =  getInsertionPointWorldTransform(insPt) * (*insPt)->getAttachedLink()->getTran().inverse();   
    insPointLinkTrans.push_back( std::pair<transf, Link*>(linkTrans, (*insPt)->getAttachedLink()) );
  }
}

PROF_DECLARE(TENDON_GET_INS_POINT_FORCE_MAGNITUDES);
void Tendon::getInsertionPointForceMagnitudes(std::vector<double> &magnitudes)
{
  PROF_TIMER_FUNC(TENDON_GET_INS_POINT_FORCE_MAGNITUDES);
  if (mInsPointList.size() <= 1) {
    DBGA("Insertion point transforms ill-defined, not enough insertion points");
    return;
  }
  std::list<TendonInsertionPoint*>::iterator insPt;  
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    SbVec3f pCur = (*insPt)->getWorldPosition();
    std::list<TendonInsertionPoint*>::iterator prevInsPt = insPt; prevInsPt--;
    std::list<TendonInsertionPoint*>::iterator nextInsPt = insPt; nextInsPt++;
    vec3 dPrev(0,0,0), dNext(0,0,0);

    if (insPt != mInsPointList.begin()) dPrev = normalise( vec3(pCur) - vec3((*prevInsPt)->getWorldPosition()) );
    if (nextInsPt != mInsPointList.end()) dNext = normalise( vec3(pCur) - vec3((*nextInsPt)->getWorldPosition()) );
    magnitudes.push_back( vec3(dPrev + dNext).len() );
  }
}

double Tendon::getTotalFrictionCoefficient()
{
  return getFrictionCoefficientBetweenPermInsPoints(0, getNumPermInsPoints() - 1, true);
}


double Tendon::getFrictionCoefficientBetweenPermInsPoints(int start, int end, bool inclusive)
{
  if (start < 0 || end >= getNumPermInsPoints() || start > end) {
    DBGA("Incorrect start or end for tendon friction interval");
    return -1.0;
  }
  std::vector<double> magnitudes;
  getInsertionPointForceMagnitudes(magnitudes);
  assert(magnitudes.size() == mInsPointList.size());
  
  std::list<TendonInsertionPoint*>::const_iterator it;
  int num = 0, count = 0;
  double totalMu = 0.0;
  for (it=mInsPointList.begin(); it!=mInsPointList.end(); it++)
  {
    if (inclusive && num >= start) 
    {
      totalMu += magnitudes.at(count) * (*it)->getMu();
    }
    else if (num > start && num < end) totalMu += magnitudes.at(count) * (*it)->getMu();
    if ((*it)->isPermanent()) num++;
    if (num > end) break;
    count++;
  }
  assert(num > end);
  return totalMu;
}

/*! Simple method for some passive force computation
  - assume inextensible tendon is attached to a muscle, 
  therefore tendon elongation is actually muscle elongation
  - hard-code some values for muscle rest length and max. force 
  (each muscle should actually have its own values)
  - use formula from Hill-type model (Tsang et al.) for computing 
  muscle force
*/
//PROF_DECLARE(TENDON_COMPUTE_PASSIVE_FORCES);
void Tendon::computeSimplePassiveForces()
{
  //PROF_TIMER_FUNC(TENDON_COMPUTE_PASSIVE_FORCES);
  //only apply passive force if tendon is elongated
  if ( getExcursion() < 0 ) {
    mPassiveForce = 0;
    return;
  }
  /*
  //stiffness based on muscle elongation
  //numbers loosely inspired by Pollard and Gilbert, 2002 / Gonzales, Delp et al, 1997
  float muscleMaxForce=5*125*1.0e6; //convert from newtons!
  float muscleRestLength=70;
  float elongation = ( muscleRestLength + getExcursion() ) / muscleRestLength;
  mPassiveForce = 2.77 * (elongation - 1) * (elongation - 1) * muscleMaxForce; 
  */
  //stiffness based on linear springs
  mPassiveForce = getExcursion() * getStiffness();
}

/*! Given a level of active and passive forces MAGNITUDES this 
  computes what force is applied at each insertion point, and in 
  what DIRECTION.	Geometry needs to be up-to-date 
  (use updateGeometry() ).
*/
PROF_DECLARE(TENDON_UPDATE_INSERTION_FORCES);
void Tendon::updateInsertionForces()
{
  PROF_TIMER_FUNC(TENDON_UPDATE_INSERTION_FORCES);
  SbVec3f pPrev,pCur,pNext;
  position tmpPos;
  vec3 dPrev,dNext,dRes,c;
  
  std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, nextInsPt, newInsPt;
  
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    prevInsPt = insPt;
    nextInsPt = insPt;
    nextInsPt ++;
    
    if (insPt!=mInsPointList.begin())
    {
      prevInsPt--;
      pPrev = (*prevInsPt)->getWorldPosition();
    }
    
    pCur = (*insPt)->getWorldPosition();
    
    if (nextInsPt != mInsPointList.end() )
      pNext = (*nextInsPt)->getWorldPosition();
    
    /*compute resultant force on insertion point*/
    if (insPt == mInsPointList.begin())
    {
      /*first insertion point: force is applied in direction to pNext*/
      dNext = vec3(pNext) - vec3(pCur);
      dNext = ( (float)1 / dNext.len() ) * dNext;
      dRes = getTotalForce() * dNext;
      (*insPt)->mInsertionForce = dRes;
    }
    else if ( nextInsPt != mInsPointList.end() )
    {
      /*middle insertion points: force is resultant of forces along connectors to pPrev and pNext*/
      dPrev = vec3(pPrev) - vec3(pCur);
      dNext = vec3(pNext) - vec3(pCur);
      dPrev = ( (float)1 / dPrev.len() ) * dPrev;
      dNext = ( (float)1 / dNext.len() ) * dNext;
      dRes = getTotalForce() * (dPrev + dNext);
      (*insPt)->mInsertionForce = dRes;
    }
    else
    {
      /*last insertion point: force is applied in direction to pPrev */
      dPrev = vec3(pPrev) - vec3(pCur);
      dPrev = ( (float)1 / dPrev.len() ) * dPrev;
      dRes = getTotalForce() * dPrev;
      (*insPt)->mInsertionForce = dRes;
    }
  }
}

void Tendon::setRestLength(double length)
{
  if (length==0) DBGA("WARNING: length 0 set on tendon");
  mRestLength = length;
  computeSimplePassiveForces();
  updateInsertionForces();
  if (mVisible && mForcesVisible) updateForceIndicators();
}

void Tendon::select()
{
  std::list<TendonInsertionPoint*>::iterator insPt;
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    SoMaterial* mat = (*insPt)->getIVInsertionMaterial();
    if ( (*insPt)->isPermanent() ) mat->diffuseColor.setValue(1.0f,0.7f,0.7f);
    else mat->diffuseColor.setValue(1.0f,0.5f,0.5f);    
    if (insPt!=mInsPointList.begin())
    {
      mat = (*insPt)->getIVConnectorMaterial();
      mat->diffuseColor.setValue(1.0f,0.5f,0.5f);
    }
  }
  mIVForceIndMaterial->diffuseColor.setValue(0.5f , 0.8f , 0.8f);
  mSelected = true;
}

void Tendon::deselect()
{
  std::list<TendonInsertionPoint*>::iterator insPt;
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    SoMaterial* mat = (*insPt)->getIVInsertionMaterial();
    if ((*insPt)->isPermanent()) mat->diffuseColor.setValue(0.7f,0.2f,0.2f);
    else mat->diffuseColor.setValue(0.5f,0.5f,0.5f);
    if (insPt!=mInsPointList.begin())
    {
      mat = (*insPt)->getIVConnectorMaterial();
      mat->diffuseColor.setValue(0.5f,0.5f,0.5f);
    }
  }
  mIVForceIndMaterial->diffuseColor.setValue(0.2f , 0.4f , 0.4f);
  mSelected = false;
}

void Tendon::setVisible(bool v)
{
  /*one small issue: tendon will still intercept mouse clicks even it it's not visible on the screen*/
  mVisible = v;
  if (v) {
    updateGeometry();
    mIVVisibleToggle->style  = SoDrawStyle::FILLED;
    if (mForcesVisible) mIVForceIndToggle->style = SoDrawStyle::FILLED;
  } else {
    mIVVisibleToggle->style = SoDrawStyle::INVISIBLE;
    mIVForceIndToggle->style = SoDrawStyle::INVISIBLE;
  }
}

void Tendon::setForcesVisible(bool v)
{
  if (v && !mVisible) return;
  mForcesVisible = v;
  if (v) {
    updateForceIndicators();
    mIVForceIndToggle->style = SoDrawStyle::FILLED;
  } else {
    mIVForceIndToggle->style = SoDrawStyle::INVISIBLE;
  }
}

void Tendon::addInsertionPoint(int chain, int link, vec3 point, double mu, bool isPerm)
{
  insertInsertionPoint( mInsPointList.end(), chain, link, point, mu, isPerm);
}

std::list<TendonInsertionPoint*>::iterator
Tendon::insertInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos, 
                             int chain, int link, vec3 point, double mu, bool isPerm)
{
  std::list<TendonInsertionPoint*>::iterator newInsPt;
  newInsPt = mInsPointList.insert( itPos, new TendonInsertionPoint(this,chain,link,point,mu,isPerm) );
  mIVRoot->addChild( (*newInsPt)->getIVInsertion() );
  mIVRoot->addChild( (*newInsPt)->getIVConnector() );
  return newInsPt;
}

int Tendon::getNumPermInsPoints() const
{
  std::list<TendonInsertionPoint*>::const_iterator it;
  int num = 0;
  for (it=mInsPointList.begin(); it!=mInsPointList.end(); it++)
  {
    if ((*it)->isPermanent()) num++;
  }
  return num;
}

TendonInsertionPoint* Tendon::getPermInsPoint(int n)
{
  std::list<TendonInsertionPoint*>::const_iterator it;
  int num = 0;
  for (it=mInsPointList.begin(); it!=mInsPointList.end(); it++)
  {
    if ((*it)->isPermanent() && num++ == n) return *it;
  }
  DBGA("Requested tendon insertion point not found");
  return NULL;
}

PROF_DECLARE(TENDON_REMOVE_INS_POINT);
std::list<TendonInsertionPoint*>::iterator 
Tendon::removeInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos)
{
  PROF_TIMER_FUNC(TENDON_REMOVE_INS_POINT);
  (*itPos)->removeAllGeometry();
  mIVRoot->removeChild( (*itPos)->getIVConnector() );
  mIVRoot->removeChild( (*itPos)->getIVInsertion() );
  if ( (*itPos)->isPermanent() ) DBGA("WARNING: removing a permanent insertion point!");
  delete *itPos;
  std::list<TendonInsertionPoint*>::iterator newIt = mInsPointList.erase( itPos );
  return newIt;
  /*should check if we are actually removing the first insertion point, because it would mean 
    there is a connector to remove also (the one of the "new" first insertion point */
}
void Tendon::applyForces()
{
  if (getTotalForce() <= 0) return;
  std::list<TendonInsertionPoint*>::iterator insPt;
  for (insPt=mInsPointList.begin(); insPt!=mInsPointList.end(); insPt++)
  {
    Link* link = (*insPt)->getAttachedLink();	
    /*convert insertion point location to world coordinates*/
    SbVec3f tmp = ( (*insPt)->getAttachPoint() ).toSbVec3f();
    position pos = position(tmp) * ( link->getTran() );
    
    /*insertion point force is already stored in world coordinates*/
    vec3 force = (*insPt)->mInsertionForce; 
    link->addForceAtPos( force , pos );
  }
}

bool Tendon::loadFromXml(const TiXmlElement *root)
{
  QString name = root->Attribute("name");
  if(name.isNull()){
    DBGA("Tendon name undefined");
    return false;
  }
  setName(name);

  double stiffness;
  if (!getDouble(root, "stiffness", stiffness)) stiffness = 0.0;
  setStiffness(stiffness * 1.0e6);

  double defaultRestLength;
  if (!getDouble(root, "restLength", defaultRestLength)) mDefaultRestLength = -1.0;
  else mDefaultRestLength = defaultRestLength;

  double preTensionLength;
  if (!getDouble(root, "preTensionLength", preTensionLength)) mPreTensionLength = -1.0;
  else mPreTensionLength = preTensionLength;
    
  int nrInsPoints=countXmlElements(root, "insertionPoint");
  if (nrInsPoints<2){
    DBGA("Incorrect number of Ins Points");
    return false;
  }

  std::list<const TiXmlElement*> elementList2 = findAllXmlElements(root, "insertionPoint");  
  std::list<const TiXmlElement*>::iterator p2;
  int j;
  for (p2 = elementList2.begin(), j=0; p2!=elementList2.end(); p2++,j++) {
    int chain;
    if(!getInt(*p2,"chain",chain)){
      DBGA("Failed to read chain on ins point"<<j);
      return false;
    }
    int link;
    if(!getInt(*p2,"link",link)){
      DBGA("Failed to read link on ins point"<<j);
      return false;
    }
    const TiXmlElement* element = findXmlElement(*p2,"position");
    if(!element){
      DBGA("Failed to read position on ins point"<<j);
      return false;
    }
    vec3 position;
    if(!getPosition(element,position)){
      DBGA("Failed to read position on ins point"<<j);
      return false;      
    }
    double friction;
    if (!getDouble(*p2, "friction", friction)) friction = 0.0;
    addInsertionPoint(chain,link,position,friction,true);
  }  
  return true;
}

TendonWrapper::TendonWrapper(Robot *myOwner) : 
  mMu(0.0),
  owner(myOwner)
{
}

/*! Use this function to get the link the tendon wrapper is attached 
	to. If chain is -1 this means it is attached to the base of the 
	robot and this function will handle this correctly.
*/
Link* TendonWrapper::getAttachedLink()
{
  if (attachChainNr == -1 || attachLinkNr == -1){
    //insertion point is attached to robot base
    return owner->getBase();
  } else {
    return owner->getChain(attachChainNr)->getLink(attachLinkNr);
  }
}

void TendonWrapper::createGeometry()
{
  IVWrapper = new SoSeparator;
  IVWrapperMaterial = new SoMaterial;
  IVWrapperTran = new SoTransform;
  IVWrapperGeom = new SoCylinder;
  
  /*draw wrappers in wireframe*/
  SoDrawStyle *ds = new SoDrawStyle; 
  IVWrapper->addChild(ds);
  ds->style = SoDrawStyle::LINES;
  //ds->setOverride(TRUE);
  
  /*insert a pointer to the transform of the link this wrapper is attached to
    like this, we don't have to worry about updating anything.
    One problem: cleanup. This reference might prevent link's tranform IV node from being deleted when it should be*/
  IVWrapper->addChild( getAttachedLink()->getIVTran() );
  
  /* insertion point's location relative to link's origin and align axis with wrapper orientation*/
  IVWrapper->addChild(IVWrapperTran);
  
  /*could share material between all wrappers, and not declare it locally*/
  IVWrapper->addChild(IVWrapperMaterial);
  IVWrapper->addChild(IVWrapperGeom);
  IVWrapperMaterial->diffuseColor.setValue(0.7f , 0.1f , 0.1f);
}

void TendonWrapper::updateGeometry()
{
  IVWrapperTran->pointAt(location.toSbVec3f(),location.toSbVec3f() + orientation.toSbVec3f());
  //neg 90 x
  rotateSoTransform(IVWrapperTran, vec3(1.0, 0.0, 0.0), -1.5707);
  IVWrapperGeom->radius=radius;
  IVWrapperGeom->height=length;  
}

void TendonWrapper::setLocation(vec3 loc)
{
  location = loc;
  updateGeometry();
}

void TendonWrapper::setOrientation(vec3 orient)
{
  orientation = orient;
  updateGeometry();
}

void TendonWrapper::setRadius(double r)
{
  radius = r;
  updateGeometry();
}

bool TendonWrapper::wrappingSide(QString tendonName, vec3 &direction)
{
  for (size_t i=0; i<mWrappingSideVec.size(); i++) {
    if (mWrappingSideVec[i].mTendonName == tendonName) {
      direction = mWrappingSideVec[i].mDirection;
      return true;
    }    
  }
  return false;
}

bool TendonWrapper::loadFromXml(const TiXmlElement* root)
{
  int chain;
  if(!getInt(root,"chain",chain)){
    DBGA("Failed to read chain on tendon wrapper");
    return false;
  }
  int link;
  if(!getInt(root,"link",link)){
    DBGA("Failed to read link on tendon wrapper");
    return false;
  }
  double mu;
  if (!getDouble(root, "friction", mu)) mu = 0.0;
  const TiXmlElement* element = findXmlElement(root,"position");
  if(!element){
    DBGA("Failed to read position on tendon wrapper");
    return false;
  }
  vec3 loc;
  if(!getPosition(element,loc)){
    DBGA("Failed to read position tendon wrapper");
    return false;
  }
  element = findXmlElement(root,"orientation");
  if(!element){
    DBGA("Failed to read orientation on tendon wrapper");
    return false;
  }
  vec3 ort;
  if(!getPosition(element,ort)){
    DBGA("Failed to read orientation tendon wrapper");
    return false;
  }
  double rad;
  if(!getDouble(root,"radius",rad)){
    DBGA("Failed to read radius on tendon wrapper");
    return false;
  }
  double len;
  if(!getDouble(root,"length",len)){
    DBGA("Tendon wrapper with no specified length, using default");
    len = 10.0;
  }
  attachChainNr = chain;
  attachLinkNr = link;
  location = loc;
  orientation = ort;
  radius = rad;
  length = len;
  mMu = mu;

  //read list of exempt tendons
  std::list<const TiXmlElement*> elementList = findAllXmlElements(root,"exemption");
  std::list<const TiXmlElement*>::iterator p;
  for (p = elementList.begin(); p != elementList.end(); p++) {
    QString name = (*p)->Attribute("tendon");
    if(name.isNull()){
      DBGA("Warning: tendon name undefined for wrapper exemption");
      continue;
    }
    mExemptList.push_back(name);
  }

  //read list of tendon wrapping wrapping sides
  elementList = findAllXmlElements(root,"wrapping_side");
  for (p = elementList.begin(); p != elementList.end(); p++) {
    QString name = (*p)->Attribute("tendon");
    if(name.isNull()){
      DBGA("Warning: tendon name undefined for wrapping side");
      continue;
    }
    const TiXmlElement* element = findXmlElement(*p,"orientation");
    if(!element){ DBGA("Failed to find orientation on tendon wrapper"); continue;}
    vec3 side_dir;
    if (!getPosition(element, side_dir)) { DBGA("Warning: incorrect wrapping side direction"); continue;}
    WrappingSide side;
    side.mTendonName = name;
    side.mDirection = side_dir;
    mWrappingSideVec.push_back(side);
  }
  
  return true;
}

bool TendonWrapper::isExempt(QString name)
{
  for (std::list<QString>::iterator it=mExemptList.begin(); it!=mExemptList.end(); it++) {
    if ( *it == name ) return true;
  }
  return false;
}

HumanHand::HumanHand(World *w,const char *name) : Hand(w,name)
{
}

/*! First calls the super to load the hand like a regular one (all 
  kinematic chains, links etc). Then loads tendon information from
  the same file.
*/
int HumanHand::loadFromXml(const TiXmlElement* root, QString rootPath)
{
  if ( Hand::loadFromXml(root, rootPath) != SUCCESS) return FAILURE;
  std::list<const TiXmlElement*> elementList = findAllXmlElements(root,"tendon");
  std::list<const TiXmlElement*>::iterator p;
  int i;
  for (p = elementList.begin(), i=0; p != elementList.end(); p++,i++) {
    Tendon* newTendon = new Tendon(this);
    if (!newTendon->loadFromXml(*p)) {
      DBGA("Failed to read tendon " << i);
      delete newTendon;
    } else {
      mTendonVec.push_back(newTendon);
    }
  }
  for (size_t t=0; t<mTendonVec.size(); t++)
  {
    IVRoot->addChild( mTendonVec[t]->getIVRoot() );
    DBGA("Tendon "<< mTendonVec[t]->getName().ascii()<<" read and added");
  }
  
  elementList = findAllXmlElements(root,"tendonWrapper");
  for (p = elementList.begin(), i=0; p != elementList.end(); p++,i++){
    TendonWrapper* newTW = new TendonWrapper(this);
    if (!newTW->loadFromXml(*p)) {
      DBGA("Failed to load tendon wrapper " << i);
      delete newTW;
    } else {
      newTW->createGeometry();
      IVRoot->addChild( newTW->getIVRoot() );	
      newTW->updateGeometry();
      DBGA("TendonWrapper " << i << " geometry added");
      mTendonWrapperVec.push_back(newTW);
    }
  }
  updateTendonGeometry();
  setRestPosition();
  return SUCCESS;
}

void HumanHand::updateTendonGeometry()
{
  for (size_t i=0; i<mTendonVec.size(); i++) {
    mTendonVec[i]->updateGeometry();
  }
}

void HumanHand::applyTendonForces()
{
  for (size_t i=0; i<mTendonVec.size(); i++) {
    mTendonVec[i]->applyForces();
  }
}

/*! Applies tendon forces; for now both active and passive.*/
void HumanHand::DOFController(double)
{
  applyTendonForces();
}

/*! Creates the individual force matrices for the "fake contacts" at tendon location
  insertion points, which are only allowed to apply force in the normal direction 
  (along local z). This should be integrated with the matrices that do the same thing 
  for real contacts.
*/
Matrix insPtForceBlockMatrix(unsigned int numInsPoints)
{
  if (!numInsPoints) return Matrix(0,0);
  Matrix D(Matrix::ZEROES<Matrix>(6*numInsPoints, numInsPoints));
  for(unsigned int i=0; i<numInsPoints; i++)
  {
    D.elem(6*i + 2, i) = 1.0;
  }
  return D;
}

PROF_DECLARE(HH_TENDON_EQUILIBRIUM);
int HumanHand::tendonEquilibrium(const std::set<size_t> &activeTendons,
                                 const std::set<size_t> &passiveTendons,
                                 bool compute_tendon_forces,
                                 std::vector<double> &activeTendonForces,
                                 std::vector<double> &jointResiduals,
                                 double& unbalanced_magnitude,
                                 bool useJointSprings)
{
  PROF_TIMER_FUNC(HH_TENDON_EQUILIBRIUM);
  std::list<Joint*> joints;
  for(int c=0; c<getNumChains(); c++) 
  {
    std::list<Joint*> chainJoints = getChain(c)->getJoints();
    joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
  }

  if (activeTendons.empty() && passiveTendons.empty()) 
  {
    DBGA("Need either active or passive tendons (or both) for analysis");
    return -1;
  }
  
  Matrix LeftHand(Matrix::ZEROES<Matrix>(joints.size(), activeTendons.size()));
  Matrix RightHand(Matrix::ZEROES<Matrix>(joints.size(), 1));
  for (size_t i=0; i<mTendonVec.size(); i++)
  {    
    std::list< std::pair<transf, Link*> > insPointLinkTrans;
    mTendonVec[i]->getInsertionPointLinkTransforms(insPointLinkTrans);
    Matrix J( grasp->contactJacobian(joints, insPointLinkTrans) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( insPointLinkTrans.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );

    std::vector<double> magnitudes;
    mTendonVec[i]->getInsertionPointForceMagnitudes(magnitudes);
    Matrix M(&magnitudes[0], magnitudes.size(), 1, true);
    assert(M.rows() == JTD.cols());
    Matrix JTDM( JTD.rows(), M.cols());
    matrixMultiply( JTD, M, JTDM);

    if (activeTendons.find(i) != activeTendons.end())
    {
      assert( JTDM.rows() == LeftHand.rows());
      assert( JTDM.cols() == 1);
      LeftHand.copySubMatrix(0, i, JTDM);
    }
    else if (passiveTendons.find(i) != passiveTendons.end())
    {
      JTDM.multiply( mTendonVec[i]->getPassiveForce() );
      matrixAdd(RightHand, JTDM, RightHand);
    }
    else
    {
      DBGP("Warning: tendon " << i << " is not active or passive");
    }
  }

  //add the contribution of the joint springs
  if (useJointSprings)
  {
    std::list<Joint*>::iterator jit;
    size_t j_num;
    for (jit=joints.begin(),j_num=0; jit!=joints.end(); jit++,j_num++)
    {
      RightHand.elem(j_num,0) = RightHand.elem(j_num,0) - (*jit)->getSpringForce();
    }
  }

  Matrix x(LeftHand.cols(), 1);

  if (compute_tendon_forces)
  {
    //compute optimal tendon forces
    //scale down right hand for numerical reasons
    double scale = std::max(1.0, RightHand.absMax());
    RightHand.multiply(1.0/scale);
    RightHand.multiply(-1.0);  
    if ( linearSolveSVD(LeftHand, RightHand, x) != 0 )
    {
      DBGA("SVD decomposition solving failed");
      return -1;
    }
    x.multiply(scale);
    RightHand.multiply(-1.0);
    RightHand.multiply(scale);
  }
  else
  {
    //use passed in tendon forces
    if ((int)activeTendonForces.size() != x.rows())
    {
      DBGA("Incorrect active tendon forces passed in");
      return -1;
    }
    int t=0;
    for (size_t i=0; i<mTendonVec.size(); i++)
    {    
      if (activeTendons.find(i) != activeTendons.end())
      {
        x.elem(t,0) = activeTendonForces.at(t);
        t++;
      }
    }
  }

  //compute the residual joint torques
  Matrix tau(LeftHand.rows(), 1);
  matrixMultiply(LeftHand, x, tau);
  matrixAdd(tau, RightHand, tau);
  tau.getData(&jointResiduals);  
  DBGP("Joint residuals: " << jointResiduals[0] << " " <<jointResiduals[1]);
  unbalanced_magnitude = tau.fnorm();
  
  //pass back the active forces on the tendons
  if (compute_tendon_forces)
  {
    activeTendonForces.resize( activeTendons.size(), 0.0 );
    int t=0;
    for (size_t i=0; i<mTendonVec.size(); i++)
    {    
      if (activeTendons.find(i) != activeTendons.end())
      {
        activeTendonForces.at(t) = x.elem(t,0);
        t++;
      }
    }
  }

  return 0;
}

/*! Computes the following:
    JTc c = tau
    where Jc is the Jacobian of the contacts
    Returns tau.
*/
int HumanHand::contactTorques(std::list<Contact*> contacts,
                              std::vector<double> &jointTorques)
{
  std::list<Joint*> joints;
  for(int c=0; c<getNumChains(); c++) 
  {
    std::list<Joint*> chainJoints = getChain(c)->getJoints();
    joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
  }
  Matrix RightHand(joints.size(), 1);
  {
    Matrix J( grasp->contactJacobian(joints, contacts) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( contacts.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );

    std::vector<double> magnitudes;
    //for now, all contacts apply equal force
    //virtual contact normal points outwards, so use negative magnitude
    magnitudes.resize(contacts.size(), -1.0);
    Matrix M(&magnitudes[0], magnitudes.size(), 1, true);
    assert(M.rows() == JTD.cols());
    Matrix JTDM( JTD.rows(), M.cols());
    matrixMultiply( JTD, M, JTDM);

    RightHand.copyMatrix(JTDM);    
  }
  //assume we were using newtons to begin with
  double scale = 1.0e6;
  RightHand.multiply(-1.0);
  jointTorques.resize(RightHand.rows());
  for (size_t i=0; i<jointTorques.size(); i++)
  {
    jointTorques[i] = RightHand.elem(i,0) * scale;
  }
  return 0;
}

/*! Solves:
  JTt f = JTc c
  where Jt is the Jacobian of tendon insertion points and Jc is the Jacobian of contacts.
  Assumes c = [1 .. 1]
  Returns the f that minimizes error norm (unbalanced magnitude).
 */
PROF_DECLARE(HH_CONTACT_EQUILIBRIUM);
int HumanHand::contactEquilibrium(std::list<Contact*> contacts,
                                  const std::set<size_t> &activeTendons,
                                  std::vector<double> &activeTendonForces,
                                  double &unbalanced_magnitude)
{
  PROF_TIMER_FUNC(HH_CONTACT_EQUILIBRIUM);
  std::list<Joint*> joints;
  for(int c=0; c<getNumChains(); c++) 
  {
    std::list<Joint*> chainJoints = getChain(c)->getJoints();
    joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
  }

  activeTendonForces.resize( activeTendons.size(), 0.0 );
  if (activeTendons.empty()) 
  {
    DBGA("Need active tendons for analysis");
    return -1;
  }
  if (contacts.empty())
  {
    DBGA("Contact list empty for contact equilibrium");
    return -1;
  }
  
  Matrix LeftHand(joints.size(), activeTendons.size());
  for (size_t i=0; i<mTendonVec.size(); i++)
  {    
    if (activeTendons.find(i) == activeTendons.end()) continue;

    std::list< std::pair<transf, Link*> > insPointLinkTrans;
    mTendonVec[i]->getInsertionPointLinkTransforms(insPointLinkTrans);
    Matrix J( grasp->contactJacobian(joints, insPointLinkTrans) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( insPointLinkTrans.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );

    std::vector<double> magnitudes;
    mTendonVec[i]->getInsertionPointForceMagnitudes(magnitudes);
    Matrix M(&magnitudes[0], magnitudes.size(), 1, true);
    assert(M.rows() == JTD.cols());
    Matrix JTDM( JTD.rows(), M.cols());
    matrixMultiply( JTD, M, JTDM);

    assert( JTDM.rows() == LeftHand.rows());
    assert( JTDM.cols() == 1);
    LeftHand.copySubMatrix(0, i, JTDM);
  }

  Matrix RightHand(joints.size(), 1);
  {
    Matrix J( grasp->contactJacobian(joints, contacts) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( contacts.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );

    std::vector<double> magnitudes;
    //for now, all contacts apply equal force
    //virtual contact normal points outwards, so use negative magnitude
    magnitudes.resize(contacts.size(), -1.0);
    Matrix M(&magnitudes[0], magnitudes.size(), 1, true);
    assert(M.rows() == JTD.cols());
    Matrix JTDM( JTD.rows(), M.cols());
    matrixMultiply( JTD, M, JTDM);

    RightHand.copyMatrix(JTDM);    
  }
  //assume we were using newtons to begin with
  double scale = 1.0e6;
  RightHand.multiply(-1.0);
  
  DBGA("Joint torques: " << RightHand.elem(0,0) << " " << RightHand.elem(1,0));

  Matrix x(LeftHand.cols(), 1);
  if ( linearSolveSVD(LeftHand, RightHand, x) != 0 )
  {
    DBGA("SVD decomposition solving failed");
    return -1;
  }
  
  //compute the residual joint torques
  Matrix tau(LeftHand.rows(), 1);
  matrixMultiply(LeftHand, x, tau);
  RightHand.multiply(-1.0);
  matrixAdd(tau, RightHand, tau);
  //DBGA("Residual joint torques:");
  //tau.print();
  unbalanced_magnitude = tau.fnorm()*scale;

  //scale result back 
  x.multiply(scale);
  //compute the active forces on the tendon
  int t=0;
  for (size_t i=0; i<mTendonVec.size(); i++)
  {    
    if (activeTendons.find(i) != activeTendons.end())
    {
      //DBGA("Tendon active force: " << x.elem(t,0) );
      activeTendonForces.at(t) = x.elem(t,0);
      t++;
    }
  }

  //DBGA("Unbalanced magnitude: " << unbalanced_magnitude);
  return 0;
}

/*! Solves:
  JTc c = JTt f
  where Jt is the Jacobian of tendon insertion points and Jc is the Jacobian of contacts.
  Uses passed in tendon forces for f
  Returns the c that minimizes error norm (unbalanced magnitude).
 */
int HumanHand::contactForcesFromTendonForces(std::list<Contact*> contacts,
                                             std::vector<double> &contactForces,
                                             const std::set<size_t> &activeTendons,
                                             const std::vector<double> &activeTendonForces)
{
  std::list<Joint*> joints;
  for(int c=0; c<getNumChains(); c++) 
  {
    std::list<Joint*> chainJoints = getChain(c)->getJoints();
    joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
  }

  if (activeTendons.empty()) 
  {
    DBGA("Need active tendons for analysis");
    return -1;
  }
  if (contacts.empty())
  {
    DBGA("Contact list empty for contact equilibrium");
    return -1;
  }
  
  Matrix LeftHand(joints.size(), activeTendons.size());
  int t=0;
  for (size_t i=0; i<mTendonVec.size(); i++)
  {    
    if (activeTendons.find(i) == activeTendons.end()) continue;

    std::list< std::pair<transf, Link*> > insPointLinkTrans;
    mTendonVec[i]->getInsertionPointLinkTransforms(insPointLinkTrans);
    Matrix J( grasp->contactJacobian(joints, insPointLinkTrans) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( insPointLinkTrans.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );

    std::vector<double> magnitudes;
    mTendonVec[i]->getInsertionPointForceMagnitudes(magnitudes);
    Matrix M(&magnitudes[0], magnitudes.size(), 1, true);
    assert(M.rows() == JTD.cols());
    Matrix JTDM( JTD.rows(), M.cols());
    matrixMultiply( JTD, M, JTDM);

    assert( JTDM.rows() == LeftHand.rows());
    assert( JTDM.cols() == 1);
    LeftHand.copySubMatrix(0, t, JTDM);
    t++;
  }

  //use passed in tendon forces
  Matrix x(LeftHand.cols(), 1);
  if ((int)activeTendonForces.size() != x.rows())
  {
    DBGA("Incorrect active tendon forces passed in");
    return -1;
  }
  t=0;
  for (size_t i=0; i<mTendonVec.size(); i++)
  {    
    if (activeTendons.find(i) != activeTendons.end())
    {
      x.elem(t,0) = activeTendonForces.at(t);
      t++;
    }
  }
  //compute the joint torques
  Matrix tau(LeftHand.rows(), 1);
  matrixMultiply(LeftHand, x, tau);
  DBGP("Joint torques: " << tau.elem(0,0) << " " << tau.elem(1,0));

  Matrix RightHand(joints.size(), contacts.size());
  {
    Matrix J( grasp->contactJacobian(joints, contacts) );
    Matrix JTran( J.transposed() );
    Matrix D( insPtForceBlockMatrix( contacts.size() ) );
    Matrix JTD( JTran.rows(), D.cols() );
    matrixMultiply( JTran, D, JTD );
    RightHand.copyMatrix(JTD);    
  }

  //DBGP("Contact jac:\n" << RightHand.elem(0,0) << "\n" <<  RightHand.elem(1,0) );
  //DBGP("Contact jac:\n" << RightHand.elem(0,0) << " " << RightHand.elem(0,1) << "\n" << 
  //     RightHand.elem(1,0) << " " << RightHand.elem(1,1));

  double scale = std::max(1.0, tau.absMax());
  tau.multiply(1.0/scale);

  DBGP("Joint torques scaled: " << tau.elem(0,0) << " " << tau.elem(1,0));

  Matrix c(RightHand.cols(), 1);
  if ( linearSolveSVD(RightHand, tau, c) != 0 )
  {
    DBGA("SVD decomposition solving failed");
    return -1;
  }
  
  /*
  Matrix c(tau);
  if ( triangularSolve(RightHand, c) != 0 )
  {
    DBGA("Triangular solving failed");
    return -1;
  }
  */

  DBGP("Contact forces computed for " << contacts.size() << "contacts");
  Matrix test(RightHand.rows(),1);
  matrixMultiply(RightHand, c, test);
  test.multiply(-1);
  matrixAdd(test, tau, test);
  if (test.fnorm() > 1.0e-5) {DBGA("Warning: norm of error " << test.fnorm());}

  c.multiply(scale);
  contactForces.resize( contacts.size() );
  for (size_t i=0; i<contacts.size(); i++)
  {
    contactForces[i] = c.elem(i,0);
  }
  
  return 0;
}

bool HumanHand::insPointInsideWrapper()
{
  for(size_t i=0; i<mTendonVec.size(); i++)
  {
    if (mTendonVec[i]->insPointInsideWrapper()) 
    {
      //std::cerr << " on tendon " << i << "\n";
      return true;
    }
  }
  return false;
}

