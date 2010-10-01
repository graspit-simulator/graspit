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
#include "tinyxml.h"
#include "debug.h"
#define WRAPPER_TOLERANCE 0.98
 
/*! Given two line segments, P1-P2 and P3-P4, returns the line segment 
	Pa-Pb that is the shortest route between them. Calculates also the 
	values of \a mua and \a mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Returns FALSE if no solution exists.
   adapted from http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
*/
int LineLineIntersect(vec3 p1,vec3 p2,vec3 p3,vec3 p4,vec3 *pa,vec3 *pb,double *mua, double *mub)
{
   vec3 p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;
   double EPS = 1.0e-5;

   p13.x() = p1.x() - p3.x();
   p13.y() = p1.y() - p3.y();
   p13.z() = p1.z() - p3.z();
   p43.x() = p4.x() - p3.x();
   p43.y() = p4.y() - p3.y();
   p43.z() = p4.z() - p3.z();
   if ( fabs(p43.x())  < EPS && fabs(p43.y())  < EPS && fabs(p43.z())  < EPS )
      return false;

   p21.x() = p2.x() - p1.x();
   p21.y() = p2.y() - p1.y();
   p21.z() = p2.z() - p1.z();
   if ( fabs(p21.x())  < EPS && fabs(p21.y())  < EPS && fabs(p21.z())  < EPS )
      return false;

   d1343 = p13.x() * p43.x() + p13.y() * p43.y() + p13.z() * p43.z();
   d4321 = p43.x() * p21.x() + p43.y() * p21.y() + p43.z() * p21.z();
   d1321 = p13.x() * p21.x() + p13.y() * p21.y() + p13.z() * p21.z();
   d4343 = p43.x() * p43.x() + p43.y() * p43.y() + p43.z() * p43.z();
   d2121 = p21.x() * p21.x() + p21.y() * p21.y() + p21.z() * p21.z();

   denom = d2121 * d4343 - d4321 * d4321;
   if ( fabs(denom) < EPS )
      return false;
   numer = d1343 * d4321 - d1321 * d4343;

   *mua = numer / denom;
   *mub = (d1343 + d4321 * (*mua)) / d4343;

   pa->x() = p1.x() + (*mua) * p21.x();
   pa->y() = p1.y() + (*mua) * p21.y();
   pa->z() = p1.z() + (*mua) * p21.z();
   pb->x() = p3.x() + (*mub) * p43.x();
   pb->y() = p3.y() + (*mub) * p43.y();
   pb->z() = p3.z() + (*mub) * p43.z();

   return true;
}
TendonInsertionPoint::TendonInsertionPoint(Tendon *myOwner,int chain,int link ,vec3 point, bool isPerm)
{
	owner=myOwner;
	attachChainNr=chain;
	attachLinkNr=link;
	attachPoint=point;
	permanent = isPerm; //true is default value in declaration
}
	
/*! Use this function to get the link the insertion point is 
	attached to. If chain is -1 this means it is attached to 
	the base of the robot this function will handle this 
	correctly.
*/
Link* TendonInsertionPoint::getAttachedLink()
{

	if (attachChainNr == -1 || attachLinkNr == -1) {
		// insertion point is attached to robot base
		return owner->getRobot()->getBase();
	} else {
		return owner->getRobot()->getChain(attachChainNr)->getLink(attachLinkNr);
	}
}

SbVec3f TendonInsertionPoint::getWorldPosition()
{
	position worldPos;
	worldPos = position(attachPoint.toSbVec3f()) * ( getAttachedLink()->getTran() );
	return worldPos.toSbVec3f();
}

void TendonInsertionPoint::createInsertionGeometry()
{
	SoTransform *linkIVTran;

	IVInsertion = new SoSeparator;
	IVInsertionMaterial = new SoMaterial;
	IVInsertionTran = new SoTransform;
	IVInsertionGeom = new SoSphere;

	/*insert a pointer to the transform of the link this insertion point is attached to
	like this, we don't have to worry about updating anything.
	One problem: cleanup. This reference might prevent link's tranform IV node from being deleted when it should be
	*/
	linkIVTran = getAttachedLink()->getIVTran();
	IVInsertion->addChild(linkIVTran);

	/* insertion point's location relative to link's origin*/
	IVInsertionTran->translation.setValue(attachPoint.x() , attachPoint.y() , attachPoint.z() );
	IVInsertion->addChild(IVInsertionTran);

	if ( isPermanent() )
	{
		IVInsertionMaterial->diffuseColor.setValue( (float)0.7 , (float)0.2 , (float)0.2);
		IVInsertionGeom->radius=(float)1.5;
	}
	else
	{
		if (owner->isSelected())
			IVInsertionMaterial->diffuseColor.setValue( (float)1.0 , (float)0.5 , (float)0.5);
		else
			IVInsertionMaterial->diffuseColor.setValue( (float)0.5 , (float)0.5 , (float)0.5);
		IVInsertionGeom->radius=(float)0.8;
	}
	IVInsertion->addChild(IVInsertionMaterial);
	IVInsertion->addChild(IVInsertionGeom);
}

void TendonInsertionPoint::createConnectorGeometry()
{
	IVConnector = new SoSeparator;
	IVConnectorTran = new SoTransform;
	IVConnectorMaterial = new SoMaterial;
	IVConnectorGeom = new SoCylinder;

	if ( owner->isSelected() )
		IVConnectorMaterial->diffuseColor.setValue(1.0 , 0.5 , 0.5);
	else
		IVConnectorMaterial->diffuseColor.setValue(0.5 , 0.5 , 0.5);

	IVConnector->addChild( IVConnectorTran );
	IVConnector->addChild( IVConnectorMaterial );
	IVConnector->addChild( IVConnectorGeom);
}

void TendonInsertionPoint::removeAllGeometry()
{
	IVConnector->removeAllChildren();
	IVInsertion->removeAllChildren();
}

Tendon::Tendon(Robot *myOwner)
{
	nrInsPoints=0;
	owner = myOwner;
	activeForce = 0;
	IVRoot = new SoSeparator;
	IVVisibleToggle = new SoDrawStyle();
	IVVisibleToggle->style = SoDrawStyle::FILLED;
	IVRoot->addChild(IVVisibleToggle);
	tendonName = "unnamed";
	visible = true;
	selected = false;
	restLength = 0;
	currentLength = 0;
	applyPassiveForce = true;
}

void Tendon::setActiveForce(float f)
{
	if (f>=0)
		activeForce = f;
	else
		activeForce = 0;
	updateInsertionForces();
}
void Tendon::setPassiveForce(float f)
{
	if (f>=0)
		passiveForce = f;
	else
		passiveForce = 0;
	updateInsertionForces();
}

void Tendon::removeWrapperIntersections()
{
	int j;
	SbVec3f pPrev,pCur,pNext;
	position tmpPos;
	vec3 dPrev,dRes,P3,P4,Pa,Pb;
	double mua,mub;
	Link *link;
	bool needed;

	std::list<TendonInsertionPoint*>::iterator prevInsPt, insPt, nextInsPt;

	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
	{
		nextInsPt = insPt;
		prevInsPt = insPt;
		nextInsPt ++;

		if ( !(*insPt)->isPermanent() && insPt!=insPointList.begin() && nextInsPt!=insPointList.end() )
		{
			prevInsPt--;

			pPrev = (*prevInsPt)->getWorldPosition();
			pCur = (*insPt)->getWorldPosition();
			pNext = (*nextInsPt)->getWorldPosition();

			needed = false;
			for (j=0; j< ((HumanHand*)getRobot())->nrTendonWrappers; j++)
			{
				//two points along axis of tendon wrapper
				P3 = ((HumanHand*)getRobot())->getTendonWrapper(j)->location;
				P4 = P3 + ((HumanHand*)getRobot())->getTendonWrapper(j)->orientation;
				//convert them to world coordinates 
				link = ((HumanHand*)getRobot())->getTendonWrapper(j)->getAttachedLink();
				tmpPos = position (P3.toSbVec3f()) * ( link->getTran());
				P3 = vec3 ( tmpPos.toSbVec3f() );
				tmpPos = position (P4.toSbVec3f()) * ( link->getTran());
				P4 = vec3 ( tmpPos.toSbVec3f() );

				LineLineIntersect( vec3(pPrev) , vec3(pNext) , P3,P4 , &Pa, &Pb, &mua, &mub);
				dPrev = Pa - Pb;
				/*careful: here we are using the exact radius, not a slightly smaller value*/
				/*changed my mind: we are*/
				if (dPrev.len() <  WRAPPER_TOLERANCE * ((HumanHand*)getRobot())->getTendonWrapper(j)->radius && 
					mua>0 && mua<1)
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
void Tendon::checkWrapperIntersections()
{
	int j;
	int chainNr,linkNr;
	SbVec3f pCur,pNext;
	position tmpPos;
	vec3 dPrev,dRes,P3,P4,Pa,Pb;
	double mua,mub;
	Link *link;

	std::list<TendonInsertionPoint*>::iterator prevInsPt, insPt, nextInsPt, newInsPt;

	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++) {
		nextInsPt = insPt;
		nextInsPt ++;
		pCur = (*insPt)->getWorldPosition();
		if (nextInsPt != insPointList.end() ) {
			pNext = (*nextInsPt)->getWorldPosition();
			for (j=0; j< ((HumanHand*)getRobot())->nrTendonWrappers; j++) {
				//two points along axis of tendon wrapper
				P3 = ((HumanHand*)getRobot())->getTendonWrapper(j)->location;
				P4 = P3 + ((HumanHand*)getRobot())->getTendonWrapper(j)->orientation;
				//convert them to world coordinates 
				link = ((HumanHand*)getRobot())->getTendonWrapper(j)->getAttachedLink();
				tmpPos = position (P3.toSbVec3f()) * ( link->getTran());
				P3 = vec3 ( tmpPos.toSbVec3f() );
				tmpPos = position (P4.toSbVec3f()) * ( link->getTran());
				P4 = vec3 ( tmpPos.toSbVec3f() );

				LineLineIntersect( vec3(pCur) , vec3(pNext) , P3,P4 , &Pa, &Pb, &mua, &mub);

				// check two things:
				//- if tendon is too close to wrapper
				//- if closest point actually falls between insertion points 
				//  (wrappers extends to infinity, we don't check that)
				//- WE SHOULD IN THE FUTURE! don't want one finger's tendon to wrap around another finger's wrapper
				//
				dPrev = Pa - Pb;
				if (dPrev.len() < WRAPPER_TOLERANCE * ((HumanHand*)getRobot())->getTendonWrapper(j)->radius 
					&& mua>0 && mua<1)
				{
					/* compute location of new insertion point - on cylinder edge */
					dPrev = normalise(dPrev);
					dRes = Pb + ( ((HumanHand*)getRobot())->getTendonWrapper(j)->radius ) * dPrev;
					/* transform it to coordinate system of wrapper */

					tmpPos = position ( dRes.toSbVec3f() ) * ( link->getTran().inverse() );

					chainNr = ((HumanHand*)getRobot())->getTendonWrapper(j)->getChainNr();
					linkNr = ((HumanHand*)getRobot())->getTendonWrapper(j)->getLinkNr();

					/*create new insertion point*/
					newInsPt = insertInsertionPoint( nextInsPt, chainNr, linkNr, vec3(tmpPos.toSbVec3f()), false );
				}
			}
		}
	}
}

/*!	Updates the geometry of the connectors between insertion points, which
	need to move together with the robot's links. However, some of them 
	depend on two links, so we can not use link transforms directly (like 
	we do for the geometry of the insertion points).Rather, this geometry 
	needs to be recomputed after every change in link status.
*/
void Tendon::updateGeometry()
{
	SbVec3f pPrev,pCur;
	position tmpPos;
	vec3 dPrev,dRes,c;
	float m;
	SoTransform *tran;
	SoCylinder *geom;

	SoTransform *neg90x = new SoTransform();
	neg90x->rotation.setValue(SbVec3f(1.0f,0.0f,0.0f),-1.5707f);

	/*first we wrap tendon around wrappers*/
	checkWrapperIntersections();

	/*and we remove unnecessary ones*/
	removeWrapperIntersections();

	/* we also compute the length of the tendon as the sum of connector lengths*/
	currentLength = 0;

	std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, newInsPt;

	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
	{
		prevInsPt = insPt;
		if (insPt!=insPointList.begin())
		{
			prevInsPt--;
			pPrev = (*prevInsPt)->getWorldPosition();
			pCur = (*insPt)->getWorldPosition();
		
			dPrev = vec3(pCur) - vec3(pPrev);
			/* distance between the two */
			m = dPrev.len();
			/* add it to tendon length */
			currentLength += m;

			/* midpoint between the two */
			c = vec3(pPrev) + dPrev*0.5;
			tran = (*insPt)->getIVConnectorTran();
			geom = (*insPt)->getIVConnectorGeom();
			geom->radius=(float)0.8;
			geom->height = m;
			tran->pointAt(c.toSbVec3f(),pPrev);
			tran->combineLeft(neg90x);
		}
	}

	/*after geometry has been updated, go ahead and update forces*/
	computeSimplePassiveForces();
	updateInsertionForces();
}

/*! Simple method for some passive force computation
	- assume inextensible tendon is attached to a muscle, 
	  therefore tendon elongation is actually muscle elongation
	- hard-code some values for muscle rest length and max. force 
	  (each muscle should actually have its own values)
	- use formula from Hill-type model (Tsang et al.) for computing 
	  muscle force
*/
void Tendon::computeSimplePassiveForces()
{
	/* numbers loosely inspired by Pollard and Gilbert, 2002 / Gonzales, Delp et al, 1997 */
	float muscleMaxForce=125*1.0e6; //convert from newtons!
	float muscleRestLength=70;
	if ( getExcursion() < 0 ) {
		//only apply passive force is muscle is elongated
		passiveForce = 0;
		return;
	}
	float elongation = ( muscleRestLength + getExcursion() ) / muscleRestLength;
	passiveForce = 2.77 * (elongation - 1) * (elongation - 1) * muscleMaxForce; 
}

/*! Given a level of active and passive forces MAGNITUDES this 
	computes what force is applied at each insertion point, and in 
	what DIRECTION.	Geometry needs to be up-to-date 
	(use updateGeometry() ).
*/
void Tendon::updateInsertionForces()
{
	SbVec3f pPrev,pCur,pNext;
	position tmpPos;
	vec3 dPrev,dNext,dRes,c;

	std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, nextInsPt, newInsPt;

	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
	{
		prevInsPt = insPt;
		nextInsPt = insPt;
		nextInsPt ++;

		if (insPt!=insPointList.begin())
		{
			prevInsPt--;
			pPrev = (*prevInsPt)->getWorldPosition();
		}

		pCur = (*insPt)->getWorldPosition();
		
		if (nextInsPt != insPointList.end() )
			pNext = (*nextInsPt)->getWorldPosition();
		
		/*compute resultant force on insertion point*/
		if (insPt == insPointList.begin())
		{
			/*first insertion point: no force at the moment*/
			(*insPt)->insertionForce=vec3(0,0,0);
		}
		else if ( nextInsPt != insPointList.end() )
		{
			/*middle insertion points: force is resultant of forces along connectors to pPrev and pNext*/
			dPrev = vec3(pPrev) - vec3(pCur);
			dNext = vec3(pNext) - vec3(pCur);
			dPrev = ( (float)1 / dPrev.len() ) * dPrev;
			dNext = ( (float)1 / dNext.len() ) * dNext;
			dRes = getTotalForce() * (dPrev + dNext);
			(*insPt)->insertionForce = dRes;
		}
		else
		{
			/*last insertion point: force is applied in direction to pPrev */
			dPrev = vec3(pPrev) - vec3(pCur);
			dPrev = ( (float)1 / dPrev.len() ) * dPrev;
			dRes = getTotalForce() * dPrev;
			(*insPt)->insertionForce = dRes;
		}
	}
}

void Tendon::setRestPosition()
{
	if (currentLength==0)
		printf("WARNING: setRestPosition called on tendon, but currentLength is zero!\n");
	restLength = currentLength;
}

void Tendon::select()
{
	SoMaterial *mat;
	std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, nextInsPt;
	selected = true;
	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
	{
		mat = (*insPt)->getIVInsertionMaterial();
		if ( (*insPt)->isPermanent() )
			mat->diffuseColor.setValue(1.0f,0.7f,0.7f);
		else
			mat->diffuseColor.setValue(1.0f,0.5f,0.5f);

		if (insPt!=insPointList.begin())
		{
			mat = (*insPt)->getIVConnectorMaterial();
			mat->diffuseColor.setValue(1.0f,0.5f,0.5f);
		}
	}
}

void Tendon::deselect()
{
	SoMaterial *mat;
	std::list<TendonInsertionPoint*>::iterator insPt, prevInsPt, nextInsPt;
	selected = false;
	for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
	{
		mat = (*insPt)->getIVInsertionMaterial();
		if ((*insPt)->isPermanent())
			mat->diffuseColor.setValue(0.7f,0.2f,0.2f);
		else
			mat->diffuseColor.setValue(0.5f,0.5f,0.5f);
		if (insPt!=insPointList.begin())
		{
			mat = (*insPt)->getIVConnectorMaterial();
			mat->diffuseColor.setValue(0.5f,0.5f,0.5f);
		}
	}
}

void Tendon::setVisible(bool v)
{
	/*one small issue: tendon will still intercept mouse clicks even it it's not visible on the screen*/
	visible = v;
	if (v) {
		IVVisibleToggle->style  = SoDrawStyle::FILLED;
	} else {
		IVVisibleToggle->style = SoDrawStyle::INVISIBLE;
	}
}

void Tendon::addInsertionPoint(int chain, int link, vec3 point, bool isPerm)
{
	TendonInsertionPoint* newInsPt = new TendonInsertionPoint(this,chain,link,point,isPerm);
	insPointList.push_back(newInsPt);
	nrInsPoints++;
	
	newInsPt->createInsertionGeometry();
	IVRoot->addChild( newInsPt->getIVInsertion() );
	if (nrInsPoints>1)
	{
		newInsPt->createConnectorGeometry();
		IVRoot->addChild( newInsPt->getIVConnector() );
	}
}

std::list<TendonInsertionPoint*>::iterator
	Tendon::insertInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos, 
									int chain, int link, vec3 point, bool isPerm)
{
	std::list<TendonInsertionPoint*>::iterator newInsPt;
	newInsPt = insPointList.insert( itPos,new TendonInsertionPoint(this,chain,link,point,isPerm) );
	nrInsPoints++;

	(*newInsPt)->createInsertionGeometry();
	IVRoot->addChild( (*newInsPt)->getIVInsertion() );
	if (newInsPt!=insPointList.begin())
	{
		(*newInsPt)->createConnectorGeometry();
		IVRoot->addChild( (*newInsPt)->getIVConnector() );
	}

	return newInsPt;
}

void Tendon::removeInsertionPoint(std::list<TendonInsertionPoint*>::iterator itPos)
{
	(*itPos)->removeAllGeometry();
	IVRoot->removeChild( (*itPos)->getIVConnector() );
	IVRoot->removeChild( (*itPos)->getIVInsertion() );
	if ( (*itPos)->isPermanent() )
		printf("WARNING: removing a permanent insertion point!\n");
	insPointList.erase( itPos );
	/*should check if we are actually removing the first insertion point, because it would mean 
		there is a connector to remove also (the one of the "new" first insertion point */
}
void Tendon::applyForces()
{
	Link *link;
	vec3 force;
	position pos;
	SbVec3f tmp;
	std::list<TendonInsertionPoint*>::iterator insPt;

	if (getTotalForce() > 0)
		for (insPt=insPointList.begin(); insPt!=insPointList.end(); insPt++)
		{
			link = (*insPt)->getAttachedLink();	
			/*convert insertion point location to world coordinates*/
			tmp = ( (*insPt)->attachPoint ).toSbVec3f();
			pos = position(tmp) * ( link->getTran() );

			/*insertion point force is already stored in world coordinates*/
			force = (*insPt)->insertionForce; 
			link->addForceAtPos( force , pos );
		}
}

TendonWrapper::TendonWrapper(Robot *myOwner, int chain, int link, vec3 loc, vec3 ort, double r)
{
	attachChainNr = chain;
	attachLinkNr = link;
	location = loc;
	orientation = ort;
	radius = r;
	owner = myOwner;
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
	SoTransform *linkIVTran;
	float geomHeight = 10;

	SoTransform *neg90x = new SoTransform();
	neg90x->rotation.setValue(SbVec3f(1.0f,0.0f,0.0f),-1.5707f);

	IVWrapper = new SoSeparator;
	IVWrapperMaterial = new SoMaterial;
	IVWrapperTran = new SoTransform;
	IVWrapperGeom = new SoCylinder;

	/*draw wrappers in wireframe*/
	SoDrawStyle *ds = new SoDrawStyle; 
	ds->style = SoDrawStyle::LINES;
	//ds->setOverride(TRUE);
	IVWrapper->addChild(ds);

	/*insert a pointer to the transform of the link this wrapper is attached to
	like this, we don't have to worry about updating anything.
	One problem: cleanup. This reference might prevent link's tranform IV node from being deleted when it should be*/
	linkIVTran = getAttachedLink()->getIVTran();
	IVWrapper->addChild(linkIVTran);

	/* insertion point's location relative to link's origin and align axis with wrapper orientation*/
	IVWrapperTran->pointAt(location.toSbVec3f(),orientation.toSbVec3f());
	IVWrapperTran->combineLeft(neg90x);
	IVWrapper->addChild(IVWrapperTran);

	/*could share material between all wrappers, and not declare it locally*/
	IVWrapperMaterial->diffuseColor.setValue(0.7f , 0.1f , 0.1f);
	IVWrapperGeom->radius=radius;
	IVWrapperGeom->height=geomHeight;
	IVWrapper->addChild(IVWrapperMaterial);
	IVWrapper->addChild(IVWrapperGeom);
}

HumanHand::HumanHand(World *w,const char *name) : Hand(w,name)
{
	nrTendons=0;
	nrTendonWrappers=0;
}

/*! First calls the super to load the hand like a regular one (all 
	kinematic chains, links etc). Then loads tendon information from
	the same file.
*/
int HumanHand::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	if ( Hand::loadFromXml(root, rootPath) != SUCCESS)
		return FAILURE;
	int i,j;
	QString ivdir,tendonName;
	QString line;
	const TiXmlElement* element;
	double rad;
	int nrInsPoints,chain,link;
	vec3 position,orientation;
	//load tendons from xml node
	nrTendons=countXmlElements(root,"tendon");
	if (nrTendons<0)
	{
		DBGA("Incorrect number of tendons");
		return FAILURE;
	}
	std::list<const TiXmlElement*> elementList = findAllXmlElements(root,"tendon");
	std::list<const TiXmlElement*> elementList2;
	tendonVec.resize(nrTendons, NULL);
	for (i=0; i<nrTendons; i++) {
		tendonVec[i]=new Tendon(this);
	}
	std::list<const TiXmlElement*>::iterator p, p2;

	for (p = elementList.begin(), i=0; p != elementList.end(); p++,i++) {
		tendonName = (*p)->Attribute("name");
		if(tendonName.isNull()){
			DBGA("Tendon Name undefined: tendon"<<i);
			return FAILURE;
		}
		nrInsPoints=countXmlElements(*p,"insertionPoint");
		if (nrInsPoints<2){
			DBGA("Incorrect number of Ins Points on tendon"<<i);
			return FAILURE;
		}
		delete tendonVec[i];
		tendonVec[i]=new Tendon(this);
		tendonVec[i]->setName(tendonName);
		elementList2 = findAllXmlElements(*p,"insertionPoint");
		for (p2 = elementList2.begin(), j=0; p2!=elementList2.end(); p2++,j++) {
			if(!getInt(*p2,"chain",chain)){
				DBGA("Failed to read chain on tendon"<<i<<"ins point"<<j);
				delete tendonVec[i];
				tendonVec[i] = new Tendon(this);
				return FAILURE;
			}
			if(!getInt(*p2,"link",link)){
				DBGA("Failed to read link on tendon"<<i<<"ins point"<<j);
				delete tendonVec[i];
				tendonVec[i] = new Tendon(this);
				return FAILURE;
			}
			element = findXmlElement(*p2,"position");
			if(!element){
				DBGA("Failed to read position on tendon"<<i<<"ins point"<<j);
				delete tendonVec[i];
				tendonVec[i] = new Tendon(this);
				return FAILURE;
			}
			if(!getPosition(element,position)){
				DBGA("Failed to read position tendon"<<i<<"ins point"<<j);
				delete tendonVec[i];
				tendonVec[i] = new Tendon(this);
				return FAILURE;
			
			}
			tendonVec[i]->addInsertionPoint(chain,link,position,true);
		}
	}
	for (i=0; i<nrTendons; i++)
	{
		IVRoot->addChild( tendonVec[i]->getIVRoot() );
		DBGA("Tendon "<< tendonVec[i]->getName().ascii()<<" read and added with "
		<<tendonVec[i]->nrInsPoints<<" insertion points");
	}

	//load tendon wrapper data from file
	nrTendonWrappers=countXmlElements(root,"tendonWrapper");
	if (nrTendons<0)
	{
		DBGA("Incorrect number of tendon wrappers");
		return FAILURE;
	}
	tendonWrapperVec.resize(nrTendonWrappers, NULL);
	elementList = findAllXmlElements(root,"tendonWrapper");
	for (i=0; i<nrTendonWrappers; i++)
		tendonWrapperVec[i]=new TendonWrapper(this,-1,-1,vec3(0,0,0),vec3(1,0,0),0);
	for (p = elementList.begin(), i=0; p != elementList.end(); p++,i++){
			if(!getInt(*p,"chain",chain)){
				DBGA("Failed to read chain on tendon wrapper"<<i);
				return FAILURE;
			}
			if(!getInt(*p,"link",link)){
				DBGA("Failed to read link on tendon wrapper"<<i);
				return FAILURE;
			}
			element = findXmlElement(*p,"position");
			if(!element){
				DBGA("Failed to read position on tendon wrapper"<<i);
				return FAILURE;
			}
			if(!getPosition(element,position)){
				DBGA("Failed to read position tendon wrapper"<<i);
				return FAILURE;
			}
			element = findXmlElement(*p,"orientation");
			if(!element){
				DBGA("Failed to read orientation on tendon wrapper"<<i);
				return FAILURE;
			}
			if(!getPosition(element,orientation)){
				DBGA("Failed to read orientation tendon wrapper"<<i);
				return FAILURE;
			}
			if(!getDouble(*p,"radius",rad)){
				DBGA("Failed to read radius on tendon wrapper"<<i);
				return FAILURE;
			}
			delete tendonWrapperVec[i];
			tendonWrapperVec[i]=new TendonWrapper(this,chain,link,position,orientation,rad);
	}
	for (i=0; i<nrTendonWrappers; i++)
	{
		DBGA("TendonWrapper "<<i<<" read and added");
		tendonWrapperVec[i]->createGeometry();
		IVRoot->addChild( tendonWrapperVec[i]->getIVRoot() );
	}
	for (i=0; i<nrTendons; i++)
	{
		tendonVec[i]->updateGeometry();
		tendonVec[i]->setRestPosition();
	}
	return SUCCESS;
}

void HumanHand::updateTendonGeometry()
{
	for (int i=0; i<nrTendons; i++) {
		tendonVec[i]->updateGeometry();
	}
}

void HumanHand::applyTendonForces()
{
	for (int i=0; i<nrTendons; i++) {
		tendonVec[i]->applyForces();
	}
}

/*! Applies tendon forces; for now both active and passive.*/
void HumanHand::DOFController(double)
{
	applyTendonForces();
}
