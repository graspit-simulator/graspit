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
// $Id: PQPCollision.cpp,v 1.8 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#include "PQPCollision.h"

#include "vcollide.h"

#include "body.h"
#include "debug.h"
#include "bBox.h"
#include "triangle.h"

PQPCollision::PQPCollision()
{
	mVc = new VCollide();
}

PQPCollision::~PQPCollision()
{
	delete mVc;
}

bool 
PQPCollision::addBody(Body *body, bool ExpectEmpty)
{
	//create the new object
	int id;
	//guard the body creation statement so that two collision bodies
	//don't get the same id
	mMutex.lock();
	int rc = mVc->NewObject(&id,getThreadId());
	mMutex.unlock();

	if (rc != VC_OK) {
		DBGA("PQPCollision error in VCollide.NewObject(): " << rc );
		return false;
	}

	if (id < 0) {
		DBGA("PQPCollision: VCollide returns negative id: " << id);
		return false;
	}

	if ( mIds.find(body) != mIds.end() ) {
		DBGA("PQPCollision: body already present in id list");
		return false;
	}
	if (~ExpectEmpty){
		// get the triangles of the object
		std::vector<Triangle> triangles;
		body->getGeometryTriangles(&triangles);

		//add all the triangles
		int triId = 0;
		double dp1[3], dp2[3], dp3[3];
		for (int i=0; i<(int)triangles.size(); i++) {
		triangles[i].v1.get(dp1);
		triangles[i].v2.get(dp2);
		triangles[i].v3.get(dp3);
		mVc->AddTri(dp1, dp2, dp3,triId++);
		}
	}

	//end the creation process
	if ((rc = mVc->EndObject(ExpectEmpty)) != VC_OK) {
		DBGA("PQPCollision error in VCollide.EndObject(): " << rc );
		return false;
	}

	//map the body pointer to the id
	mIds[body] = id;
	mBodies[id] = body;

	return true;	
}

bool PQPCollision::updateBodyGeometry(Body*, bool)
{
  DBGA("ERROR: Update body geometry not implemented in PQP collision");
  return false;
}

void 
PQPCollision::removeBody(Body *body)
{
	int id = getId(body);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	int rc = mVc->DeleteObject(id);
	if (rc != VC_OK) {
		DBGA("PQPCollision: error deleting object");
	}
	mIds.erase( mIds.find(body) );
	mBodies.erase( mBodies.find(id) );
}

void 
PQPCollision::cloneBody(Body *clone, const Body *original)
{
	int originalId = getId(original);
	if (originalId < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}

	int id,rc;
	//guard the body creation statement so that two collision bodies
	//don't get the same id
	mMutex.lock();
	rc = mVc->NewClone(&id, originalId, getThreadId());
	mMutex.unlock();

	if (rc != VC_OK) {
		DBGA("PQPCollision -- error creating clone: " << rc);
		return;
	}
	if ((rc = mVc->EndObject()) != VC_OK) {
		DBGA("PQPCollision -- error finishing clone: " << rc);
		return ;
	}
	mIds[clone] = id;
	mBodies[id] = clone;
}

void 
PQPCollision::setBodyTransform(Body *body, const transf &t)
{
	int id = getId(body);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	double newTrans[4][4];
	t.toRowMajorMatrix(newTrans);
	mVc->UpdateTrans(id, newTrans);	
}

void 
PQPCollision::activateBody(const Body *body, bool active)
{
	int id = getId(body);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	if (active) {
		mVc->ActivateObject(id);
	} else {
		mVc->DeactivateObject(id);
	}
}

void 
PQPCollision::activatePair(const Body *body1, const Body *body2, bool active)
{
	int id1 = getId(body1);
	int id2 = getId(body2);
	if (id1 < 0 || id2 < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	if (active) {
		mVc->ActivatePair(id1, id2);
	} else {
		mVc->DeactivatePair(id1, id2);
	}
}

bool PQPCollision::isActive(const Body *body1, const Body *body2)
{
	int id1 = getId(body1);
	if (id1 < 0) {
		DBGA("PQPCollision: body not found");
		return false;
	}
	if (!body2) {
		return mVc->isObjectActivated(id1);
	}
	int id2 = getId(body2);
	if (id2 < 0) {
		DBGA("PQPCollision: body not found");
		return false;
	}
	return mVc->isPairActivated(id1, id2);
}

bool 
PQPCollision::getIdsFromList(const std::vector<Body*> *interestList, int **iList, int *iSize)
{
	*iSize = 0;
	*iList = NULL;
	if ( interestList && !interestList->empty() ) {
		*iList = new int[ (int)interestList->size() ];
		std::vector<Body*>::const_iterator it;
		for (it = interestList->begin(); it!=interestList->end(); it++) {
			(*iList)[*iSize] = getId(*it);
			if ( (*iList)[*iSize]<0 ) {
				DBGA("PQPCollision: body not found");
				delete [] iList;
				return false;
			}
			(*iSize)++;
		}
	}
	return true;
}

int 
PQPCollision::allCollisions(DetectionType type, CollisionReport *report,
			   				 const std::vector<Body*> *interestList)
{
	//prepare the list of bodies we are interested in
	int iSize; int *iList;
	if (!getIdsFromList(interestList, &iList, &iSize)) return 0;

	//prepare the query type
	VCInternal::Query query;
	switch(type) {
		case FAST_COLLISION:
			query = VCInternal::FAST_COLLISION;
			break;
		case ALL_COLLISIONS:
			query = VCInternal::ALL_COLLISIONS;
			break;
		default:
			assert(0);
	}
	//prepare the result structure
	VCReportType *result = NULL;
	int resultSize = 0;
	if (report) {
		resultSize = MAXCOLLISIONS;
		result = new VCReportType[resultSize];
	}
	//collision detection call
	int numCols = mVc->AllCollisions(query, result, resultSize, iList, iSize, getThreadId());
	//convert the result 
	if (numCols && result) {
		assert(report);
		assert(numCols < resultSize);
		for (int i=0; i<numCols; i++) {
			Body *b1 = getBody( result[i].id1 );
			assert(b1);
			Body *b2 = getBody( result[i].id2 );
			assert(b2);
			report->push_back( CollisionData(b1,b2) );
		}
	}

	if (iList) delete [] iList;
	if (result) delete [] result;
	return numCols;
}

void 
PQPCollision::convertContactReport(PQP_ContactResult *pqpReport, ContactReport *report)
{
	ContactSetT::iterator it;
	for (it=pqpReport->contactSet.begin(); it!=pqpReport->contactSet.end(); it++) {

		position b1p( (*it).b1_pos[0], (*it).b1_pos[1], (*it).b1_pos[2] );
		position b2p( (*it).b2_pos[0], (*it).b2_pos[1], (*it).b2_pos[2] );

		vec3 b1n( (*it).b1_normal[0], (*it).b1_normal[1], (*it).b1_normal[2] );
		vec3 b2n( (*it).b2_normal[0], (*it).b2_normal[1], (*it).b2_normal[2] );

		report->push_back( ContactData(b1p, b2p, b1n, b2n ) );
	}
}

int 
PQPCollision::allContacts(CollisionReport *report, double threshold, const std::vector<Body*> *interestList)
{
	//prepare the list of bodies we are interested in
	int iSize; int *iList;
	if (!getIdsFromList(interestList, &iList, &iSize)) return 0;

	//prepare the result structure
	int resultSize = MAXCOLLISIONS;
	PQP_ContactResult *result = new PQP_ContactResult[resultSize];

	//contact detection call
	int numContacts = mVc->AllContacts(result, threshold, iList, iSize, getThreadId());

	for (int i=0; i<numContacts; i++) {
		Body *b1 = getBody( result[i].body1Id );
		assert(b1);
		Body *b2 = getBody( result[i].body2Id );
		assert(b2);
		report->push_back( CollisionData(b1,b2) );
		//also get the contact data
		convertContactReport( &result[i], &report->back().contacts );
		//compact contacts with same normal
		compactContactSet(&report->back().contacts);
	}

	delete [] result;
	return numContacts;
}

int 
PQPCollision::contact(ContactReport *report, double threshold, const Body *body1, const Body *body2)
{
	int id1 = getId(body1);
	int id2 = getId(body2);
	if (id1 < 0 || id2 < 0) {
		DBGA("PQPCollision: body not found");
		return 0;
	}

	PQP_ContactResult result;
	int numContacts = mVc->Contact(id1, id2, threshold, result);
	convertContactReport( &result, report );
	compactContactSet(report);
	return numContacts;
}

double 
PQPCollision::pointToBodyDistance(const Body *body1, position point,
								  position &closestPoint, vec3 &closestNormal)
{
	int id = getId(body1);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return 0;
	}
	PQP_REAL pt[3], closest_pt[3], closest_normal[3];
	pt[0] = point.x(); pt[1] = point.y(); pt[2] = point.z();
	mVc->FindShortDist(pt, id, closest_pt, closest_normal, 0);

	closestNormal.set(closest_normal[0], closest_normal[1], closest_normal[2]);
	closestPoint.set ( closest_pt[0], closest_pt[1], closest_pt[2]);

	closestPoint = closestPoint * body1->getTran();
	closestNormal = normalise(closestNormal) * body1->getTran();
	vec3 dif = point - closestPoint;
	return dif.len();
}

double 
PQPCollision::bodyToBodyDistance(const Body *body1, const Body *body2,
								 position &p1, position &p2)
{
	int id1 = getId(body1);
	int id2 = getId(body2);
	if (id1 < 0 || id2 < 0) {
		DBGA("PQPCollision: body not found");
		return 0;
	}

	PQP_DistanceResult dRes;
	mVc->Dist(id1, id2, dRes, -1);
	p1.set( dRes.P1()[0], dRes.P1()[1] ,dRes.P1()[2] );
	p2.set( dRes.P2()[0], dRes.P2()[1], dRes.P2()[2] );
	return dRes.Distance();
}

void 
PQPCollision::bodyRegion(const Body *body, position point, vec3 normal, 
				 		 double radius, Neighborhood *neighborhood)
{
	int id = getId(body);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	PQP_REAL pt[3], n[3];
	pt[0] = point.x(); pt[1] = point.y(); pt[2] = point.z();
	n[0] = normal.x(); n[1] = normal.y(); n[2] = normal.z();
	std::vector<PQP_Vec> ptList;
	mVc->FindRegion( id, pt, n, radius, &ptList );
	//convert results
	for (int i=0; i<(int)ptList.size(); i++) {
		neighborhood->push_back( position( ptList[i].d ) );
	}
}

void 
PQPCollision::getBoundingVolumes(const Body* body, int depth, std::vector<BoundingBox> *bvs)
{
	int id = getId(body);
	if (id < 0) {
		DBGA("PQPCollision: body not found");
		return;
	}
	std::vector<BV*> pqp_bvs;
	mVc->getBvs(id, depth, &pqp_bvs);
	for (int i=0; i<(int)pqp_bvs.size(); i++) {
		vec3 size( pqp_bvs[i]->d[0], pqp_bvs[i]->d[1], pqp_bvs[i]->d[2] );
		vec3 loc( pqp_bvs[i]->To[0], pqp_bvs[i]->To[1], pqp_bvs[i]->To[2] );

		vec3 v1( pqp_bvs[i]->R[0][0], pqp_bvs[i]->R[0][1], pqp_bvs[i]->R[0][2] );
		vec3 v2( pqp_bvs[i]->R[1][0], pqp_bvs[i]->R[1][1], pqp_bvs[i]->R[1][2] );
		vec3 v3( pqp_bvs[i]->R[2][0], pqp_bvs[i]->R[2][1], pqp_bvs[i]->R[2][2] );
		mat3 rot(v1,v2,v3);
		rot = rot.transpose();

		transf tr(rot, loc);
		bvs->push_back( BoundingBox(tr, size) );		
		delete pqp_bvs[i];
	}
}

void
PQPCollision::newThread()
{
	if (mVc->newThread()!=VC_OK) {
		DBGA("Threading disabled in VCollide");
		return;
	}
	CollisionInterface::newThread();
}
