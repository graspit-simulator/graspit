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
// $Id: graspitCollision.cpp,v 1.10 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#include "graspitCollision.h"

#include <algorithm>

//#define GRASPITDBG
#include "debug.h"

#include "body.h"
#include "collisionModel.h"
#include "collisionAlgorithms.h"

//#define PROF_ENABLED
#include "profiling.h"

using namespace Collision;

/*! A pair of bodies is active iff:
	<ul>
	<li> none of the bodies is disabled
	<li> collision between this specific pair of bodies is not disabled
	<li> when \a interestList is not NULL, if at least one of the bodies
	is in the \a interestList.
	<li> at least one of the bodies has the same thread ID as the 
	current thread, and the other body has either the same thread ID, or
	thread ID 0 (master thread). In other words, bodies from a given 
	thread only collide with bodies from the same thread, or from the 
	master thread (ID 0).
	</ul>
*/
void 
GraspitCollision::getActivePairs(std::list<CollisionPair> *activePairs, 
								 const std::set<CollisionModel*> *interestList)
{
	int myThread = getThreadId();
	//cycle through all models
	std::vector<CollisionModel*>::iterator it1, it2;
	for (it1=mModels.begin(); it1!=mModels.end(); it1++) {
		//if the first model is disabled we are done with it
		if ( !(*it1)->isActive() ) continue;
		//if the first model is in the wrong thread, we are done
		if ( (*it1)->getThreadId() != myThread && (*it1)->getThreadId() != 0) continue;
		//get the list of disabled pairs involving this model
		//key is this model, as it comes earlier in the sorted order
		std::pair<DisabledIterator, DisabledIterator> range;
		range = mDisabledMap.equal_range(*it1);
		//check if this body is in the interest list
		bool interest1 = true;
		if (interestList) {
			if (interestList->find(*it1) == interestList->end()) interest1 = false;
		}
		//cycle throught the rest of the models.
		//start from it1 since they are sorted
		for (it2 = it1+1; it2!=mModels.end(); it2++) {
			//if second model is disabled we are done
			if ( !(*it2)->isActive() ) continue;
			//check thread id
			if ( (*it2)->getThreadId() != myThread && (*it2)->getThreadId() != 0) continue;
			if ( myThread != 0 && (*it1)->getThreadId() == 0 && (*it2)->getThreadId() == 0) continue;
			//check if this pair is disabled
			DisabledIterator rangeIt;
			bool disabled = false;
			for (rangeIt = range.first; rangeIt != range.second && !disabled; rangeIt++) {
				if (rangeIt->second == *it2) disabled = true;
			}
			//this pair is in the disabled list
			if (disabled) continue;
			//check if either of these bodies is in the interest list
			if (!interest1) {
				if (interestList->find(*it2) == interestList->end()) continue;
			}
			//this pair is not disabled; add it to collision list
			activePairs->push_back( CollisionPair(*it1, *it2) );
		}
	}
}

void 
GraspitCollision::activateBody(const Body* body, bool active)
{
	CollisionModel *model = getModel(body);
	if (!model) {
		DBGA("GCOL: model not found");
		return;
	}
	model->setActive(active);
}

void 
GraspitCollision::activatePair(const Body* body1, const Body* body2, bool active)
{
	CollisionModel *model1 = getModel(body1);
	CollisionModel *model2 = getModel(body2);
	if (!model1 || !model2) {
		DBGA("GCOL: model not found");
		return;
	}
	if (model2 == model1) {
		//bodies are the same
		DBGA("GCOL Warning: insertion collision pair is actually one body");
		model1->setActive(active);
		return;
	}
	else if (model2 < model1) {
		//make sure the key is always smaller than the value
		std::swap(model1, model2);
	}
	if (!active) {
		DBGP("Disable pair: " << model1 << " -- " << model2);
		mDisabledMap.insert( std::pair<const CollisionModel*, const CollisionModel*>(model1, model2) );
	} else {
		//remove from list
	}
}

bool 
GraspitCollision::isActive(const Body* body1, const Body* body2)
{
	CollisionModel *model1 = getModel(body1);
	if (!model1) {
		DBGA("GCOL: model not found");
		return false;
	}
	if (!body2) return model1->isActive();

	CollisionModel *model2 = getModel(body2);
	if (!model2) {
		DBGA("GCOL: model not found");
		return false;
	}
	if (!model1->isActive() || !model2->isActive()) {
		return false;
	}
	if (model2==model1) {
		//bodies are the same
		DBGA("GCOL Warning: collision pair is actually one body");
		return model1->isActive();
	}
	else if (model2 < model1) {
		//make sure the key is always smaller than the value
		std::swap(model1, model2);
	}

	//get the list of disabled pairs involving this model
	//key is model1, as it comes earlier in the sorted order
	std::pair<DisabledIterator, DisabledIterator> range;
	range = mDisabledMap.equal_range(model1);
	//look for the second model in this range
	DisabledIterator rangeIt;
	bool disabled = false;
	for (rangeIt = range.first; rangeIt != range.second && !disabled; rangeIt++) {
		if (rangeIt->second == model2) disabled = true;
	}
	if (disabled) return false;
	return true;
}

bool 
GraspitCollision::addBody(Body *body,  bool)
{
	if (getModel(body)) {
		DBGA("GCOL: body already present");
		return false;
	}

	CollisionModel *model = new CollisionModel(getThreadId());

	// get the triangles of the object
	std::vector<Triangle> triangles;
	body->getGeometryTriangles(&triangles);

	//add all the triangles
	for (int i=0; i<(int)triangles.size(); i++) {
		model->addTriangle(triangles[i]);
	}

	//end the creation process
	model->build();

	//map the body pointer to the id
	mModelMap[body] = model;
	mBodyMap[model] = body;
	mModels.push_back(model);

	//we keep the list of models sorted based on pointer comparisons
	//this helps with keeping track of pairs of active / inactive pairs
	std::sort( mModels.begin(), mModels.end() );

	return true;	
}

bool GraspitCollision::updateBodyGeometry(Body* body, bool)
{
  CollisionModel *model = getModel(body);
  if (!model)
  {
    DBGA("GCOL: body not found for geometry update");
    return false;
  }
  model->reset();

  // get the triangles of the object
  std::vector<Triangle> triangles;
  body->getGeometryTriangles(&triangles);
  
  //add all the triangles
  for (int i=0; i<(int)triangles.size(); i++) {
    model->addTriangle(triangles[i]);
  }
  
  //end the creation process
  model->build();
  return true;
}

void 
GraspitCollision::removeBody(Body *body)
{
	CollisionModel *model = getModel(body);
	if (!model) {
		DBGA("GCOL: model not found");
		return;
	}
		
	std::vector<CollisionModel*>::iterator it;
	for (it=mModels.begin(); it!=mModels.end(); it++){
		if ( *it == model ) break;
	}
	if (it==mModels.end()) {
		DBGA("GCOL error: model for deletion not present");
	} else {
		mModels.erase(it);
	}

	mModelMap.erase( mModelMap.find(body) );
	mBodyMap.erase( mBodyMap.find(model) );
	delete model;
}

void 
GraspitCollision::cloneBody(Body* clone, const Body* original)
{
	CollisionModel *originalModel = getModel(original);
	if (!originalModel) {
		DBGA("GCOL: model not found");
		return;
	}
	if (getModel(clone) ) {
		DBGA("GCOL: clone already in collision detection system");
		return;
	}
	CollisionModel *cloneModel = new CollisionModel(getThreadId());
	cloneModel->cloneModel(originalModel);

	//map the body pointer to the id
	mModelMap[clone] = cloneModel;
	mBodyMap[cloneModel] = clone;
	mModels.push_back(cloneModel);

	//we keep the list of models sorted based on pointer comparisons
	//this helps with keeping track of pairs of active / inactive pairs
	std::sort( mModels.begin(), mModels.end() );
}

void 
GraspitCollision::setBodyTransform(Body *body, const transf &t)
{
	CollisionModel *model = getModel(body);
	if (!model) {
		DBGA("GCOL: model not found");
		return;
	}
	model->setTran(t);
}

void 
GraspitCollision::convertInterestList(const std::vector<Body*> *inList, 
									  std::set<CollisionModel*> *outSet)
{
	for (int i=0; i<(int)inList->size(); i++) {
		CollisionModel *model = getModel((*inList)[i]);
		if (!model) {
			DBGA("GCOL: interest list model not found");
		} else {
			outSet->insert(model);
		}
	}
}

int 
GraspitCollision::allCollisions(DetectionType type, CollisionReport *report, 
							    const std::vector<Body*> *interestList)
{
	int collisions = 0;
	//convert the interest list
	std::set<CollisionModel*> *intModelList = NULL;
	if (interestList) {
		intModelList = new std::set<CollisionModel*>;
		convertInterestList(interestList, intModelList);
	}
	//get the list of active pairs
	std::list<CollisionPair> activeList;
	getActivePairs(&activeList, intModelList);
	std::list<CollisionPair>::iterator it;
	for (it = activeList.begin(); it!=activeList.end(); it++) {
		CollisionCallback cc(it->first, it->second);
		//the actual recursive call to collision detection
		startRecursion(it->first, it->second, &cc);
		DBGST( cc.printStatistics() );
		if ( cc.isCollision() ) {
			collisions++;
			if (type == FAST_COLLISION) break;
			Body *b1 = getBody(it->first);
			Body *b2 = getBody(it->second);
			assert(b1 && b2);
			report->push_back( CollisionData(b1,b2) );
		}
	}
	if (intModelList) delete intModelList;
	return collisions;
}

int 
GraspitCollision::allContacts(CollisionReport *report, double threshold, 
							  const std::vector<Body*> *interestList)
{
	int contacts = 0;
	//convert the interest list
	std::set<CollisionModel*> *intModelList = NULL;
	if (interestList) {
		intModelList = new std::set<CollisionModel*>;
		convertInterestList(interestList, intModelList);
	}
	//get the list of active pairs
	std::list<CollisionPair> activeList;
	getActivePairs(&activeList, intModelList);
	std::list<CollisionPair>::iterator it;
	for (it = activeList.begin(); it!=activeList.end(); it++) {
		ContactCallback cc(threshold, it->first, it->second);
		//the actual recursive call to contact detection
		startRecursion(it->first, it->second, &cc);
		DBGST( cc.printStatistics() );
		if (!cc.getReport().empty()) {
			contacts ++;
			Body *b1 = getBody(it->first);
			Body *b2 = getBody(it->second);
			assert(b1 && b2);
			report->push_back( CollisionData(b1,b2) );
			report->back().contacts = cc.getReport();

			//remove duplicates from the contact report
			DBGP("Body: " << getBody(it->first)->getName().latin1());
			DBGP("Before duplicate removal: " << report->back().contacts.size() );
			removeContactDuplicates(&(report->back().contacts), CONTACT_DUPLICATE_THRESHOLD);
			DBGP("After duplicate removal: " << report->back().contacts.size() );

			compactContactSet(&report->back().contacts);
			DBGP("After compact set: " << report->back().contacts.size() );
		}
	}
	if (intModelList) delete intModelList;
	return contacts;
}

int 
GraspitCollision::contact(ContactReport *report, double threshold, 
							const Body *body1, const Body *body2)
{
	CollisionModel *model1 = getModel(body1);
	CollisionModel *model2 = getModel(body2);
	if (!model1 || !model2) {
		DBGA("GCOL: model not found");
		return 0;
	}

	ContactCallback cc(threshold, model1, model2);
	startRecursion(model1, model2, &cc);
	DBGST( cc.printStatistics() );
	if (!cc.getReport().empty()) {
		*report = cc.getReport();

		//remove duplicates from the contact report
		DBGP("Before duplicate removal: " << report->size() );
		removeContactDuplicates(report, CONTACT_DUPLICATE_THRESHOLD);
		DBGP("After duplicate removal: " << report->size() );

		compactContactSet(report);
		DBGP("After compact set: " << report->size() );
	}
	return report->size();
}

double 
GraspitCollision::pointToBodyDistance(const Body *body1, position point,
									  position &closestPoint, vec3 &closestNormal)
{
	CollisionModel *model = getModel(body1);
	if (!model) {
		DBGA("GCOL: model not found");
		return 0;
	}
	//this callback operates in body coordinates
	ClosestPtCallback pc(model, point * body1->getTran().inverse());
	startRecursion(model, NULL, &pc);
	DBGST( pc.printStatistics() );
	//go back to world coordinates
	closestPoint = pc.getClosestPt() * body1->getTran();
	//we compute the normal as being in the direction that
	//relates the two points. There is really no need to do this here,
	//but it is the legacy interface. The PQP interface looks at the normal of
	//the triangle instead, but I believe this is better
	closestNormal = normalise(closestPoint - point);
	return pc.getMin();
}

double 
GraspitCollision::bodyToBodyDistance(const Body *body1, const Body *body2,
									 position &p1, position &p2)
{
	CollisionModel *model1 = getModel(body1);
	CollisionModel *model2 = getModel(body2);
	if (!model1 || !model2) {
		DBGA("GCOL: model not found");
		return 0;
	}
	DistanceCallback dc(model1, model2);
	//the actual recursive call to contact detection
	startRecursion(model1, model2, &dc);
	DBGST( dc.printStatistics() );
	dc.getClosestPoints(p1, p2);
	//the legacy interface requests p1 and p2 in each body's coordinate system
	p1 = p1 * body1->getTran().inverse();
	p2 = p2 * body2->getTran().inverse();
	return dc.getMin();
}

void 
GraspitCollision::bodyRegion(const Body *body, position point, vec3 normal, 
							 double radius, Neighborhood *neighborhood)
{
	CollisionModel *model = getModel(body);
	if (!model) {
		DBGA("GCOL: model not found");
		return;
	}
	//region callback operates in body coordinate system
	RegionCallback rc(model, point, normal, radius);
	//the actual recursive call to region detection
	startRecursion(model, NULL, &rc);
	DBGST( rc.printStatistics() );
	*neighborhood = rc.getRegion();
	//add actual reference point to set. Not sure why this is done
	neighborhood->push_back(point);
}


void 
GraspitCollision::getBoundingVolumes(const Body* body, int depth, std::vector<BoundingBox>* bvs)
{
	CollisionModel *model = getModel(body);
	if (!model) {
		DBGA("GCOL: model not found");
		return;
	}
	model->getBoundingVolumes(depth, bvs);

}
