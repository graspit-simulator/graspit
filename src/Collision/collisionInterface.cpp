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
// $Id: collisionInterface.cpp,v 1.7 2009/06/30 16:44:03 cmatei Exp $
//
//######################################################################

#include "collisionInterface.h"

#include <list>

//#define GRASPITDBG
#include "debug.h"

extern "C" {
#include <qhull_a.h>
}
#include <qmutex.h>
extern QMutex qhull_mutex;

const int CollisionInterface::CONTACT_DUPLICATE_THRESHOLD = 3.0;
//thread id 0 is reserved for the master thread
int CollisionInterface::mNextThreadId = 1;
QMutex CollisionInterface::mMutex;

/*! By definition, any CollisionInterface is created with threadId 0, which is
	the main thread.

	In general, you should not use multiple instances of the CollisionInterface
	and the one instance should be constructed in the main thread.
*/
CollisionInterface::CollisionInterface()
{
	int *threadId = new int(0);
	mThreadIdStorage.setLocalData( threadId );
}

void 
CollisionInterface::newThread()
{
	mMutex.lock();
	int *threadId = new int(mNextThreadId);
	mNextThreadId++;
	mMutex.unlock();
	mThreadIdStorage.setLocalData( threadId );
}

int 
CollisionInterface::getThreadId()
{
	int* threadId = mThreadIdStorage.localData();
	if (!threadId) return 0;
	return (*threadId);
}

void 
CollisionInterface::compactContactSet(ContactReport *contacts)
{
	if (contacts->size() < 2) return;
	DBGP("Compacting total contacts: " << contacts->size());

	double NORMAL_TOLERANCE = 1e-3;
	double DISTANCE_TOLERANCE = 1.0e-1;

	//this will hold all the groups of contacts that have the same normal
	std::list<ContactReport> contactGroups;

	//create lists of contacts with the same normal
	ContactReport::iterator cp;
	std::list<ContactReport>::iterator sp;
	for (cp=contacts->begin(); cp!=contacts->end(); cp++) {
		for (sp=contactGroups.begin(); sp!=contactGroups.end(); sp++) {
			if ( sp->begin()->b1_normal % cp->b1_normal > 1.0-NORMAL_TOLERANCE) break;
		}
		if (sp == contactGroups.end()) {
			contactGroups.push_back(ContactReport());
			contactGroups.back().push_back(*cp);
		} else {
			// we need to make sure it's not in there already since it's a list, not a set
			ContactReport::iterator it;
			for (it=sp->begin(); it!=sp->end(); it++) {
				vec3 dist = it->b1_pos - cp->b1_pos;
				if ( dist.len_sq() < DISTANCE_TOLERANCE) break;
			}
			if (it==sp->end()) sp->push_back(*cp);
		}
	}
 
	//take perimeter of sets with same normal
	for (sp=contactGroups.begin(); sp!=contactGroups.end(); sp++) {
		DBGP("Set with same normal: " << sp->size());
		if (sp->size() > 1) replaceContactSetWithPerimeter(*sp);
		DBGP("Perimeter contacts: " << sp->size());
	}

	//insert compacted sets back in result
	contacts->clear();
	for (sp=contactGroups.begin(); sp!=contactGroups.end(); sp++) {
		for (cp=sp->begin();cp!=sp->end();cp++) {
			contacts->push_back(*cp);
		}
	}
}

void 
CollisionInterface::replaceContactSetWithPerimeter(ContactReport &contactSet)
{
	if (contactSet.size() < 2) return;
	
	double my_resabs = 1.0e-1;

	// first check for a colinear point set
	vec3 currLine, testLine;
	ContactReport::iterator endPt1 = contactSet.begin();
	ContactReport::iterator endPt2 = ++contactSet.begin();
	ContactReport::iterator cp;
	for (cp=contactSet.begin();cp!=contactSet.end();cp++) {    
		currLine = endPt2->b1_pos - endPt1->b1_pos;
		testLine = cp->b1_pos - endPt1->b1_pos;
		vec3 crossProd = testLine * currLine;
		if ( crossProd.len() > my_resabs) break;  // not colinear
		double dot = testLine % currLine;
		if (dot < 0) endPt1 = cp;
		if (dot > currLine % currLine) endPt2 = cp;
	}

	if (cp==contactSet.end()) {  
		// colinear points
		ContactReport tmpSet;
		tmpSet.push_back(*endPt1);
		tmpSet.push_back(*endPt2);
		contactSet.clear();
		contactSet = tmpSet;
		return;
	}

	// compute the origin of the projection frame
	vec3 contactNormal = contactSet.begin()->b1_normal;
	vec3 normal = normalise(testLine * currLine);
	double Soffset = contactSet.begin()->b1_pos % normal;
	vec3 origin_pr = Soffset * normal;

	// compute 2 other axes along the plane of S
	vec3 axis1 = normalise(testLine);
	vec3 axis2 = normal * axis1;

	coordT *array = new coordT[contactSet.size()*2];
	coordT *ptr = &array[0];
	int ptCount = 0;
	for (cp=contactSet.begin(); cp!=contactSet.end(); cp++) {    
		*ptr++ = (cp->b1_pos - position::ORIGIN) % axis1;
		*ptr++ = (cp->b1_pos - position::ORIGIN) % axis2;
		ptCount++;
	}

	ContactReport tmpSet = contactSet;
	contactSet.clear();

	//qhull paramerers
	int exitcode,curlong,totlong;
	char options[200];

	//serialize access to qhull which is not thread-safe
	qhull_mutex.lock();

	bool ismalloc = False; 	// True if qh_freeqhull should 'free(array)'
	FILE *qhfp = fopen("logfile","w");
	if (!qhfp) {
		fprintf(stderr,"Could not open qhull logfile!\n");
		qh_init_A(NULL, stdout, stderr, 0, NULL);
	} else {
		qh_init_A(NULL, qhfp, qhfp, 0, NULL);
	}

	if((exitcode = setjmp(qh errexit))) {
		delete [] array;
		if (qhfp) fclose(qhfp);
		qhull_mutex.unlock();
		return;
	}

	sprintf(options, "qhull n Pp");
	try {
		qh_initflags(options);
		qh_init_B(&array[0],ptCount, 2, ismalloc);
		qh_qhull();
		qh_check_output();
	} catch(...) {
		//qhull has failed
		DBGA("QHull CompactSet failed!!!");
		//reinsert all original contacts
		contactSet.insert(contactSet.begin(), tmpSet.begin(), tmpSet.begin());
		delete [] array;
		fclose(qhfp);
		qhull_mutex.unlock();
		return;
	}
	fclose(qhfp);

	vertexT *vertex;
	double x,y;
	ContactData tmpContact;

	// keep only those vertices in the set that match the convex hull vertices
	FORALLvertices {
		x = vertex->point[0];
		y = vertex->point[1];
		tmpContact.b1_pos[0] = x * axis1[0] + y * axis2[0] + origin_pr[0];
		tmpContact.b1_pos[1] = x * axis1[1] + y * axis2[1] + origin_pr[1];
		tmpContact.b1_pos[2] = x * axis1[2] + y * axis2[2] + origin_pr[2];
		tmpContact.b1_normal = contactNormal;

		for (cp=tmpSet.begin(); cp!=tmpSet.end(); cp++) {
			if (fabs(cp->b1_pos[0] - tmpContact.b1_pos[0]) < my_resabs &&
				fabs(cp->b1_pos[1] - tmpContact.b1_pos[1]) < my_resabs &&
				fabs(cp->b1_pos[2] - tmpContact.b1_pos[2]) < my_resabs &&
				fabs(cp->b1_normal[0] - tmpContact.b1_normal[0]) < my_resabs &&
				fabs(cp->b1_normal[1] - tmpContact.b1_normal[1]) < my_resabs &&
				fabs(cp->b1_normal[2] - tmpContact.b1_normal[2]) < my_resabs)
				break;
		}
		if (cp==tmpSet.end()) {
		} else {
			contactSet.push_back(*cp);
		}
	}

	qh NOerrexit= True;
	qh_freeqhull(!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);
	qhull_mutex.unlock();
	delete [] array;
}

void
CollisionInterface::removeContactDuplicates(ContactReport *contacts, double duplicateThreshold)
{
	ContactReport::iterator it = contacts->begin();
	double threshSq = duplicateThreshold * duplicateThreshold;
	//loop through all contacts
	while (it!=contacts->end()) {
		//check all the other contacts if they are a duplicate of this one
		bool removeMe = false;
		ContactReport::iterator it2 = it+1;
		while(it2 != contacts->end()) {
			if ( (it2->b1_pos - it->b1_pos).len_sq() > threshSq || 
				 (it2->b2_pos - it->b2_pos).len_sq() > threshSq ) {
					 //not the same contact
					 it2++;
				 } else {
					 //same contact; one of them must go
					 if (it->distSq < it2->distSq) {
						 //this contact must go
						 it2 = contacts->erase(it2);
					 } else {
						 //our original contact must go
						 removeMe = true;
						 break;
					 }
				 }
		}
		if (removeMe) {
			it = contacts->erase(it);
		} else {
			it++;
		}
	}
}
