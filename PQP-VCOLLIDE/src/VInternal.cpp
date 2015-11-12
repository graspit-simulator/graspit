/************************************************************************\

  Copyright 1997 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following three paragraphs appear in all copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL
  HILL BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
  INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS,
  ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
  EVEN IF THE UNIVERSITY OF NORTH CAROLINA HAVE BEEN ADVISED OF
  THE POSSIBILITY OF SUCH DAMAGES.


  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following three paragraphs appear in all copies.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
  BASIS, AND THE UNIVERSITY OF NORTH CAROLINA HAS NO OBLIGATION
  TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
  MODIFICATIONS.


   --------------------------------- 
  |Please send all BUG REPORTS to:  |
  |                                 |
  |   geom@cs.unc.edu               |
  |                                 |
   ---------------------------------
  
     
  The authors may be contacted via:

  US Mail:  A. Pattekar/J. Cohen/T. Hudson/S. Gottschalk/M. Lin/D. Manocha
            Department of Computer Science
            Sitterson Hall, CB #3175
            University of N. Carolina
            Chapel Hill, NC 27599-3175
	    
  Phone:    (919)962-1749
	    
  EMail:    geom@cs.unc.edu

\************************************************************************/

/************************************************************************\
Filename: VInternal.C
--
Description: This file implements the member functions of the class vinternal.c

\************************************************************************/



#include <iostream>
#include <string.h>     //for memset and memcpy.
#include <stdio.h>
#include <stdlib.h>
#include "VInternal.H"
#include "MatVec.h"

//#define GRASPITDBG
#include "debug.h"

//#include "dmalloc.h"

//#include "obb.H"

#define VC_USE_NBODY 0 // 1 if NBody should be used for FAST overlapi tests. Significant speed improvement in some
					   // cases but NOT THREAD-SAFE. Set to 0 if NBody should not be used.

const int DEFAULT_SIZE=10; //some arbitrary default size for "vc_objects" array.

//error codes returned by VCollide API.
//these are multiply defined in the files VCollide.H and VCollide.h
//so, any changes to these need to be reflected in all three places.
const int VC_ERR_INVALID_ID            = -4; //invalid id was passed to the
                                             //routine.
const int VC_ERR_EMPTY_OBJECT          = -3; //EndObject called without adding
                                             //adding any triangles.
const int VC_ERR_CALL_OUT_OF_SEQUENCE  = -2; //calls out of sequence.
const int VC_ERR                       = -1; //some other error.
const int VC_OK                        =  1; //No error.

#ifndef _VCREPORTTYPE
#define _VCREPORTTYPE
struct VCReportType
{
  int id1, id2;
};
#endif

VCInternal::VCInternal()
{
  nbody = new NBody;

  state = VCstate_default;
  next_id = 0;
  
  mSize = DEFAULT_SIZE;                   //set the size of the array.
  vc_objects = new VCObject*[mSize]; //allocate the array.

  int i;
  for (i=0; i<mSize; i++)
    vc_objects[i] = NULL;
  
  disabled.Clear();  //to begin with, no pairs are disabled.
}


VCInternal::~VCInternal()
{

  //deallocate the memory.
  int i;
  for (i=0; i<mSize; i++)
    {
      if (vc_objects[i])
	{
	  delete vc_objects[i]->b;
	  delete vc_objects[i];
	}
    }
  delete [] vc_objects;
  delete nbody;
}

int
VCInternal::newThread()
{
#ifndef VC_THREAD_SUPPORT
	fprintf(stderr,"Thread support disabled in VCInternal!\n");
	return VC_ERR;
#endif
	if (VC_USE_NBODY) {
		fprintf(stderr,"VC_USE_NBODY is on, it is not thread-safe!\n");
		return VC_ERR;
	}

	if (state != VCstate_default) {
		fprintf(stderr,"Failed to set new VCollide thread!\n");
		return VC_ERR_CALL_OUT_OF_SEQUENCE;
	}
	return VC_OK;
}

int VCInternal::ResetObject(int id)
{
  if (id<0 || id >= next_id) return VC_ERR;

  int threadId = 0;
  if (vc_objects[id]) {
	  threadId = vc_objects[id]->threadId;
	  DeleteObject(id);
  }

  //check if we are in the correct state.
  if (state != VCstate_default)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }
  state = VCstate_newObject; //set the new state.

  vc_objects[id] = new VCObject;
  if (vc_objects[id] == NULL)
    return VC_ERR;

  current_id = id;
  vc_objects[id]->id = id;
  vc_objects[id]->isClone = false;
  vc_objects[id]->threadId = threadId;
  vc_objects[id]->b = new PQP_Model;
  vc_objects[id]->b->BeginModel();
  vc_objects[id]->activation_state = 1;
  
  return VC_OK;
}

int VCInternal::NewClone(int *id, int originalId, int threadId)
{
  //check if we are in the correct state.
  if (state != VCstate_default)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }
	
  //check that the original's id is valid
  if ( originalId < 0 || originalId >= mSize || vc_objects[originalId]==NULL) {
	  fprintf(stderr,"VCINT: invalid original ID \n");
	  return VC_ERR;
  }

  state = VCstate_newClone; //set the new state.

  //increase the size of the "vc_objects" array if required.
  if (next_id >= mSize) 
    {
      int newsize = (next_id >= 2*mSize) ? (next_id+1) : 2*mSize;
      VCObject **temp = new VCObject*[newsize];
      int i;
      for (i=0; i<mSize; i++)
		temp[i] = vc_objects[i];
      for (i=mSize; i<newsize; i++)
		temp[i] = NULL;
      delete [] vc_objects;
      vc_objects = temp;
      mSize = newsize;
      
    }
  
  //allocate a new object.
  vc_objects[next_id] = new VCObject;
  if (vc_objects[next_id] == NULL)
    return VC_ERR;
  
  *id = next_id;  //for returning the id generated by VCollide.
  current_id = next_id;
  vc_objects[next_id]->id = next_id;
  //std::cerr << "Created clone in thread " << threadId << " with ID " 
  //			<< vc_objects[next_id]->id << "\n";
  vc_objects[next_id]->isClone = true;
  vc_objects[next_id]->threadId = threadId;
//  vc_objects[next_id]->b = new PQP_Model;
//  vc_objects[next_id]->b->BeginModel();
//  point to the original model's geometry instead of creating a new one
  vc_objects[next_id]->b = vc_objects[originalId]->b;
  vc_objects[next_id]->activation_state = 1;
  next_id++; 

  //fprintf(stderr,"VCINT clone created. \n");

  return VC_OK;
}

int VCInternal::NewObject(int *id, int threadId) //create a new object in the database.
{
  //check if we are in the correct state.
  if (state != VCstate_default)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }
  state = VCstate_newObject; //set the new state.

  //increase the size of the "vc_objects" array if required.
  if (next_id >= mSize) 
    {
      int newsize = (next_id >= 2*mSize) ? (next_id+1) : 2*mSize;
      VCObject **temp = new VCObject*[newsize];
      int i;
      for (i=0; i<mSize; i++)
		temp[i] = vc_objects[i];
      for (i=mSize; i<newsize; i++)
		temp[i] = NULL;
      delete [] vc_objects;
      vc_objects = temp;
      mSize = newsize;
      
    }
  
  //allocate a new object.
  vc_objects[next_id] = new VCObject;
  if (vc_objects[next_id] == NULL)
    return VC_ERR;
  
  *id = next_id;  //for returning the id generated by VCollide.
  current_id = next_id;
  vc_objects[next_id]->id = next_id;
  vc_objects[next_id]->isClone = false;
  vc_objects[next_id]->threadId = threadId;
  vc_objects[next_id]->b = new PQP_Model;
  vc_objects[next_id]->b->BeginModel();
  vc_objects[next_id]->activation_state = 1;
  next_id++; 
  
  return VC_OK;
}

int VCInternal::AddTri(double v1[], double v2[], double v3[],int triInd) 
{                     //add geometry to the newly created object.

  //check whether we are in the correct state.
  if ( (state != VCstate_newObject) && (state != VCstate_addTri) )
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }

  state=VCstate_addTri; //set the new state.

  vc_objects[current_id]->b->AddTri(v1, v2, v3,triInd);  //add triangle.


  return VC_OK;
  
}

int VCInternal::EndObject(bool ExpectEmpty)
{   //tells VCollide that inputting the geometry is complete.

  //check whether we are in the correct state.
	if (state == VCstate_newObject) {
		//return VC_ERR_EMPTY_OBJECT;
		//even for empty bodies, end the processing
	} else if ( state != VCstate_addTri  && !ExpectEmpty) {
		if ( state != VCstate_newClone || !vc_objects[current_id]->isClone )
			return VC_ERR_CALL_OUT_OF_SEQUENCE;
	}

  state = VCstate_default;  //set the new state.

  //add the object to the NBody database.

  if (VC_USE_NBODY) nbody->AddObject(current_id, vc_objects[current_id]->b);

  
  if ( !vc_objects[current_id]->isClone) {
	  //now, have PQP build the OBB tree.
	vc_objects[current_id]->b->EndModel(ExpectEmpty);
  } //clones already have this completed	

  //initialize the transformation matrix to identity.
  //doing it the following way is probably faster, since the
  //compiler can use lower level memory calls for initialization.
  memset( ( (void *)vc_objects[current_id]->trans), 0, 16*sizeof(double) );
  vc_objects[current_id]->trans[0][0] = 1.0;
  vc_objects[current_id]->trans[1][1] = 1.0;
  vc_objects[current_id]->trans[2][2] = 1.0;
  vc_objects[current_id]->trans[3][3] = 1.0;

  return VC_OK;
  
}

int VCInternal::EditObject(int id)
{
  if (state != VCstate_default)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }
  if (id >= mSize)  //invalid id.
    {
      //cerr<<"VCInternal::EditObject - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::EditObject - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }

  state=VCstate_moveTri; //set the new state.

  current_id = id;
  
  return VC_OK;
}
  

int VCInternal::EditTri(double v1[], double v2[], double v3[],int triInd) 
{                     //move one of the triangles

  //check whether we are in the correct state.
  if (state != VCstate_moveTri)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }

  vc_objects[current_id]->b->MoveTri(v1, v2, v3,triInd);  //change triangle.

  return VC_OK;
  
}

int VCInternal::EditFinished(void)
{   //tells VCollide that inputting the geometry is complete.

  //check whether we are in the correct state.
  if (state != VCstate_moveTri)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  state = VCstate_default;  //set the new state.

  //update the nbody bounding box
  if (VC_USE_NBODY) nbody->AddObject(current_id, vc_objects[current_id]->b);
  
  //now, have PQP rebuild the OBB tree.
  vc_objects[current_id]->b->EndModel();

  return VC_OK;

}  



int VCInternal::GetObject(int id,double **& vList)
{
  int i;
  double R1[3][3], T1[3];

  if ((id < mSize) && vc_objects[id] != NULL) {
			      
    //set up the rotation and translation matrices
    R1[0][0] = vc_objects[id]->trans[0][0];
    R1[0][1] = vc_objects[id]->trans[0][1];
    R1[0][2] = vc_objects[id]->trans[0][2];
    R1[1][0] = vc_objects[id]->trans[1][0];
    R1[1][1] = vc_objects[id]->trans[1][1];
    R1[1][2] = vc_objects[id]->trans[1][2];
    R1[2][0] = vc_objects[id]->trans[2][0];
    R1[2][1] = vc_objects[id]->trans[2][1];
    R1[2][2] = vc_objects[id]->trans[2][2];
    
    T1[0] = vc_objects[id]->trans[0][3];
    T1[1] = vc_objects[id]->trans[1][3];
    T1[2] = vc_objects[id]->trans[2][3];

    vList = (double **)malloc(3*vc_objects[id]->b->num_tris*sizeof(double *));

    for (i=0;i<vc_objects[id]->b->num_tris;i++) {
      vList[3*i] = (double *)malloc(3*sizeof(double));
      MxVpV(vList[3*i],R1,vc_objects[id]->b->tris[i].p1,T1);
      vList[3*i+1] = (double *)malloc(3*sizeof(double));
      MxVpV(vList[3*i+1],R1,vc_objects[id]->b->tris[i].p2,T1);
      vList[3*i+2] = (double *)malloc(3*sizeof(double));
      MxVpV(vList[3*i+2],R1,vc_objects[id]->b->tris[i].p3,T1);
    }
    
    return vc_objects[id]->b->num_tris;
  }
  return 0;
}

int VCInternal::UpdateTrans(int id, double t[][4])
{             
	//update the transformation matrix of the object.
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;


  VCObject *current;
  
  if (id >= mSize)  //invalid id.
    {
      //cerr<<"VCInternal::update_trans - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::update_trans - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      current = vc_objects[id];
    }

  //update the private copy of the transformation matrix.
  memcpy((void *)current->trans, (void *)t, 16*sizeof(double));
  //have the nbody database update itself appropriately.
  if (VC_USE_NBODY) nbody->UpdateTrans(current->id, t);

  return VC_OK;
  
}


int VCInternal::ActivateObject(int id)
{  //activate an object for collision detection.

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
  if (id >= mSize)  //invalid id.
    {
      //cerr<<"VCInternal::activate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::activate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      vc_objects[id]->activation_state = 1;
    }
  return VC_OK;
}

int VCInternal::DeactivateObject(int id)
{  //deactivate an object from collision detection.

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
  if (id >= mSize)  //invalid id.
    {
      //cerr<<"VCInternal::deactivate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::deactivate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    } 
  else
    {
      vc_objects[id]->activation_state = 0;
    }
  return VC_OK;
}


int VCInternal::ActivatePair(int id1, int id2)
{
  
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
  if ( (id1 >= mSize) || (id2 >= mSize) )  //invalid id.
    {
      //cerr<<"VCInternal::activate_pair - invalid id for activation\n";
      return VC_ERR_INVALID_ID;
    }
  else if ( (vc_objects[id1] == NULL) || (vc_objects[id2] == NULL) )//invalid id.
    {
      //cerr<<"VCInternal::activate_pair - invalid id for activation\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      disabled.DelPair(id1, id2);
      return VC_OK;
    }
}

int VCInternal::DeactivatePair(int id1, int id2)
{

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
  if ( (id1 >= mSize) || (id2 >= mSize) )  //invalid id.
    {
      //cerr<<"VCInternal::deactivate_pair - invalid id for deactivation\n";
      return VC_ERR_INVALID_ID;
    }
  else if ( (vc_objects[id1] == NULL) || (vc_objects[id2] == NULL) )//invalid id.
    {
      //cerr<<"VCInternal::deactivate_pair - invalid id for deactivation\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      if (id1!=id2)
		disabled.AddPair(id1, id2);
      
      return VC_OK;
    }
}

bool VCInternal::isObjectActivated(int id)
{
  return vc_objects[id]->activation_state == 1;
}

bool VCInternal::isPairActivated(int id1, int id2)
{
  return !disabled.ExistsPair(id1,id2);
}

int VCInternal::DeleteObject(int id) //delete an object from the database.
{
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
  
  if (id >= mSize) //invalid id.
    {
      //cerr<<"VCollide::delete_object - object with id = "<<id<<" does not exist\n";
      return VC_ERR_INVALID_ID;
    }
  
  if (vc_objects[id] == NULL) //invalid id.
    {
      //cerr<<"VCollide::delete_object - object with id = "<<id<<" does not exist\n";
      return VC_ERR_INVALID_ID;
    } 
  else
    {
		if ( !vc_objects[id]->isClone ) {
			delete vc_objects[id]->b; //delete the RAPID box.
			// WHAT IF I DELETE THE ORIGINAL AND THE CLONE LINGERS??!!??
		}
      delete vc_objects[id];    //delete the object.
      vc_objects[id] = NULL; 
      
      disabled.DelPairsInvolvingId(id);

	  if (VC_USE_NBODY) nbody->DeleteObject(id); //delete the object from the nbody database.
      return VC_OK;
    }
  
}

/*
	Prepares a list of bodies that we actually need to perform collision or contact tests on.
	This is based on which bodies are enabled, which we are interested in, and, in a single
	threaded app, on what the overlapping bounding box structure tells us.
*/
int
VCInternal::GetEnabledList(PairData *collTests, int *interestList, int interestSize, int myThread)
{
  //do we have a list of indices we are interested in?
  if (interestSize) assert(interestList);
	
  //std::cerr << "Enabled list request in thread " << myThread << "\n";

  if (!VC_USE_NBODY) {
    //dumb checking that does not use NBody, but checks everybody against everybody. Thread-safe.
	for (int i=0; i<mSize; i++) {
		if ( !vc_objects[i] ) continue;
		if ( vc_objects[i]->activation_state != 1 ) continue;
		//we only do coll det. btw. objects from this thread and objects from thread 0 (main thread)
		//however, they all collide with bodies from thread 0. This is irrelevant if VC_THREAD_SUPPORT is not
		//defined as everybody is from thread 0.
		if (vc_objects[i]->threadId != myThread && vc_objects[i]->threadId != 0) continue;

		for (int k=0; k < i; k++) {
			if (!vc_objects[k]) continue;
			if (vc_objects[k]->activation_state != 1) continue;
			//clones have to collide with other clones; otherwise we can't test hands for self-collision
			//if (vc_objects[k]->isClone && vc_objects[i]->isClone) continue;
			if (vc_objects[k]->threadId != myThread && vc_objects[k]->threadId != 0) continue;
			if (myThread != 0 && vc_objects[i]->threadId == 0 && vc_objects[k]->threadId == 0) continue;
			if (disabled.ExistsPair(k,i)) continue;
			/*
			if (myThread == 1) {
				std::cerr << "Adding pair: " << vc_objects[i]->threadId << " and " 
						  << vc_objects[k]->threadId << "\n";
			}
			*/
			if (interestList) {
				int l;
				for (l=0; l<interestSize; l++) {
					if ( interestList[l]==i || interestList[l]==k ) break;
				}
				if (l == interestSize) continue;
			}
			collTests->AddPair(i, k );
		}  
	}
  } else {
	//use NBody for overlapping pairs. NOT THREAD_SAFE
	//Simultaneously traverse the "overlapping_pairs" database and the 
	//"disabled_pairs" database and decide where the actual collisions tests are required
	for (int i=0; i<nbody->overlapping_pairs.size; i++) {
	    
		if (!vc_objects[i]) continue;
		if (vc_objects[i]->activation_state != 1) continue;
		if (vc_objects[i]->threadId != myThread && vc_objects[i]->threadId != 0) continue;

		Elem *curr_disabled;
		if (i<disabled.size)
		curr_disabled = disabled.arr[i];
		else
		curr_disabled = NULL;
	      
		for (Elem *curr_ovrlp = nbody->overlapping_pairs.arr[i]; curr_ovrlp!=NULL; curr_ovrlp = curr_ovrlp->next) {
		
			//traverse the "disabled_pairs" list
			while (curr_disabled != NULL && curr_disabled->id < curr_ovrlp->id ) {
				curr_disabled = curr_disabled->next;
			}
		
			//is this pair disabled?
			if ( curr_disabled && (curr_disabled->id == curr_ovrlp->id ) ) continue;

			//clones have to collide with other clones; otherwise we can't test hands for self-collision
			//if ( vc_objects[curr_ovrlp->id]->isClone && vc_objects[i]->isClone ) continue;

			//we only do coll det. btw. objects from this thread and objects from thread 0 (main thread)
			//however, they all collide with bodies from thread 0. This is irrelevant if VC_THREAD_SUPPORT is not
			//defined as everybody is from thread 0.
			if (vc_objects[curr_ovrlp->id]->threadId != myThread && 
				vc_objects[curr_ovrlp->id]->threadId != 0) continue;
			if (myThread != 0 && vc_objects[i]->threadId == 0 && vc_objects[curr_ovrlp->id]->threadId == 0) continue;

			//is at least one of these bodies in our interest list?
			if (interestList) {
				int il;
				for (il=0; il<interestSize; il++) {
					if ( interestList[il]==i || interestList[il]==curr_ovrlp->id) break;
				}
				if (il == interestSize) {
					//fprintf(stderr,"Collision %d -- %d NOT in interest list\n",i,curr_ovrlp->id);
					continue;
				}
				//fprintf(stderr,"Collision %d -- %d is in interest list\n",i,curr_ovrlp->id);
			}

			//we need to do coll detection on this pair
			if (vc_objects[curr_ovrlp->id]->activation_state == 1) {
				collTests->AddPair(i, curr_ovrlp->id );
				DBGP("Added overlapping pair: " << i << " " << curr_ovrlp->id);
			}
		}
	}
  }
  return collTests->size;
}

int 
VCInternal::AllCollisions(Query q, VCReportType *vc_report, int size, 
						  int *interestList, int interestSize, int thisThreadId)
{
	//check whether we are in the right state.
	if (state != VCstate_default)
		return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
	// here we will store all the tests we want to make based on the overlapping pairs
	PairData collTests;
	GetEnabledList(&collTests, interestList, interestSize, thisThreadId);

	//go ahead now and perform the actual tests
	PairData report_data;
	for (int i=0; i<collTests.size; i++) {
		Elem *curr_test = collTests.arr[i];
		while (curr_test != NULL) {
			int nc = Collision(i,curr_test->id, &report_data);
			if (nc) {
				DBGP(i << " -- " << curr_test->id << " are colliding");
				if (q == FAST_COLLISION) return 1;
			} else {
				DBGP(i << " -- " << curr_test->id << " are NOT colliding");
			}
			curr_test = curr_test->next;
		}
	}
	if (q == FAST_COLLISION) return 0;
	assert(vc_report);
	return Report(size, vc_report, &report_data);		
}

int 
VCInternal::AllContacts(PQP_ContactResult *cresArr, double dist_thresh, 
						int *interestList, int interestSize, int thisThreadId)
{
	//check whether we are in the right state.
	if (state != VCstate_default)
		return VC_ERR_CALL_OUT_OF_SEQUENCE;
  
	int numContacts = 0;

	// here we will store all the tests we want to make based on the overlapping pairs
	PairData collTests;
	GetEnabledList(&collTests, interestList, interestSize, thisThreadId);

	//go ahead now and perform the actual tests
	for (int i=0; i<collTests.size; i++) {
		Elem *curr_test = collTests.arr[i];
		while (curr_test != NULL) {
			Contact(curr_test->id,i,dist_thresh,cresArr[numContacts] );
		    if (!cresArr[numContacts].contactSet.empty()) {
				cresArr[numContacts].body2Id = i;
				cresArr[numContacts].body1Id = curr_test->id;
				numContacts++;
			}
			curr_test = curr_test->next;
		}
	}
	return numContacts;
}

int VCInternal::Collision(int id1,int id2, PairData *report_data)
{
  //now, we need to call the PQP collision detection routine.			      
  double R1[3][3], T1[3], R2[3][3], T2[3];
  PQP_CollideResult cres;
			      
  //set up the rotation and translation matrices as
  //required by RAPID for the first object.
  R1[0][0] = vc_objects[id1]->trans[0][0];
  R1[0][1] = vc_objects[id1]->trans[0][1];
  R1[0][2] = vc_objects[id1]->trans[0][2];
  R1[1][0] = vc_objects[id1]->trans[1][0];
  R1[1][1] = vc_objects[id1]->trans[1][1];
  R1[1][2] = vc_objects[id1]->trans[1][2];
  R1[2][0] = vc_objects[id1]->trans[2][0];
  R1[2][1] = vc_objects[id1]->trans[2][1];
  R1[2][2] = vc_objects[id1]->trans[2][2];
  
  T1[0] = vc_objects[id1]->trans[0][3];
  T1[1] = vc_objects[id1]->trans[1][3];
  T1[2] = vc_objects[id1]->trans[2][3];
  
  DBGP("Body 1: " << T1[0] << " " << T1[1] << " " << T1[2]);

  //set up the rotation and translation matrices for
  //the second object. FIXME this is extremely lame.
  R2[0][0] = vc_objects[id2]->trans[0][0];
  R2[0][1] = vc_objects[id2]->trans[0][1];
  R2[0][2] = vc_objects[id2]->trans[0][2];
  R2[1][0] = vc_objects[id2]->trans[1][0];
  R2[1][1] = vc_objects[id2]->trans[1][1];
  R2[1][2] = vc_objects[id2]->trans[1][2];
  R2[2][0] = vc_objects[id2]->trans[2][0];
  R2[2][1] = vc_objects[id2]->trans[2][1];
  R2[2][2] = vc_objects[id2]->trans[2][2];
			      
  T2[0] = vc_objects[id2]->trans[0][3];
  T2[1] = vc_objects[id2]->trans[1][3];
  T2[2] = vc_objects[id2]->trans[2][3];
			
  DBGP("Body 2: " << T2[0] << " " << T2[1] << " " << T2[2]);

  DBGP("checking collisions between body " << id1 <<" and " << id2);
      
  //call the PQP collision detection routine.
  PQP_Collide(&cres, R1, T1, vc_objects[id1]->b, R2, T2, vc_objects[id2]->b,PQP_FIRST_CONTACT);

  if (cres.NumPairs() > 0) {
    report_data->AddPair(id1,id2);
	return 1;
  }
  return 0;
}

int VCInternal::Contact(int id1,int id2,double thresh,
						PQP_ContactResult &cres )
{
  //now, we need to call the PQP distance routine.			      
  double R1[3][3], T1[3], R2[3][3], T2[3];
			      
  //set up the rotation and translation matrices as
  //required by RAPID for the first object.
  R1[0][0] = vc_objects[id1]->trans[0][0];
  R1[0][1] = vc_objects[id1]->trans[0][1];
  R1[0][2] = vc_objects[id1]->trans[0][2];
  R1[1][0] = vc_objects[id1]->trans[1][0];
  R1[1][1] = vc_objects[id1]->trans[1][1];
  R1[1][2] = vc_objects[id1]->trans[1][2];
  R1[2][0] = vc_objects[id1]->trans[2][0];
  R1[2][1] = vc_objects[id1]->trans[2][1];
  R1[2][2] = vc_objects[id1]->trans[2][2];
  
  T1[0] = vc_objects[id1]->trans[0][3];
  T1[1] = vc_objects[id1]->trans[1][3];
  T1[2] = vc_objects[id1]->trans[2][3];
  
  //set up the rotation and translation matrices for
  //the second object.
  R2[0][0] = vc_objects[id2]->trans[0][0];
  R2[0][1] = vc_objects[id2]->trans[0][1];
  R2[0][2] = vc_objects[id2]->trans[0][2];
  R2[1][0] = vc_objects[id2]->trans[1][0];
  R2[1][1] = vc_objects[id2]->trans[1][1];
  R2[1][2] = vc_objects[id2]->trans[1][2];
  R2[2][0] = vc_objects[id2]->trans[2][0];
  R2[2][1] = vc_objects[id2]->trans[2][1];
  R2[2][2] = vc_objects[id2]->trans[2][2];
			      
  T2[0] = vc_objects[id2]->trans[0][3];
  T2[1] = vc_objects[id2]->trans[1][3];
  T2[2] = vc_objects[id2]->trans[2][3];
  
  //call the PQP contact routine
  DBGP("checking contacts between body " << id1 << " and " << id2);
  PQP_Contact(&cres, R1, T1, vc_objects[id1]->b, R2, T2, vc_objects[id2]->b, thresh);
  DBGP("   " << cres.contactSet.size() << " contacts.");

  return (int)cres.contactSet.size();
}

//report the results of collision detection.
//sz is the size of the array pointed to by vcrep. If sz is less than
//the number of collision pairs, then fill the array with first sz number
//of collision pairs.
//Returns the total number of collision pairs.
int VCInternal::Report(int sz, VCReportType *vcrep, PairData *report_data)
{
  int no_of_colliding_pairs=0;
  int vc_rep_count = 0;
  
  int i;
  for (i=0; i<report_data->size; i++)
    {
      Elem *current;
      for (current=report_data->arr[i]; current != NULL; current=current->next){
		no_of_colliding_pairs++;
		if (vc_rep_count <sz) {
			//if the array is not full yet, then 
			//fill the data in it.
			vcrep[vc_rep_count].id1 = i;
			vcrep[vc_rep_count].id2 = current->id;
			//fprintf(stderr,"%d and %d\n",vcrep[vc_rep_count].id1,vcrep[vc_rep_count].id2);
			vc_rep_count++;
		}
	  }      
    }
//	fprintf(stderr,"Report cols: %d\n",no_of_colliding_pairs);
  return no_of_colliding_pairs;
}

double VCInternal::Dist(int id1,int id2)
{
  //now, we need to call the PQP distance routine.			      
  double R1[3][3], T1[3], R2[3][3], T2[3];
  PQP_DistanceResult dres;
			      
  //set up the rotation and translation matrices as
  //required by RAPID for the first object.
  R1[0][0] = vc_objects[id1]->trans[0][0];
  R1[0][1] = vc_objects[id1]->trans[0][1];
  R1[0][2] = vc_objects[id1]->trans[0][2];
  R1[1][0] = vc_objects[id1]->trans[1][0];
  R1[1][1] = vc_objects[id1]->trans[1][1];
  R1[1][2] = vc_objects[id1]->trans[1][2];
  R1[2][0] = vc_objects[id1]->trans[2][0];
  R1[2][1] = vc_objects[id1]->trans[2][1];
  R1[2][2] = vc_objects[id1]->trans[2][2];
  
  T1[0] = vc_objects[id1]->trans[0][3];
  T1[1] = vc_objects[id1]->trans[1][3];
  T1[2] = vc_objects[id1]->trans[2][3];
  
  //set up the rotation and translation matrices for
  //the second object.
  R2[0][0] = vc_objects[id2]->trans[0][0];
  R2[0][1] = vc_objects[id2]->trans[0][1];
  R2[0][2] = vc_objects[id2]->trans[0][2];
  R2[1][0] = vc_objects[id2]->trans[1][0];
  R2[1][1] = vc_objects[id2]->trans[1][1];
  R2[1][2] = vc_objects[id2]->trans[1][2];
  R2[2][0] = vc_objects[id2]->trans[2][0];
  R2[2][1] = vc_objects[id2]->trans[2][1];
  R2[2][2] = vc_objects[id2]->trans[2][2];
			      
  T2[0] = vc_objects[id2]->trans[0][3];
  T2[1] = vc_objects[id2]->trans[1][3];
  T2[2] = vc_objects[id2]->trans[2][3];
			      
  //call the PQP distance routine
  PQP_Distance(&dres, R1, T1, vc_objects[id1]->b, R2, T2, vc_objects[id2]->b,
	       0.0, 0.0);

  return dres.Distance();
}

int VCInternal::Dist(int id1,int id2,PQP_DistanceResult &dres,
		     double dist_thresh)
{
  //now, we need to call the PQP distance routine.			      
  double R1[3][3], T1[3], R2[3][3], T2[3];
			      
  //set up the rotation and translation matrices as
  //required by RAPID for the first object.
  R1[0][0] = vc_objects[id1]->trans[0][0];
  R1[0][1] = vc_objects[id1]->trans[0][1];
  R1[0][2] = vc_objects[id1]->trans[0][2];
  R1[1][0] = vc_objects[id1]->trans[1][0];
  R1[1][1] = vc_objects[id1]->trans[1][1];
  R1[1][2] = vc_objects[id1]->trans[1][2];
  R1[2][0] = vc_objects[id1]->trans[2][0];
  R1[2][1] = vc_objects[id1]->trans[2][1];
  R1[2][2] = vc_objects[id1]->trans[2][2];
  
  T1[0] = vc_objects[id1]->trans[0][3];
  T1[1] = vc_objects[id1]->trans[1][3];
  T1[2] = vc_objects[id1]->trans[2][3];
  
  //set up the rotation and translation matrices for
  //the second object.
  R2[0][0] = vc_objects[id2]->trans[0][0];
  R2[0][1] = vc_objects[id2]->trans[0][1];
  R2[0][2] = vc_objects[id2]->trans[0][2];
  R2[1][0] = vc_objects[id2]->trans[1][0];
  R2[1][1] = vc_objects[id2]->trans[1][1];
  R2[1][2] = vc_objects[id2]->trans[1][2];
  R2[2][0] = vc_objects[id2]->trans[2][0];
  R2[2][1] = vc_objects[id2]->trans[2][1];
  R2[2][2] = vc_objects[id2]->trans[2][2];
			      
  T2[0] = vc_objects[id2]->trans[0][3];
  T2[1] = vc_objects[id2]->trans[1][3];
  T2[2] = vc_objects[id2]->trans[2][3];
			      
  //call the PQP distance routine
  return PQP_Distance(&dres, R1, T1, vc_objects[id1]->b, R2, T2,
		      vc_objects[id2]->b, 0.0, 0.0, 2, dist_thresh);
}

//cnl finds the region on a given model that is within a distance threshold 
//to a given point, current calls PQP::GetNghbdPts which gives the region
//by returning triangles within a distance threshold
//it could also call GetAdjNghbdPts which gets vertices from a set of
//adjacent triangles
void VCInternal::FindRegion( int id1, PQP_REAL pt[3], PQP_REAL normal[3], PQP_REAL rad, 
						   std::vector<PQP_Vec> *ptList)
{
   double R1[3][3], T1[3];
			      
  //set up the rotation and translation matrices
  //from world to body
/*
  R1[0][0] = vc_objects[id1]->trans[0][0];
  R1[0][1] = vc_objects[id1]->trans[0][1];
  R1[0][2] = vc_objects[id1]->trans[0][2];
  R1[1][0] = vc_objects[id1]->trans[1][0];
  R1[1][1] = vc_objects[id1]->trans[1][1];
  R1[1][2] = vc_objects[id1]->trans[1][2];
  R1[2][0] = vc_objects[id1]->trans[2][0];
  R1[2][1] = vc_objects[id1]->trans[2][1];
  R1[2][2] = vc_objects[id1]->trans[2][2];
  
  T1[0] = vc_objects[id1]->trans[0][3];
  T1[1] = vc_objects[id1]->trans[1][3];
  T1[2] = vc_objects[id1]->trans[2][3];
*/
   //cnl-currently assuming that points are given in body frame
   Midentity( R1 );
   Videntity( T1 );
  
  GetNghbdPts( pt, vc_objects[id1]->b, R1, T1, normal, rad, ptList );
}

double VCInternal::FindShortDist(PQP_REAL pt[3], int id1, 
								 PQP_REAL closest_pt[3], PQP_REAL closest_normal[3], PQP_REAL thresh)
{
	PQP_REAL R[3][3];
	PQP_REAL T[3];

  //set up the rotation and translation matrices
  //from world to body
	R[0][0] = vc_objects[id1]->trans[0][0];
	R[0][1] = vc_objects[id1]->trans[0][1];
	R[0][2] = vc_objects[id1]->trans[0][2];
	R[1][0] = vc_objects[id1]->trans[1][0];
	R[1][1] = vc_objects[id1]->trans[1][1];
	R[1][2] = vc_objects[id1]->trans[1][2];
	R[2][0] = vc_objects[id1]->trans[2][0];
	R[2][1] = vc_objects[id1]->trans[2][1];
	R[2][2] = vc_objects[id1]->trans[2][2];
  
	T[0] = vc_objects[id1]->trans[0][3];
	T[1] = vc_objects[id1]->trans[1][3];
	T[2] = vc_objects[id1]->trans[2][3];

	//Midentity(R);
	//Videntity(T);
	 
	// this can be used to request "shortest distance than x"
	// set it to a huge value to get the general shortest distance
	double closest_dist = 1.0e9;
	return GetShortestDist(pt, vc_objects[id1]->b, R, T, 
						   closest_dist, closest_pt, closest_normal, thresh);
}

void VCInternal::getBvs(int id, int desiredDepth, std::vector<BV*> *bvs)
{
	vc_objects[id]->b->getBvs(desiredDepth, bvs);
}


