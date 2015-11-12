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
Filename: NBody.C
--
Description: This file implements the member functions of the class NBody.

\************************************************************************/

#include <iostream>
#include <math.h>
//#include "obb.H"
#include "NBody.H"
//#include "dmalloc.h"
#include <string.h>

const int REVERSE  = 1;//whether the direction of movement of the interval
const int FORWARD  = 2;//along the list is forward (FORWARD), reverse (REVERSE)
const int NOCHANGE = 3;//or there is no movement at all (NOCHANGE).

const int DEFAULT_SIZE=10; //default number of AABBs. Arbitrarily set here to
                           //10.

inline double GT(double a, double b)
{
  return (( (a) > (b) ) ? (a) : (b));
}

void
copyAABB(AABB *curr, const AABB *orig)
{
	curr->radius = orig->radius;
	memcpy( curr->center, orig->center, 3*sizeof(double) );
	curr->hi = new EndPoint;
	curr->hi->aabb = curr;
	curr->hi->minmax = MAX;
	memcpy(curr->hi->val, orig->hi->val, 3*sizeof(double) ); 
	curr->lo = new EndPoint;
	curr->lo->aabb = curr;
	curr->lo->minmax = MIN;
	memcpy(curr->lo->val, orig->lo->val, 3*sizeof(double) ); 
}

NBody::NBody()  //constructor.
{
  elist[0] = NULL;
  elist[1] = NULL;
  elist[2] = NULL;
  size = DEFAULT_SIZE;
  AABB_arr = new AABB*[size];  //allocate the dynamic array and initialize
  int i;
  for (i=0; i<size; i++)      //all its elements to NULL.
    AABB_arr[i] = NULL;
  
}

NBody::NBody(const NBody *original) //copy constructor
{
	elist[0] = NULL;
	elist[1] = NULL;
	elist[2] = NULL;

	size = original->size;
	AABB_arr = new AABB*[size];
	for (int i=0; i<size; i++) {
		//copy bbox array
		if ( original->AABB_arr[i] == NULL ) {
			AABB_arr[i] = NULL;
			continue;
		}
	
		AABB *curr = new AABB;
		AABB *orig = original->AABB_arr[i];
		curr->id = i;
		copyAABB(curr, orig);
		AABB_arr[i] = curr;
			for (int j=0; j<i; j++) {
			if (AABB_arr[j] && overlaps(AABB_arr[i], AABB_arr[j]) ) {
				add_overlap_pair(i,j);
			}
		}
		InsertInLists(AABB_arr[i]);
	}
}

NBody::~NBody()  //destructor.
{
  int i;
  for (i=0; i<size; i++)
    {
      if (AABB_arr[i])
		{
			delete AABB_arr[i]->hi;
			delete AABB_arr[i]->lo;
			delete AABB_arr[i];
			AABB_arr[i] = NULL;
		}
    }

  delete [] AABB_arr;
}


int NBody::overlaps(AABB *obj1, AABB *obj2) //to check it the two AABBs overlap.
{
  int coord;
  for (coord=0; coord<3; coord++)
    {
      if (obj1->lo->val[coord] < obj2->lo->val[coord])
	{
	  if (obj2->lo->val[coord] > obj1->hi->val[coord])
	    return 0;
	}
      else
	{
	  if (obj1->lo->val[coord] > obj2->hi->val[coord])
	    return 0;
	}
    }
  
  return 1;
}


void NBody::AddObject(int id, const PQP_Model *b) //add a new object
{
  AABB *curr = new AABB;
  
  curr->id = id; //set the id to the given value.
  
  //The centroid of the object is computed and this is taken to be the
  //center of the AABB.
  curr->center[0] = curr->center[1] = curr->center[2] = 0.0;
  
  int i;
  for (i=0; i<(b->num_tris); i++)
    {
      curr->center[0] += b->tris[i].p1[0] + b->tris[i].p2[0] + b->tris[i].p3[0];
      curr->center[1] += b->tris[i].p1[1] + b->tris[i].p2[1] + b->tris[i].p3[1];
      curr->center[2] += b->tris[i].p1[2] + b->tris[i].p2[2] + b->tris[i].p3[2];
    }
  
  curr->center[0] /= (3*b->num_tris); 
  curr->center[1] /= (3*b->num_tris);
  curr->center[2] /= (3*b->num_tris);
  

  //The "radius" of the AABB is computed as the maximum distance of the AABB
  //center from any of the vertices of the object.
  curr->radius = 0.0;

  for (i=0; i<(b->num_tris); i++)
    {
      double cur_rad1_sq = (
			    (curr->center[0] - b->tris[i].p1[0])
			    *
			    (curr->center[0] - b->tris[i].p1[0])
			    
			    +
			    
			    (curr->center[1] - b->tris[i].p1[1])
			    *
			    (curr->center[1] - b->tris[i].p1[1])
			    
			    +
			    
			    (curr->center[2] - b->tris[i].p1[2])
			    *
			    (curr->center[2] - b->tris[i].p1[2])
			    );
      

      double cur_rad2_sq = (
			    (curr->center[0] - b->tris[i].p2[0])
			    *
			    (curr->center[0] - b->tris[i].p2[0])
			    
			    +
			    
			    (curr->center[1] - b->tris[i].p2[1])
			    *
			    (curr->center[1] - b->tris[i].p2[1])
			    
			    +
			    
			    (curr->center[2] - b->tris[i].p2[2])
			    *
			    (curr->center[2] - b->tris[i].p2[2])
			    );

      double cur_rad3_sq = (
			    (curr->center[0] - b->tris[i].p3[0])
			    *
			    (curr->center[0] - b->tris[i].p3[0])
			    
			    +
			    
			    (curr->center[1] - b->tris[i].p3[1])
			    *
			    (curr->center[1] - b->tris[i].p3[1])
			    
			    +
			    
			    (curr->center[2] - b->tris[i].p3[2])
			    *
			    (curr->center[2] - b->tris[i].p3[2])
			    );
      
      
      
      double max_rad_sq = GT(cur_rad1_sq, GT(cur_rad2_sq,cur_rad3_sq));
      
      curr->radius = GT(max_rad_sq, curr->radius);
      
    }

  curr->radius = sqrt(curr->radius);
  
  curr->radius *= 1.0001;  //add a 0.01% buffer.

  double min[3], max[3];
  
  min[0] = curr->center[0] - curr->radius; //min holds the min endpoints 
  max[0] = curr->center[0] + curr->radius; //of the three intervals and
  min[1] = curr->center[1] - curr->radius; //max holds the max endpoints.
  max[1] = curr->center[1] + curr->radius;
  min[2] = curr->center[2] - curr->radius;
  max[2] = curr->center[2] + curr->radius;
  
  curr->lo = new EndPoint;    //set up the two EndPoints data structures 
                              //for the AABB.
  curr->lo->minmax = MIN;
  curr->lo->val[0] = min[0];
  curr->lo->val[1] = min[1];
  curr->lo->val[2] = min[2];
  curr->lo->aabb = curr;

  curr->hi = new EndPoint;
  
  curr->hi->minmax = MAX;
  curr->hi->val[0] = max[0];
  curr->hi->val[1] = max[1];
  curr->hi->val[2] = max[2];
  curr->hi->aabb = curr;

  for (i=0; i<size; i++)      //Now, check the overlap of this AABB with 
    {                         //with all other AABBs and add the pair to
      if (AABB_arr[i])        //the set of overlapping pairs if reqd.
		if (overlaps(curr, AABB_arr[i]))
		  add_overlap_pair(curr->id, i);
    }

  if (id >= size)    //increase the size of the dynamic array if necessary.
      {
		int newsize = (id >= 2*size) ? (id+1) : 2*size;

		AABB **temp = new AABB*[newsize];
		int i;
		for (i=0; i<size; i++)
			temp[i] = AABB_arr[i];
		for (i=size; i<newsize; i++)
			temp[i] = NULL;
		delete [] AABB_arr;
		AABB_arr = temp;
		size = newsize;
      }
  
  AABB_arr[id] = curr;  //finally, insert the AABB in AABB_arr.
  InsertInLists(curr);   //and into the linked lists

}
void
NBody::InsertInLists(AABB *curr)
{
  //For each of the three co-ordinates, insert the interval
  //in the correspoding list. 
  int coord;
  for (coord=0; coord <3; coord++)
    {
      EndPoint *current = elist[coord];
      
      //first insert the "hi" endpoint.
      if (current == NULL)    //if the list is empty, insert in front.
	{
	  elist[coord] = curr->hi;
	  curr->hi->prev[coord] = curr->hi->next[coord] = NULL;
	}
      else  //otherwise, find the correct location in the list and
	{   //insert there. Note: the list is sorted.
	  while ( (current->next[coord] != NULL) && (current->val[coord] < curr->hi->val[coord]) )
	    current = current->next[coord];
	  
	  
	  if (current->val[coord] >= curr->hi->val[coord])
	    {
	      curr->hi->prev[coord] = current->prev[coord];
	      curr->hi->next[coord] = current;
	      if (current->prev[coord] == NULL)
		elist[coord] = curr->hi;
	      else
		current->prev[coord]->next[coord] = curr->hi;
	      
	      current->prev[coord] = curr->hi;
	    }
	  else
	    {
	      curr->hi->prev[coord] = current;
	      curr->hi->next[coord] = NULL;
	      current->next[coord] = curr->hi;
	    }
	}
      
      //now, insert the "lo" endpoint.
      current = elist[coord];
      
      //at this point, the list cannot be empty since we have already 
      //inserted the "hi" endpoint. So, we straightaway look for the 
      //correct location in the non-empty list and insert at that location.
      while ( (current->next[coord] != NULL) && (current->val[coord] < curr->lo->val[coord]) )
	current = current->next[coord];
      
      if (current->val[coord] >= curr->lo->val[coord])
	{
	  curr->lo->prev[coord] = current->prev[coord];
	  curr->lo->next[coord] = current;
	  if (current->prev[coord] == NULL)
	    elist[coord] = curr->lo;
	  else
	    current->prev[coord]->next[coord] = curr->lo;
	  
	  current->prev[coord] = curr->lo;
	}
      else
	{
	  curr->lo->prev[coord] = current;
	  curr->lo->next[coord] = NULL;
	  current->next[coord] = curr->lo;
	}
      
    }
  
}


void NBody::UpdateTrans(int id, double trans[][4]) //update the transformation
{                                                  //matrix and linked lists.
  if (id>=size)
    {
      //cerr<<"NBody::update_trans - No polytope with id = "<<id<<"\n";
      return;
    }
  else if (AABB_arr[id] == NULL)
    {
      //cerr<<"NBody::update_trans - No polytope with id = "<<id<<"\n";
      return;
    }
  
  AABB *current = AABB_arr[id]; //the given object exists !
  
  
  //compute the new position of the AABB center.
  double new_center[3]; 
  new_center[0] = current->center[0] * trans[0][0] + current->center[1] * trans[0][1] + current->center[2] * trans[0][2] + trans[0][3];
  new_center[1] = current->center[0] * trans[1][0] + current->center[1] * trans[1][1] + current->center[2] * trans[1][2] + trans[1][3];
  new_center[2] = current->center[0] * trans[2][0] + current->center[1] * trans[2][1] + current->center[2] * trans[2][2] + trans[2][3];
  
  
  //compute the new min and max endpoints.
  double min[3], max[3];
  min[0] = new_center[0] - current->radius;
  max[0] = new_center[0] + current->radius;
  min[1] = new_center[1] - current->radius;
  max[1] = new_center[1] + current->radius;
  min[2] = new_center[2] - current->radius;
  max[2] = new_center[2] + current->radius;
  
  AABB dummy;       //we need these so that we can use the same function
  EndPoint lo, hi;  //NBody::overlaps(AABB *, AABB *) to check overlap
                    //of the newly transformed object with other objects.
  dummy.lo = &lo;
  dummy.hi = &hi;
  
  lo.minmax = MIN;
  lo.val[0] = min[0];
  lo.val[1] = min[1];
  lo.val[2] = min[2];
  lo.aabb = &dummy;

  hi.minmax = MAX;
  hi.val[0] = max[0];
  hi.val[1] = max[1];
  hi.val[2] = max[2];
  hi.aabb = &dummy;


  //update all the three lists by moving the endpoint to correct position.
  int coord;
  for (coord=0; coord<3; coord++)
    {
      int direction;
      EndPoint *temp;
      
      //set the direction of motion of the endpoint along the list.
      if (current->lo->val[coord] > min[coord])
		direction = REVERSE;
      else if (current->lo->val[coord] <min[coord])
		direction = FORWARD;
      else
		direction = NOCHANGE;
      
      if (direction == REVERSE) //backward motion....
	{
	  //first update the "lo" endpoint of the interval
	  if (current->lo->prev[coord] != NULL)
	    {
	      temp = current->lo;
	      while ((temp != NULL) && (temp->val[coord] > min[coord]))
		{
		  if (temp->minmax == MAX)
		    if (overlaps(temp->aabb, &dummy))
		      add_overlap_pair(temp->aabb->id, current->id);
		  
		  temp = temp->prev[coord];
		}
	      
	      if (temp == NULL)
		{
		  current->lo->prev[coord]->next[coord] = current->lo->next[coord];
		  current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
		  current->lo->prev[coord] = NULL;
		  current->lo->next[coord] = elist[coord];
		  elist[coord]->prev[coord] = current->lo;
		  elist[coord] = current->lo;
		}
	      else
		{
		  current->lo->prev[coord]->next[coord] = current->lo->next[coord];
		  current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
		  current->lo->prev[coord] = temp;
		  current->lo->next[coord] = temp->next[coord];
		  temp->next[coord]->prev[coord] = current->lo;
		  temp->next[coord] = current->lo;
		}
	      
	    }
	  
	  current->lo->val[coord] = min[coord];
	  
	  //then update the "hi" endpoint of the interval.
	  if (current->hi->val[coord] != max[coord])
	    {
	      temp = current->hi;
	      
	      while (temp->val[coord] > max[coord])
		{
		  if ( (temp->minmax == MIN) && (overlaps(temp->aabb, current)) )
		    del_overlap_pair(temp->aabb->id, current->id);
		  
		  temp = temp->prev[coord];
		  
		}
	      
	      current->hi->prev[coord]->next[coord] = current->hi->next[coord];
	      if (current->hi->next[coord] != NULL)
			current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
	      current->hi->prev[coord] = temp;
	      current->hi->next[coord] = temp->next[coord];
	      if (temp->next[coord] != NULL)
			temp->next[coord]->prev[coord] = current->hi;
	      temp->next[coord] = current->hi;
	      
	      current->hi->val[coord] = max[coord];
	    }
	}
      else if (direction == FORWARD) //forward motion....
	{
	  //here, we first update the "hi" endpoint.
	  if (current->hi->next[coord] != NULL)
	    {
	      temp = current->hi;
	      while ( (temp->next[coord] != NULL) && (temp->val[coord] < max[coord]) )
		{
		  if (temp->minmax == MIN)
		    if (overlaps(temp->aabb, &dummy))
		      add_overlap_pair(temp->aabb->id, current->id);
		  
		  temp = temp->next[coord];
		}
	      
	      if (temp->val[coord] < max[coord])
		{
		  current->hi->prev[coord]->next[coord] = current->hi->next[coord];
		  current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
		  current->hi->prev[coord] = temp;
		  current->hi->next[coord] = NULL;
		  temp->next[coord] = current->hi;
		}
	      else if (current->hi->val[coord] != max[coord])
		{
		  current->hi->prev[coord]->next[coord] = current->hi->next[coord];
		  current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
		  current->hi->prev[coord] = temp->prev[coord];
		  current->hi->next[coord] = temp;
		  temp->prev[coord]->next[coord] = current->hi;
		  temp->prev[coord] = current->hi;
		}
	    }
	  current->hi->val[coord] = max[coord];
	  
	  //then, update the "lo" endpoint of the interval.
	  temp = current->lo;
	  
	  while (temp->val[coord] < min[coord])
	    {
	      if ( (temp->minmax == MAX) && (overlaps(temp->aabb, current)) )
		del_overlap_pair(temp->aabb->id, current->id);
	      
	      temp = temp->next[coord];
	    }
	  
	  if (current->lo->prev[coord] != NULL)
	    current->lo->prev[coord]->next[coord] = current->lo->next[coord];
	  else
	    elist[coord] = current->lo->next[coord];
	  current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
	  current->lo->prev[coord] = temp->prev[coord];
	  current->lo->next[coord] = temp;
	  if (temp->prev[coord] != NULL)
	    temp->prev[coord]->next[coord] = current->lo;
	  else
	    elist[coord] = current->lo;
	  temp->prev[coord] = current->lo;
	  current->lo->val[coord] = min[coord];
	}
      
    }
  
}

void NBody::DeleteObject(int id) //deleting an AABB with given id.
{
  if (id >= size)
    {
      //cerr<<"Should not get here since VCollide should send only valid ids\n";
      return;
    }
  
  if (AABB_arr[id] == NULL)
    {
      //cerr<<"Should not get here since VCollide should send only valid ids\n";
      return;
    }
  
  AABB *curr = AABB_arr[id];  //this is the AABB to be deleted.
  AABB_arr[id] = NULL;        //remove it from the AABB array.
  
  //first, we delete all the three intervals from the corresponding lists.
  int coord;
  for (coord=0; coord<3; coord++)
    {
      //first delete the "lo" endpoint of the interval.
      if (curr->lo->prev[coord] == NULL)
	elist[coord] = curr->lo->next[coord];
      else
	curr->lo->prev[coord]->next[coord] = curr->lo->next[coord];
      
      curr->lo->next[coord]->prev[coord] = curr->lo->prev[coord];
      
      //then, delete the "hi" endpoint.
      if (curr->hi->prev[coord] == NULL)
	elist[coord] = curr->hi->next[coord];
      else
	curr->hi->prev[coord]->next[coord] = curr->hi->next[coord];
      
      if (curr->hi->next[coord] != NULL)
	curr->hi->next[coord]->prev[coord] = curr->hi->prev[coord];
      
    }
  
  //delete all entries involving this id from the set of 
  //overlapping pairs.
  overlapping_pairs.DelPairsInvolvingId(id);
  
  //de-allocate the memory
  delete curr->lo;
  delete curr->hi;
  delete curr;
}
