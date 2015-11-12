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
Filename: NBody.H
--
Description: This file declares the class NBody.

\************************************************************************/

#ifndef NBODY_H
#define NBODY_H

#include "PQP.h"
#include "AABB.H"
#include "EndPoint.H"
#include "PairData.H"

/************************************************************************\
Class: NBody
--
Description: The class NBody forms the core of VCollide. It
             maintains a set of three linked lists and keeps
             updating them using the sweep and prune
             algorithm. At any point in execution, the class
             NBody maintains a set of pairs of objects whose
             AABBs overlap. VCollide uses this set while making
             collision calls to RAPID. In general, this set would
             be very small compared to the n(n-1)/2 possible
             pairs.

\************************************************************************/



class NBody
{
private:

  EndPoint *elist[3];  //pointers to the three linked lists, one for
                       //each axis.
  int      size;       //the size of dynamic array - AABB_arr
  AABB     **AABB_arr; //AABB_arr points to a dynamic array of pointers.
                       //Element at index "i" in this array points to an
                       //AABB corresponding to the object with id = i. If
                       //this object doesn't exist, then the pointer is NULL.
  
  
  int overlaps(AABB *obj1, AABB *obj2);   //to check if two AABBs overlap.
  
  void add_overlap_pair(int id1, int id2) //add a pair to the set of
    {                                     //overlapping pairs.
      if (id1 != id2)
		overlapping_pairs.AddPair(id1, id2);
    }
  
  void del_overlap_pair(int id1, int id2) //delete a pair from the set.
    {
      if (id1 != id2)
		overlapping_pairs.DelPair(id1, id2);
    }
  
  void InsertInLists(AABB *curr); //insert a new AABB into the linked lists

public:  
  PairData overlapping_pairs; //at any instance during the execution of the
                              //program, overlapping_pairs contains the set
                              //of pairs of ids whose AABBs overlap.

  NBody();                    //constructor.
  NBody(const NBody*);			  //copy constructor
  ~NBody();                   //destructor.
  
  void AddObject(int id, const PQP_Model *b); //add an object to the NBody data
                                          //structure.
  void UpdateTrans(int id, double trans[][4]); //update the current position
                                               //and orientation of the object.
  void DeleteObject(int id);  //delete the object from the data structure.
  
};

#endif  /* NBODY_H */
