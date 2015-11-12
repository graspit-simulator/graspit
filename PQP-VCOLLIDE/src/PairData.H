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
Filename: PairData.H
--
Description: This file declares two classes - Elem and
             PairData. Elem is a simple class used by
             PairData. PairData stores a set of pairs 
             of integers.

\************************************************************************/


#ifndef PAIRDATA_H
#define PAIRDATA_H

#include "PQP.h"

class Elem
{
public:
  int   id;
  Elem  *next;
};




/************************************************************************
Class: PairData
--
Description: Each instance of this class stores a set of pairs of
             integers. It is assumed that the data is sparse
             (i.e. the size of this set is small as compared to
             the n*(n-1)/2 possible pairs).

             It should be noted that none of the member functions
             of PairData return any error status. For eg., it we
             try deleting a pair which doesn't exist, the Delete
             operation will simply return.

\************************************************************************/

class PairData
{

private:
  inline void OrderIds(int& id1, int& id2);
  
public:
  int    size;   //the size of array pointed to by "arr"
  Elem   **arr;  //arr points to a dynamic array. Each element of
                 //this array points to a linked list of
                 //"Elem"s.
                 //pair (id1, id2) exists <==> max(id1, id2)
                 //exists in the linked list pointed to by
                 //arr[min(id1, id2)].
  
  PairData();
  ~PairData();
  
 //add a pair of ids to the set.
  void   AddPair(int id1, int id2);

  void   DelPair(int id1, int id2);     //delete a pair from the set.
  void   DelPairsInvolvingId(int id);   //delete all pairs containing id.
  void   Clear(void);                   //empty the set.
  int    ExistsPair(int id1, int id2);  //check if a pair of ids exists
                                        //in the given set.
  
};

inline void PairData::OrderIds(int &id1, int& id2) //ensures that
{                                                  //id1 = min(id1,id2) and
  if (id1 > id2)                                   //id2 = max(id1, id2)
    {
      int temp = id1;
      id1 = id2;
      id2 = temp;
    }
}


#endif /* PAIRDATA_H */
