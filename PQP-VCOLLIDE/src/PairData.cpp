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
Filename: PairData.C
--
Description: This file defines the member functions of the
             class PairData, declared in file PairData.H

\************************************************************************/


#include <iostream>
#include <stdio.h>
#include "PairData.H"
//#include "dmalloc.h"


const int PAIRDATA_DEFAULT_SIZE=10;


PairData::PairData()
{
  size = PAIRDATA_DEFAULT_SIZE;
  arr = new Elem*[PAIRDATA_DEFAULT_SIZE];
  int i;
  for (i=0;i<PAIRDATA_DEFAULT_SIZE;i++)
    arr[i] = NULL;
}

PairData::~PairData()
{
  int i;
  for (i=0; i<size; i++)
    {
      Elem *current = arr[i];
      while (current != NULL)
	{
	  Elem *temp = current;
	  current = current->next;
	  delete temp;
	}
    }
  
  delete [] arr;
}


void PairData::AddPair(int id1, int id2)
{
  int i;
  
  OrderIds(id1, id2);  //order the ids
  
  if (id1 >= size)     //increase the size of "arr", if necessary.
    {
      int newsize = (id1 >= 2*size) ? (id1+1) : 2*size;
      
      Elem **temp = new Elem*[newsize];
      for (i=0; i<size; i++)
	temp[i] = arr[i];
      for (i=size; i<newsize; i++)
	temp[i] = NULL;
      delete [] arr;
      arr = temp;
      size = newsize;
    }
  
  Elem *current = arr[id1]; //select the right list from "arr".
  
  if (current == NULL)      //if the list is empty, insert the
    {                       //element in the front.
      current = new Elem;
      current->id = id2;
      current->next = NULL;
      arr[id1] = current;
    }
  else if (current->id > id2) //if the list is not empty but all
    {                         //elements are greater than id2, then
      current = new Elem;     //insert id2 in the front.
      current->id = id2;
      current->next = arr[id1];
      arr[id1] = current;
    }
  else
    {
      while (current->next != NULL)    //otherwise, find the correct location
	{                              //in the sorted list (ascending order) 
	  if (current->next->id > id2) //and insert id2 there.
	    break;
	  current = current->next;
	}
      if (current->id == id2)
	{
	  return;
	}
      else
	{
	  Elem *temp = new Elem;
	  temp->id = id2;
	  temp->next = current->next;
	  current->next = temp;
	}
    }
  
}
  
void PairData::DelPair(int id1, int id2) //delete a pair from the set.
{
  OrderIds(id1, id2); //order the ids.
  
  if (id1 >= size)    //the pair doesnot exist in the set. So, do nothing
    return;           //but return.
  
  Elem *current = arr[id1]; //otherwise, select the correct list.
  
  if (current == NULL) //if this list is empty, the pair doesn't exist.
    {                  //so, return. 
      return;
    }
  else if (current->id == id2)   //otherwise, if id2 is the first element, 
    {                            //delete it.
      arr[id1] = current->next;
      delete current;
      return;
    }
  else
    {
      while (current->next != NULL)     //if id2 is not the first element,
	{                               //start traversing the sorted list.
	  
	  if (current->next->id > id2)  //if you have moved too far away
	    {                           //without hitting id2, then the pair
	      return;                   //pair doesn't exist. So, return.
	    }
	  else if (current->next->id == id2)  //otherwise, delete id2.
	    {
	      Elem *temp = current->next;
	      current->next = current->next->next;
	      delete temp;
	      return;
	    }
	  current = current->next;
	}
    }
}

void PairData:: DelPairsInvolvingId(int id)  //delete all pairs containing id.
{
  if (id < size)
    {
      Elem *temp = arr[id];
      while (temp != NULL)
	{
	  Elem *t = temp;
	  temp = temp->next;
	  delete t;
	}
      arr[id] = NULL;
      
      int i;
      for (i=0; i<id; i++)
	DelPair(i, id);
    }
  else
    {
      int i;
      for (i=0;i<size; i++)
	DelPair(i, id);
    }
}


void PairData::Clear(void)     //delete all pairs from the set.
{
  int i;
  for (i=0; i<size; i++)
    {
      while (arr[i] != NULL)
	{
	  Elem *current = arr[i];
	  arr[i] = current->next;
	  delete current;
	}
    }
};

int PairData::ExistsPair(int id1, int id2)  //check if a pair exists in the
{                                           //set.
  OrderIds(id1, id2);      //order the ids.
  
  if (id1 >=size)    //if id1 >= size, then the pair cannot exist.
    return 0;
  
  Elem *current = arr[id1];  //otherwise, find the correct list and traverse
  while (current != NULL)    //it, looking for id2.
    {
      if (current->id == id2)
	return 1;
      if (current->id > id2)
	return 0;
      
      current = current->next;
    }
  return 0;
}
