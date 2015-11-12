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
Filename: EndPoint.H
--
Description: This file defines the class EndPoint, which stores
             information about either the (min_x, min_y, min_z)
             or the (max_x, max_y, max_z) end-point of an
             AABB.
\************************************************************************/

#ifndef ENDPOINT_H
#define ENDPOINT_H

class AABB;  //defined in file AABB.H

const int MIN=1;
const int MAX=2;

/************************************************************************\
Class: EndPoint
--
Description: Each instance of EndPoint appears simultaneously in
             three sorted linked lists, one for each dimension,
             as required by the sweep and prune algorithm. A
             "min" and the correspoding "max" end-point give us
             three intervals, one for each axis, which are
             actually the projections of the AABB on each of the
             three co-ordinate axii.

\************************************************************************/



class EndPoint
{
public:
  char        minmax;     //whether the instance represents a
                          //"min" or a "max" end-point.
  
  double      val[3];     //the coordinates of the EndPoint.
  EndPoint    *prev[3];   //for maintaining the three linked
  EndPoint    *next[3];   //lists.

  AABB        *aabb;      //back pointer to the parent AABB.

};

#endif /* ENDPOINT_H */
