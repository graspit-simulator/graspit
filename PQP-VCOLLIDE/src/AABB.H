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
Filename: AABB.H
--
Description: This file defines the class AABB, which is used to
             store information about an axis aligned bounding
             box.

\************************************************************************/

#ifndef AABB_H
#define AABB_H

class EndPoint; //defined in file EndPoint.H

/************************************************************************\
Class: AABB
--
Description: In our implementation, an AABB has the same "radius"
             along all the three axii. As a result of this, even
             if the object rotates about some arbitrary point and
             axis, the radius of the enclosing AABB need not be
             changed.

             Computing the center and radius of the AABBs:

             Let P1, P2, ... , Pn be the position vectors of the
             end-points of the given object. Then, the center of
             the enclosing AABB is the centroid of the object.

             center = (P1 + P2 + ... + Pn) / n


             The radius is given by:

             radius = max_i (dist(Pi, center))

\************************************************************************/

class AABB
{
public:
  AABB(){}

  int          id;             // the id of the enclosed object
  double       center[3];      // center of the AABB
  double       radius;         // radius of the AABB
  EndPoint     *lo;            // the (min_x, min_y, min_z) and  
  EndPoint     *hi;            // (max_x, max_y, max_z) corners of
                               // the AABB. These point to
                               // the corresponding nodes in
                               // the NBody linked lists.
    
};         

#endif /* AABB_H */
