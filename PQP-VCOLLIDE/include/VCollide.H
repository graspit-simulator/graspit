#include <stdlib.h>
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
Filename: VCollide.H
--
Description: This file declares the class - VCollide.

\************************************************************************/


#ifndef VCOLLIDE_H
#define VCOLLIDE_H

//error codes returned by VCollide API.
//these are multiply defined in the files VInternal.C and VCollide.h
//so, any changes to these need to be reflected in all three places.
const int VC_ERR_INVALID_ID            = -4; //invalid id was passed to the
                                             //routine.
const int VC_ERR_EMPTY_OBJECT          = -3;//EndObject called without adding
                                             //adding any triangles.
const int VC_ERR_CALL_OUT_OF_SEQUENCE  = -2; //calls out of sequence.
const int VC_ERR                       = -1; //some other error.
const int VC_OK                        =  1; //No error.



class VCInternal; //declared in file VInternal.h

class PQP_Model;

#include "VInternal.H"

/************************************************************************
Struct: VCReportType
--
Description: Used for reporting collisions.

\************************************************************************/
//multiply defined in files VInternal.C and VCollide.h
#ifndef _VCREPORTTYPE
#define _VCREPORTTYPE

#include "PQP.h"

struct VCReportType
{
  int id1, id2;
};
#endif


/************************************************************************
Class: VCollide
--
Description: This implements the VCollide API. This is the only class
             visible to the users of VCollide. It is just the
             user interface and hides all the implementation
             details. Implementation details can be found in the
             class VCInternal. The member functions of the class
             VCollide just make calls to the corresponding member
             functions of class VCInternal.

\************************************************************************/

class VCollide
{
private:  
  VCInternal *vcint;

public:
  VCollide();             //constructor
  ~VCollide();            //destructor

  int ResetObject(int id); //create a new object using an old id

  int NewObject(int *id); //create a new object in the database.
  int AddTri(double v1[3], double v2[3], double v3[3],int triInd); //insert the
                                                        //geometry .
  int EndObject(void);    //tell VCollide that inserting the 
                          //geometry is complete.
  int EditObject(int id); //begin changing object geometry
  int EditTri(double v1[], double v2[], double v3[],int triInd); //change
                                                     // the geometry a triangle

  int EditFinished(void); //tell VCollide the geometry changes are done

  int GetObject(int id,double **& vList);  // return # of triangles and all the
                                         // transformed vertices

  PQP_Model *GetModel(int id);
  int UpdateTrans(int id, double trans[4][4]);
                          //update the transformation matrix of
                          //the object.
  int ActivateObject(int id);          //activate for collision detection.
  int DeactivateObject(int id);        //deactivate from collision detection.
  int ActivatePair(int id1, int id2);  //activate the pair.
  int DeactivatePair(int id1, int id2);//deactivate the pair.

  bool isObjectActivated(int id);
  bool isPairActivated(int id1, int id2);

  int DeleteObject(int id);//delete the object from the database.

  int Collide();       //perform collision detection.
  int FindAllContacts(double dist_thresh,PQP_ContactResult *cresArr);  
  double Dist(int id1, int id2);
  int Dist(int id1, int id2,PQP_DistanceResult &dres,double dist_thresh=-1);
  int Contact(int id1, int id2,double thresh,PQP_ContactResult &cres);
  int Report(int size, VCReportType *vcrep);
                           //report the results of collision detection.

};


#endif
