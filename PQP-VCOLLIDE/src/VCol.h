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
Filename: VCol.h
--
Description: This file declares the plain C wrappers for the C++
             functions from the VCollode API.

\************************************************************************/


#ifndef VCOL_h
#define VCOL_h

/*error codes returned by VCollide API.*/
#define VC_ERR_INVALID_ID            -4 /*invalid id was passed to the
                                         *routine.*/
#define VC_ERR_EMPTY_OBJECT          -3 /*EndObject called without adding
                                         *adding any triangles.*/
#define VC_ERR_CALL_OUT_OF_SEQUENCE  -2 /*calls out of sequence.*/
#define VC_ERR                       -1 /*some other error.*/
#define VC_OK                         1 /*No error.*/



/************************************************************************
Struct: VCReportType
--
Description: Used for reporting collisions.

\************************************************************************/

/*multiply defined in files VInternal.C and VCollide.H*/
#ifndef _VCREPORTTYPE
#define _VCREPORTTYPE
typedef struct VCReportType
{
  int id1, id2;
} VCReportType;
#endif

/*Plain C wrappers for the c++ routines*/
#ifdef __cplusplus
extern "C" 
{
#endif
  
  void *vcOpen(void);
  void  vcClose(void *vc_handle);
  int   vcNewObject(void *vc_handle, int *id);
  int   vcAddTri(void *vc_handle, double v1[], double v2[], double v3[]);
  int   vcEndObject(void *vc_handle);
  int   vcUpdateTrans(void *vc_handle, int id, double t[][4]);
  int   vcActivateObject(void *vc_handle, int id);
  int   vcDeactivateObject(void *vc_handle, int id);
  int   vcActivatePair(void *vc_handle, int id1, int id2);
  int   vcDeactivatePair(void *vc_handle, int id1, int id2);
  int   vcDeleteObject(void *vc_handle, int id);
  int   vcCollide(void *vc_handle);
  int   vcReport(void *vc_handle, int size, VCReportType *vcrep);
  
  
#ifdef __cplusplus
};
#endif

#endif
