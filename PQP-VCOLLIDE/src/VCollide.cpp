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
Filename: VCollide.C
--
Description: This file implements the member functions of the class VCollide.
             The class VCollide is simply a user interface and and is
             designed for data hiding. Hence, these member functions
             simply return make calls to the corresponding member
             functions of the class VCInternal.
\************************************************************************/


#include "VCollide.H"
#include "VCol.h"
#include "VInternal.H"

VCollide::VCollide()
{
  vcint = new VCInternal;
}

VCollide::~VCollide()
{
  delete vcint;
}

int VCollide::ResetObject(int id)
{
  return vcint->ResetObject(id);
}

int VCollide::NewObject(int *id, int threadId)    //create a new object in the database.
{
  return vcint->NewObject(id, threadId);
}

int VCollide::NewClone(int *id, int originalId, int threadId)
{
	return vcint->NewClone(id,originalId, threadId);
}

int VCollide::AddTri(double v1[], double v2[], double v3[],int triInd)  //insert
                                                             //the geometry
{
  return vcint->AddTri(v1, v2, v3,triInd);
}


int VCollide::EndObject(bool ExpectEmpty) //tell VCollide that inserting the 
                              //geometry is complete.
{
  return vcint->EndObject(ExpectEmpty);
}

int VCollide::EditObject(int id) //tell VCollide to allow EditTri commands 
{
  return vcint->EditObject(id);
}

int VCollide::EditTri(double v1[], double v2[], double v3[],int triInd)
  // change the geometry of a triangle
{      
  return vcint->EditTri(v1, v2, v3,triInd);
}

int VCollide::EditFinished(void)//tell VCollide that the changes are complete 
{
  return vcint->EditFinished();
}

int VCollide::GetObject(int id, double **& vList)
{
  return vcint->GetObject(id,vList);
}

PQP_Model *VCollide::GetModel(int id)
{
  return vcint->GetModel(id);
}

int VCollide::UpdateTrans(int id, double t[][4])
  //update the transformation matrix of the object.
{
  return vcint->UpdateTrans(id, t);
}


int VCollide::ActivateObject(int id)          
{
  return vcint->ActivateObject(id);
}

int VCollide::DeactivateObject(int id)        
{
  return vcint->DeactivateObject(id);
}

int VCollide::ActivatePair(int id1, int id2)  //activate the pair.
{
  return vcint->ActivatePair(id1, id2);
}

int VCollide::DeactivatePair(int id1, int id2)//deactivate the pair.
{
  return vcint->DeactivatePair(id1, id2);
}

bool VCollide::isObjectActivated(int id)
{
  return vcint->isObjectActivated(id);
}

bool VCollide::isPairActivated(int id1, int id2)
{
  return vcint->isPairActivated(id1,id2);
}

int VCollide::DeleteObject(int id)  //delete the object from the database.
{
  return vcint->DeleteObject(id);
}

int VCollide::AllCollisions(VCInternal::Query q, VCReportType *vc_report, 
							int size, int *interestList, int interestSize, int thisThreadId)
{
	return vcint->AllCollisions(q, vc_report, size, interestList, interestSize, thisThreadId);
}

int VCollide::AllContacts(PQP_ContactResult *cresArr, double dist_thresh, 
						  int *interestList, int interestSize, int thisThreadId)
{
	return vcint->AllContacts(cresArr, dist_thresh, interestList, interestSize, thisThreadId);
}

double VCollide::Dist(int id1, int id2)
{
  return vcint->Dist(id1,id2);
}

int VCollide::Dist(int id1, int id2,PQP_DistanceResult &dres,
		   double dist_thresh)
{
  return vcint->Dist(id1,id2,dres,dist_thresh);
}

int VCollide::Contact(int id1, int id2,double thresh,PQP_ContactResult &cres )
{
  return vcint->Contact(id1,id2,thresh,cres );
}

void VCollide::FindRegion( int id1, PQP_REAL pt[3], PQP_REAL normal[3], PQP_REAL rad, 
						   std::vector<PQP_Vec> *ptList)
{
	vcint->FindRegion( id1, pt, normal, rad, ptList);
}

void VCollide::getBvs(int id, int desiredDepth, std::vector<BV*> *bvs)
{
	vcint->getBvs(id, desiredDepth, bvs);
}

//returns the distances to all the triangles within a certain threshold distance
double VCollide::FindShortDist( PQP_REAL pt[3], int id1, 
							    PQP_REAL closest_pt[3], PQP_REAL closest_normal[3], PQP_REAL thresh)
{
	return vcint->FindShortDist(pt, id1, closest_pt, closest_normal, thresh);
}
