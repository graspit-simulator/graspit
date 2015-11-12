/****************************************************************************
Legal stuff first:
Copyright 1998, Massachusetts Institute of Technology.  All Rights Reserved.

The software may be used for research and educational purposes only and
LICENSEE agrees that the Program shall not be used as the basis of a
commercial software or hardware product.

LICENSEE acknowledges that title to the COPYRIGHTS shall remain with
M.I.T. and that any copies of the software and related documentation, or
portions thereof, made by LICENSEE  hereunder, shall include an M.I.T.
copyright notice thereon in either of the following forms: "Copyright
199_, Massachusetts Institute of Technology.  All Rights Reserved." or "©
199_, M.I.T.  All Rights Reserved."  The notice shall be affixed to all
copies or portions thereof in such manner and location as to give
reasonable notice of M.I.T.'s claim of copyright.  LICENSEE shall at all
times hereafter protect the COPYRIGHTS

LICENSEE accepts the program on an "AS IS' basis.
M.I.T., its trustees, directors, officers, employees and affiliates MAKE
NO REPRESENTATIONS AND EXTENDS NO WARRANTIES OF ANY KIND, EITHER EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, AND VALIDITY OF THE COPYRIGHTS.  NOTHING
IN THIS AGREEMENT SHALL BE CONSTRUED AS A REPRESENTATION MADE OR WARRANTY
GIVEN BY M.I.T. THAT THE PRACTICE BY LICENSEE OF THE LICENSE GRANTED
HEREUNDER SHALL NOT INFRINGE THE INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD
PARTY.  FURTHERMORE, M.I.T. EXTENDS NO WARRANTIES OF ANY KIND AS TO
PROGRAM CONFORMITY WITH WHATEVER USER MANUALS OR OTHER LITERATURE MAY BE
ISSUED FROM TIME TO TIME. IN NO EVENT SHALL M.I.T., ITS TRUSTEES,
DIRECTORS, OFFICERS, EMPLOYEES AND AFFILIATES BE LIABLE FOR INCIDENTAL OR
CONSEQUENTIAL DAMAGES OF ANY KIND, INCLUDING ECONOMIC DAMAGE OR INJURY TO
PROPERTY AND LOST PROFITS, REGARDLESS OF WHETHER M.I.T. SHALL BE ADVISED,
SHALL HAVE OTHER REASON TO KNOW, OR IN FACT SHALL KNOW OF THE POSSIBILITY
OF THE FOREGOING.

*******************************************************************************

Questions? Comments? Bugs?
Send email to David Manowitz
manowitz@mit.edu, manowitz@alum.mit.edu after 6/1999

******************************************************************************/




#include "math.h"
#include "ivcollide.h"
#include <iostream>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoNormalBinding.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

int triId;

ivcollide::ivcollide()
{
  vc = new VCollide;
}

ivcollide::~ivcollide()
{
  delete vc;
}

// Uncomment the first declaration and the commented lines below if you've
// modified VCOLLIDE to have a ResetObject() method that takes the id of an
// existing object and resets the RAPID model and want to use it with
// ivcollide.

int
ivcollide::addInventorObj(SoNode *root, int old_id)
{

  static triCBInfo info;

  //  If old_id is not -1, then reuse existing object, otherwise
  //  start by making a new VCObject in the VCollide database

  int id, rc;

  if (old_id != -1) {
    id = old_id;
    if ((rc = vc->ResetObject(id)) != VC_OK) {
      std::cerr << "ivcollide--error in VCollide.ResetObject(" << old_id;
      std::cerr << "): " << rc << std::endl;
      return -1;
    }
  }
  else if ((rc = vc->NewObject(&id)) != VC_OK) {
    std::cerr << "ivcollide--error in VCollide.NewObject(): " <<rc<<std::endl;
    return -1;
  }

  triId = 0;

  // Next, set up the callback and traverse the scene graph,
  // adding triangles as they are encountered
  info.vc = vc;
  
  SoType shapeType = SoShape::getClassTypeId();
  SoCallbackAction ca;
  ca.addTriangleCallback(shapeType, ivcollideAddTriangleCB, &info);
  ca.apply(root);

  // Finally, end the creation process
  if ((rc = vc->EndObject()) != VC_OK) {
    std::cerr << "ivcollide--error in VCollide.EndObject(): " << rc<<std::endl;
    return -1;
  }

  return id;
}

int
ivcollide::cloneInventorObj(int originalId)
{
	int id,rc;

	//create the cloned vcollide object
	if ((rc = vc->NewClone(&id, originalId)) != VC_OK) {
		std::cerr << "ivcollide--error in VCollide.NewObject(): " <<rc<<std::endl;
		return -1;
	}

	// Finally, end the creation process
	if ((rc = vc->EndObject()) != VC_OK) {
		std::cerr << "ivcollide--error in VCollide.EndObject(): " << rc<<std::endl;
		return -1;
	}

	 return id;
}

int
ivcollide::updateTransform (SoTransform* trans, int id)
{

  // don't care about viewport here
  static SoGetMatrixAction ma(*(new SbViewportRegion));
  static double newTrans[4][4];

  ma.apply(trans);

  // Remember: Inventor uses matrices of floats, not doubles!
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // Must convert each element from float->double (not just pointer)!
      // Also: VCOLLIDE represents points as column vectors, not rows,
      //       so must transpose matrix!!!
      newTrans[i][j] = (ma.getMatrix().getValue())[j][i];
    }
  }

  return (vc->UpdateTrans(id, newTrans));
}


void
ivcollide::ivcollideAddTriangleCB(void* whatever, SoCallbackAction *cba,
				  const SoPrimitiveVertex* v1,
				  const SoPrimitiveVertex* v2,
				  const SoPrimitiveVertex* v3)
{

  static double dp1[3], dp2[3], dp3[3];
  
  triCBInfo* info = (triCBInfo*) whatever;
  SbVec3f p1, p2, p3;
  SbVec3f zero = SbVec3f(0.0, 0.0, 0.0);
  SbMatrix mm = cba->getModelMatrix();

  // Transform verticies (remember verticies are in the object space
  // coordinates for each triangle)
  p1 = transformPoint(mm, v1->getPoint());
  p2 = transformPoint(mm, v2->getPoint());
  p3 = transformPoint(mm, v3->getPoint());

  // Don't add degenerate triangles!
  if ((p1 == p2) || (p2 == p3) || (p1 == p3)) {    
    return;
  }

  // Need to convert each coordinate of each vertex to double
  dp1[0] = p1[0];
  dp1[1] = p1[1];
  dp1[2] = p1[2];
  dp2[0] = p2[0];
  dp2[1] = p2[1];
  dp2[2] = p2[2];
  dp3[0] = p3[0];
  dp3[1] = p3[1];
  dp3[2] = p3[2];

  // Add transformed triangle to model
  info->vc->AddTri(dp1, dp2, dp3,triId++);
}

const SbVec3f&
ivcollide::transformPoint(const SbMatrix &m, const SbVec3f &p)
{
  static SbVec3f tp;
  
  m.multVecMatrix(p, tp);
  return tp;
}

const SbVec3f&
ivcollide::transformNormal(const SbMatrix &m, const SbVec3f &n)
{
  static SbVec3f tn;

  m.multDirMatrix(n, tn);
  tn.normalize();
  return tn;
}
  
