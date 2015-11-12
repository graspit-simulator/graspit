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



#ifndef IVCollide_h
#define IVCollide_h

#include "VCollide.H"
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/actions/SoCallbackAction.h>

#ifndef TRICBINFO
#define TRICBINFO
// Used in creating VCObject models from Inventor models
class triCBInfo {

public:
  triCBInfo() {vc=NULL;};
  ~triCBInfo() {};

  VCollide* vc;
};

/*triCBInfo::triCBInfo()
{
  vc = NULL;
}

triCBInfo::~triCBInfo() {}*/
#endif

// The Inventor collision detection class
class ivcollide {

public:
  ivcollide();
  ~ivcollide();

  // Adds an Inventor scene graph (assumed to be 1 object) to the VCollide
  // database and returns the object id assigned to it.  Particularly useful
  // if the verticies of the component triangles are not readily available.
  // If an errors occurs in processing, returns -1.
  int addInventorObj(SoNode* root, int old_id=-1);
  
  // Updates VCObject <id>'s transform from Inventor SoTransform node
  int updateTransform(SoTransform* trans, int id);
  
  VCollide* vc;

private:
  static void ivcollideAddTriangleCB(void* whatever, SoCallbackAction* cba,
				     const SoPrimitiveVertex* v1,
				     const SoPrimitiveVertex* v2,
				     const SoPrimitiveVertex* v3);
  
  // Convenience functions for multiplying matricies and vectors
  static const SbVec3f& transformPoint(const SbMatrix &m, const SbVec3f &p);
  static const SbVec3f& transformNormal(const SbMatrix &m, const SbVec3f &n);
};

#endif


