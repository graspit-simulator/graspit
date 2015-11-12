//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2010  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Jonathan Weisz Corey Goldfeder
//
// $Id: caching_aligner.h,v 1.12 2009/06/17 18:57:14 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %CachingAligner class
 */

#ifndef DB_PLANNER_OFF_WRAPPING_ALIGNER_H
#define DB_PLANNER_OFF_WRAPPING_ALIGNER_H

#include <string>
#include "aligner.h"
#include "db_manager.h"
#include "model.h"
#include "caching_aligner.h"

using std::string;

namespace db_planner {
 
//! An OffWrappingAligner wraps an aligner where alignments are specified for iv files.
/*! IV files models were designed to have colocated centers of mass.  For the off files, 
	this is not the case.
*/

transf tranArrayToTransf(const float array[16]){
		transf transform;
		// synthesize the transformation matrix
		mat3 m;
		m[0] = array[0];
		m[1] = array[1];
		m[2] = array[2];
		m[3] = array[4];
		m[4] = array[5];
		m[5] = array[6];
		m[6] = array[8];
		m[7] = array[9];
		m[8] = array[10];
		vec3 v;
		v[0] = array[3];
		v[1] = array[7];
		v[2] = array[11];
		transform.set(m,v);
		return transform;
	}
  void transfToTranArray(const transf & transform, float array[16]){
		// synthesize the transformation matrix
		mat3 m = transform.affine();
		vec3 v = transform.translation();
		array[0] = m[0];
		array[1] = m[1];
		array[2] = m[2];
		array[4] = m[3];
		array[5] = m[4];
		array[6] = m[5];
		array[8] = m[6];
		array[9] = m[7];
		array[10] = m[8];
		
		array[3] = v[0];
		array[7] = v[1];
		array[11] = v[2];
	}	
class OffWrappingAligner : public Aligner<Model>{
 private:
  const Aligner<Model> aligner_;
public:
  OffWrappingAligner(const Aligner<Model>& aligner = Aligner<Model>())
    : aligner_(aligner) { }

  //! Implementation of (<Aligner>"::")<Align>
  /*! Transforms are left-multiply and column major. */
  virtual bool Align(const Model& source, 
                     const Model& dest, 
                     float transform[16]) const {
    float internalTransform[16]; // storage for internal, iv based transform
	//Try to get alignment from wrapped aligner
    if (aligner_.Align(source, dest, internalTransform)) {
		 GraspableBody * targetGB = static_cast<GraspitDBModel*>(const_cast<Model *>(&dest))->getGraspableBody();
		 GraspitDBModel* sourceGDBM = static_cast<GraspitDBModel*>(const_cast<Model *>(&source));
		 if (!sourceGDBM->geometryLoaded()){
			sourceGDBM->load(targetGB->getWorld());
		 }
		 GraspableBody * sourceGB = sourceGDBM->getGraspableBody();
		 transf tr = tranArrayToTransf(internalTransform);
		 transf finalTr = transf(mat3::IDENTITY, vec3(-sourceGB->getCoG()[0],-sourceGB->getCoG()[1],-sourceGB->getCoG()[2]))*tr*transf(mat3::IDENTITY, vec3(targetGB->getCoG()[0],targetGB->getCoG()[1],targetGB->getCoG()[2]));
		 transfToTranArray(finalTr, transform);
		 return true;
    }
    return false;  // Failure; we can't do this alignment.
  }
};


}  // end namespace db_planner

#endif  // DB_PLANNER_CACHING_ALIGNER_H