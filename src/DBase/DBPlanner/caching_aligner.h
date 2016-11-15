//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Corey Goldfeder
//
// $Id: caching_aligner.h,v 1.12 2009/06/17 18:57:14 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %CachingAligner class
 */

#ifndef DB_PLANNER_CACHING_ALIGNER_H
#define DB_PLANNER_CACHING_ALIGNER_H

#include <string>
#include "aligner.h"
#include "db_manager.h"
#include "model.h"
using std::string;

namespace db_planner {
 
//! A CachingAligner wraps another Aligner and checks for precomputed alignments.
/*! Precomputed alignments come from a database.
    If "cache_results_" is true, then new alignments are written back to 
    the database, and if "cache_inverse_results" is true the inverse is
    cached as well. By default the wrapped Aligner is the base class,
    which always fails.
*/
class CachingAligner : public Aligner<Model>{
 private:
  //! A DatabaseManager representing the database to query for alignments.
  const DatabaseManager& db_manager_;
  //! Flags for whether computed alignments are written back into the database.
  const bool cache_results_;
  //! Flags for whether inverses are written back into the database.
  const bool cache_inverse_results_;
  //! The ID of the alignment method to use when querying the database.
  const string alignment_method_name_;
  //! The Aligner that is used if the database lookup fails.
  const Aligner<Model> aligner_;
  //! A helper function to invert a 4x4 transform.
  bool InvertTransform(float* /*transform*/, 
                       float* /*inverse*/) const { return false; }
 public:
  CachingAligner(const DatabaseManager& db_manager,
                 const bool cache_results, 
                 const bool cache_inverse_results, 
                 const string alignment_method_name,
                 const Aligner<Model>& aligner = Aligner<Model>())
    : db_manager_(db_manager),
      cache_results_(cache_results), 
      cache_inverse_results_(cache_inverse_results), 
      alignment_method_name_(alignment_method_name),
      aligner_(aligner) { }

  //! Implementation of (<Aligner>"::")<Align>
  /*! Transforms are left-multiply and column major. */
  virtual bool Align(const Model& source, 
                     const Model& dest, 
                     float transform[16]) const {
    // Try to retrieve a saved alignment first.
    if (db_manager_.GetAlignment(source, 
                                 dest, 
                                 alignment_method_name_, 
                                 transform)) {
      return true;
    }
    // That failed, so try to do the alignment with the supplied Aligner.
    if (aligner_.Align(source, dest, transform)) {
      if (cache_results_) // If desired, save the results in the database.
        db_manager_.SaveAlignment(source, 
                                  dest, 
                                  alignment_method_name_, 
                                  transform);
      float inverse[16];  // The same for the inverse, if possible.
	    //todo: implementation of InvertTransform does not exist
      if (cache_inverse_results_ && InvertTransform(transform, inverse)) 
        db_manager_.SaveAlignment(dest,
                                  source, 
                                  alignment_method_name_, 
                                  inverse);
		  return true;
    }
    return false;  // Failure; we can't do this alignment.
  }
};


}  // end namespace db_planner

#endif  // DB_PLANNER_CACHING_ALIGNER_H