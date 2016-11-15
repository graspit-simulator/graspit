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
// $Id: caching_neighbor_finder.h,v 1.4 2009/06/17 18:57:14 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %CachingNeighborFinder class
 */

#ifndef DB_PLANNER_CACHING_NEIGHBOR_FINDER_H
#define DB_PLANNER_CACHING_NEIGHBOR_FINDER_H

#include <string>
#include <vector>
#include "db_manager.h"
#include "model.h"
#include "neighbor_finder.h"
using std::string;
using std::vector;

namespace db_planner {

//! A CachingNeighborFinder wraps another NeighborFinder and checks for precomputed neighbors.
/*! A CachingNeighborFinder wraps another NeighborFinder and checks
    into a database for precomputed neighbors before finding new ones. If
    "cache_results_" is true, then new neighbors are written back to the
    database. The wrapped NeighborFinder defaults to the base class,   
    which always fails. */
class CachingNeighborFinder : public NeighborFinder<Model>{
 private:
  //! A DatabaseManager representing the database to query for alignments.
  const DatabaseManager& db_manager_;
  //! Flag determines if newly found neighbors are written back to the database.
  const bool cache_results_;
  //! A string ID of the distance function in the database to use for finding
  //! neighbors (and for saving back results).
  const string distance_function_name_;
  //! A wrapped NeighborFinder to use when the database lookup fails.
  const NeighborFinder<Model>& neighbor_finder_;
 public:
  CachingNeighborFinder(
      const DatabaseManager& db_manager,
      const bool cache_results, 
      const string& distance_function_name,
      const NeighborFinder<Model>& neighbor_finder = NeighborFinder<Model>())
        : db_manager_(db_manager),
          cache_results_(cache_results), 
          distance_function_name_(distance_function_name),
          neighbor_finder_(neighbor_finder) { }
  //! Implementation of (<NeighborFinder>"::")<Find>
  /*! Given a Model and a number of neighbors to return, it finds neighbors if possible
      from the database using the given distance function, or else it uses the wrapped
      NeighborFinder. The neighbors vector is filled on successful output with
      pairs representing the neighbor Models and the distance to each neighbor. 
      On failure the function returns false and the vector contents are undefined.
      WARNING: "num_neighbors" is decieving - the function returns from 
      num_neighbors to 2 x num_neighbors results, because whenever possible
      neighbors are duplicated at the scale above and below the given Model. */
  virtual bool Find(const Model& query, 
                    const int num_neighbors,
                    vector<pair<Model*, double> >* neighbors) const {
    // Try to retrieve saved features first.
    if (db_manager_.GetNeighbors(query, 
                                 distance_function_name_, 
                                 num_neighbors,
                                 neighbors)) {
      return true;
    }
    // That failed, so try to get neighbors the supplied NeighborsGenerator.
    if (neighbor_finder_.Find(query, num_neighbors, neighbors)) {
      // If desired, save the results in the database.
      if (cache_results_) {
        db_manager_.SaveNeighbors(query,
                                  distance_function_name_,
                                  *neighbors);
      }
      return true;
    } 
    return false; // Failure; we can't find neighbors for this query.
  }
};



}  // end namespace db_planner

#endif  // DB_PLANNER_CACHING_NEIGHBOR_FINDER_H_