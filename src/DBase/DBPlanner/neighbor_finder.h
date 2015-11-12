//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Corey Goldfeder
//
// $Id: neighbor_finder.h,v 1.3 2009/06/17 18:57:15 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %NeighborFinder class
 */

#ifndef DB_PLANNER_NEIGHBOR_FINDER_H
#define DB_PLANNER_NEIGHBOR_FINDER_H

#include <vector>
#include <utility>
#include "model.h"
using std::pair;
using std::vector;

namespace db_planner {

//! A class to find Models that are neighbors of some input.
template <class Query>
class NeighborFinder {
 public:
  //! This virtual function finds neighboring models to a given Model. This
  //! base version always fails regardless of input.
  //! WARNING: "num_neighbors" is decieving - implementations should returns from 
  //! num_neighbors to 2 x num_neighbors results, because whenever possible
  //! neighbors should be duplicated at the scale above and below the given Model.           
  virtual bool Find(const Query& /*query*/, 
                    const int /*num_neighbors*/,
                    vector<pair<Model*, double> >* /*neighbors*/) const {
    return false;
  }; 
  virtual ~NeighborFinder() {}
};


}  // end namespace db_planner


#endif  // DB_PLANNER_NEIGHBOR_FINDER_H