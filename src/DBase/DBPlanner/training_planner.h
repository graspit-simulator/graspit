//######################################################################
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
// $Id: training_planner.h,v 1.2 2009/06/17 18:57:15 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %TrainingPlanner class
 */

#ifndef DB_PLANNER_TRAINING_PLANNER_H
#define DB_PLANNER_TRAINING_PLANNER_H

#include <string>
#include <vector>
using std::vector;
using std::string;

namespace db_planner {

//! A TrainingPlanner is used to generate grasps that can be added to the database.
class TrainingPlanner {
 public:
  //! Returns a vector of Grasp objects for a Model
  /*! Given a Model, this returns true on success and populates the grasps vector
      with Grasp objects that can be added to the database. On failure returns
      false and the vector's contents are undefined. */
  virtual bool TrainedGrasps(const string& model_name, vector<Grasp>* grasps) const = 0;
  virtual ~TrainingPlanner() {}
};


}  // end namespace db_planner

#endif  // DB_PLANNER_TRAINING_PLANNER_H