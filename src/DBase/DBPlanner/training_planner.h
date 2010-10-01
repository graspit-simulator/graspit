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