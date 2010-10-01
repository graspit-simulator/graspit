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
// $Id: grasp_ranker.h,v 1.4 2009/06/17 18:57:15 coreyg Exp $
//
//######################################################################

/*! \file 
    \brief Defines the %GraspRanker class.
 */


#ifndef DB_PLANNER_GRASP_RANKER_H
#define DB_PLANNER_GRASP_RANKER_H

#include <string>
#include <vector>
#include "grasp.h"
using std::string;
using std::vector;

namespace db_planner {

//! A GraspRanker takes a list of Grasp objects and sorts them from best to worst.
class GraspRanker {
private:
  //! The name of the hand to rank grasps for.
	string hand_name_;
 public:
  GraspRanker(const string& hand_name) : hand_name_(hand_name) { }
  //! Rank a list of grasps by sorting in place them from best to worst.
  /*! This default implementation doesn't change the order. */
  virtual bool Rank(vector<Grasp>* /*grasps*/) const { return true; }
  ~GraspRanker() {}
};

}  // end namespace db_planner


#endif  // DB_PLANNER_GRASP_RANKER_H