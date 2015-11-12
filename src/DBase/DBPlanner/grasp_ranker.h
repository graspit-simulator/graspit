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