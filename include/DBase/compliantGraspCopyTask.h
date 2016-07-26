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
// Author(s):  Matei T. Ciocarlie
//
// $Id: compliantGraspCopyTask.h,v 1.1 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#ifndef _COMPLIANTGRASPCOPYTASK_H_
#define _COMPLIANTGRASPCOPYTASK_H_

#include "preGraspCheckTask.h"

#include "pr2Gripper.h"

class transf;

//! For all the grasps of the given object, computes and stores their compliant equivalents
/*! A compliant equivalent grasp of a given grasp is obtained by doing a "compliant open" 
  (meaning opening around one of the fingers).

  We are inheriting from the PreGraspCheckTask as this shares some of the functionality,
  
 */
class CompliantGraspCopyTask : public PreGraspCheckTask {
 private:
  //! Does all the compliant copy work for one particular grasp
  bool compliantCopy(const db_planner::Grasp *grasp, Pr2Gripper2010::ComplianceType compliance);

  //! Stores a new grasp in the database as a compliant copy of the original
  bool checkStoreGrasp(const db_planner::Grasp *original);

  //! Returns true if two transforms are "close" to each other
  bool similarity(const transf &t1, const transf &t2);

  //! Same thing, but using the distance measure also used in grasp planning
  bool searchSimilarity(const transf &t1, const transf &t2);

 public:
  //! Just a stub for now
  CompliantGraspCopyTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
                         db_planner::TaskRecord rec);
  //! Nothing to do here
  ~CompliantGraspCopyTask(){}

  //! Actually does all the work
  virtual void start();
};


#endif
