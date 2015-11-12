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
// Author(s):  Matei Ciocarlie
//
// $Id: task.h,v 1.1 2009/11/16 18:32:39 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the cpp record of an entry in the task_list table in the dbase
 */

#ifndef DB_PLANNER_TASK_H
#define DB_PLANNER_TASK_H

#include <string>
#include <set>
using std::string;
using std::set;

namespace db_planner {

class Model;

//! For now, a task does something of a type with a hand to a model
struct TaskRecord {
  Model *model;
  string handName;
  int taskType;
  int taskId;
  int taskTime;
  string misc;
  string argument;
};

}

#endif
