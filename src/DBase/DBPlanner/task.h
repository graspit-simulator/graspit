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
};

}

#endif
