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
// $Id: ros_database_manager.h,v 1.6 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %RosDatabaseManager class.
*/

#ifndef ROS_DATABASE_MANAGER_H
#define ROS_DATABASE_MANAGER_H

#include <string>
#include <vector>
#include <utility>

#include "db_manager.h"
#include "model.h"
#include "task.h"
#include "grasp.h"

namespace household_objects_database {
  class ObjectsDatabase;
  class DatabaseScaledModel;
}

using std::pair;
using std::string;
using std::vector;

namespace db_planner {

//! Pure virtual base class for interfacing GraspIt with an unspecified Grasp Database.
class RosDatabaseManager : public DatabaseManager 
{
 protected:

  //! The actual ROS interface to the database
  household_objects_database::ObjectsDatabase* database_;

  //! Initializes a graspit model from a raw database model. Helper for ModelList
  void modelFromDBModel(Model *model, 
                        const household_objects_database::DatabaseScaledModel &db_model, 
                        std::string model_root) const;
 public:
  RosDatabaseManager(std::string host, std::string port, std::string user,
		     std::string password, std::string dbname, 
		     ModelAllocator *model_allocator, GraspAllocator* grasp_allocator);
  ~RosDatabaseManager();

  //! Sets a new grasp allocator; deletes the old one
  void SetGraspAllocator(GraspAllocator* allocator) {
    delete grasp_allocator_;
    grasp_allocator_ = allocator;
  }
  //! Sets a new model allocator; deletes the old one
  void SetModelAllocator(ModelAllocator* allocator) {
    delete model_allocator_;
    model_allocator_ = allocator;
  }

  //! Not implemented
  virtual bool GetAlignment(const Model&, const Model&, const string&, float*) const {return false;}
  //! Not implemented
  virtual bool SaveAlignment(const Model&, const Model&, const string&, const float*) const {return false;}
  //! Not implemented
  virtual bool GetNeighbors(const Model&, const string&, const int, 
			    vector<pair<Model*, double> >*) const {return false;}
  //! Not implemented
  virtual bool SaveNeighbors(const Model&, const string&,
			     const vector<pair<Model*, double> >&) const {return false;}
  //! Not implemented
  virtual bool DistanceFunctionList(vector<string>*) const {return false;}
  //! Not implemented
  virtual bool AlignmentMethodList(vector<string>*) const {return false;}

  //! Returns true if the manager has successfully connected to the database
  virtual bool isConnected() const;

  //! Get a list of models in the database
  virtual bool ModelList(vector<Model*>* model_list, 
			 FilterList::FilterType filter = FilterList::NONE) const;
  //! Gets one individual model from the database based on scaled model id
  virtual bool ScaledModel(Model* &model, int scaled_model_id) const;

  //! Acquires the next experiment to be executed from the list of tasks in the database
  /*! If accepted_types is not empty, it will only get tasks of one of that types. If it's empty,
    it will get any task. Also marks any task it gets as RUNNING in an atomic fashion, so that it 
    is not acquired by another process.*/
  virtual bool AcquireNextTask(TaskRecord *rec, std::vector<std::string> accepted_types);
  //! Change the status of a task in the database (e.g. mark it as COMPLETED)
  virtual bool SetTaskStatus(int task_id, const string &status);
  //! Fills in the details for a planning task based on the task id
  virtual bool GetPlanningTaskRecord(int task_id, PlanningTaskRecord *rec);
  //! Fills in the details for an optimzation task based on the task id
  virtual bool GetOptimizationTaskRecord(int task_id, OptimizationTaskRecord *rec);
  //! Saves the results of an optimization in the database
  virtual bool SaveOptimizationResults(const OptimizationTaskRecord &rec,
                                       const std::vector<double>& parameters,
                                       const std::vector<double>& results,
				       const std::vector<double>& seed );
  //! Get a list of grasp types available in the database
  virtual bool GraspTypeList(vector<string>* type_list) const;
  //! Get a list of the Grasps for a Model.
  virtual bool GetGrasps(const Model& model, const string& hand_name, vector<Grasp*>* grasp_list) const;

  //! Save a grasp into the database
  virtual bool SaveGrasp(const Grasp*) const;
  //! Delete a grasp from the database
  virtual bool DeleteGrasp(Grasp* grasp) const;
  //! Sets the cluster_rep field of the grasp in the database  
  virtual bool SetGraspClusterRep(Grasp *grasp, bool rep) const;
  //! Sets the table_clearance field of the grasp in the database
  virtual bool SetGraspTableClearance(Grasp *grasp, double clearance) const;
  //! Inserts into the database info that a pair of existing db grasps can be executed simultaneously
  virtual bool InsertGraspPair(const Grasp *grasp1, const Grasp *grasp2) const;
  //! Loads the model's mesh directly from the database
  virtual bool LoadModelGeometry(Model* model) const;
};

}
#endif  // ROS_DATABASE_MANAGER_H
