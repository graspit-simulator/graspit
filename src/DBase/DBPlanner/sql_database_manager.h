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
// $Id: sql_database_manager.h,v 1.19 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file 
    \brief Defines %SqlDatabaseManager class.
 */

#ifndef DB_PLANNER_SQL_DATABASE_MANAGER_H
#define DB_PLANNER_SQL_DATABASE_MANAGER_H

#include <cstdlib>
#include <string>
#include <vector>
#include "database.h"
#include "db_manager.h"
using std::string;
using std::vector;

namespace db_planner {

//! SqlDatabaseManager implements DatabaseManager for using the Postgresql version of the CGDB.
class SqlDatabaseManager : public DatabaseManager {
 protected:
  //! The connection to the SQL database
  mutable DatabaseConnection database_;

  //! Absolute path to the root directory where geometry and thumbnails are stored.
  string model_root_;
 public:
  SqlDatabaseManager(const string& host_name,                     
                     const int port,
                     const string& user_name,
                     const string& password,
                     const string& database_name, 
                     ModelAllocator *model_allocator,
                     GraspAllocator *grasp_allocator,
                     const string& connection_type = "QPSQL") 
                        : database_(host_name, 
                                    port, 
                                    user_name, 
                                    password, 
                                    database_name, 
                                    connection_type),
                                   model_root_(getenv("CGDB_MODEL_ROOT")) 
			    {
			      model_allocator_ = model_allocator;
			      grasp_allocator_ = grasp_allocator;
			    }
  ~SqlDatabaseManager(){delete model_allocator_; delete grasp_allocator_;}

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

  //! Returns true if the manager has successfully connected to the database
  virtual bool isConnected() const {return database_.isConnected();}
  //! Get the 4x4 alignment for 2 Models that are both in the database.
  /*! Returns false if either Model isn't in the database or if no alignment is found.
      Transforms are left-multiply and column major. */
  virtual bool GetAlignment(const Model& source,
                            const Model& dest, 
                            const string& alignment_method_name, 
                            float alignment[16]) const;
  //! Save a 4x4 alignment into the database.
  /*! Transforms are left-multiply and column major. */
  virtual bool SaveAlignment(const Model& source,
                             const Model& dest, 
                             const string& alignment_method_name, 
                             const float alignment[16]) const;
  //! Get the neighbors for a model from the database. Returns false if no neighbors are found.
  /*! Note that num_neighbors is misleading; this will return up to 2 x num_neighbors models,
      since whenever possible it returns a model at the scale above and below the query model. */
  virtual bool GetNeighbors(const Model& model, 
                            const string& distance_function_name, 
                            const int num_neighbors,
                            vector<pair<Model*, double> >* neighbors) const;
  //! Save the neighbors for a model into the database.
  virtual bool SaveNeighbors(const Model& model, 
                             const string& distance_function_name, 
                             const vector<pair<Model*, double> >& neighbors) const;
  //! Get a list of the Grasps for a Model.
  virtual bool GetGrasps(const Model& model, 
                         const string& hand_name, 
                         vector<Grasp*>* grasp_list) const;
  //! Save a grasp into the database.
  virtual bool SaveGrasp(const Grasp* grasp) const;
  //! Returns a vector with pointers to every Model in the database, possibly filtered.
  virtual bool ModelList(vector<Model*>* model_list, 
                         FilterList::FilterType filter = FilterList::NONE) const;
  //! Returns a vector of strings representing the available grasp sources.
  virtual bool GraspTypeList(vector<string>* type_list) const;
  //! Returns a vector of strings representing the available neighbor distance functions.
  virtual bool DistanceFunctionList(
      vector<string>* distance_function_list) const;
  //! Returns a vector of strings representing the available precomputed alignment methods.
  virtual bool AlignmentMethodList(
      vector<string>* alignment_method_list) const;

  // Functionality not implemented in this manager:
  virtual bool DeleteGrasp(Grasp* /*grasp*/) const  {return false;}
  virtual bool AcquireNextTask(TaskRecord* /*rec*/, std::vector<std::string> /*accepted_types*/){return false;}
  virtual bool SetTaskStatus(int /*task_id*/, const string& /*status*/){return false;}
  virtual bool GetPlanningTaskRecord(int task_id, PlanningTaskRecord* /*rec*/){return false;}
  virtual bool SetGraspClusterRep(Grasp* /*grasp*/, bool /*rep*/) const {return false;}
  virtual bool InsertGraspPair(const Grasp* /*grasp1*/, const Grasp* /*grasp2*/) const {return false;}
  virtual bool SetGraspTableClearance(Grasp* /*grasp*/, double /*clearance*/) const { return false;}
  virtual bool LoadModelGeometry(Model*) const {return false;}
  virtual bool ScaledModel(Model* &model, int scaled_model_id) const {return false;}
};

}  // end namespace db_planne

#endif  // DB_PLANNER_SQL_DATABASE_MANAGER
