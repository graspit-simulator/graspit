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
// $Id: sql_database_manager.h,v 1.12 2009/07/02 21:07:37 cmatei Exp $
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
 private:
  //! The connection to the SQL database
  mutable DatabaseConnection database_;
  //! An object to allocate new Model's/
  /*! These will be derived Model types, but we will only know them as Model*'s.*/
  ModelAllocator *model_allocator_;
  //! An object to allocate new Grasp's/
  /*! These will be derived Grasp types, but we will only know them as Grasp*'s.*/
  GraspAllocator *grasp_allocator_;
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
                          model_allocator_(model_allocator),
                          grasp_allocator_(grasp_allocator),
                          model_root_(getenv("CGDB_MODEL_ROOT")) {}
  ~SqlDatabaseManager(){delete model_allocator_; delete grasp_allocator_;}
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
  // Get a specific grasp from the database by grasp_id
  virtual Grasp* GetGrasp(const int graspIdNumber);
  //! Save the PCA of a model to the PCA table
  virtual bool SavePCA(
	  const Model & model, const float pca[9]) const;
  //! Get the PCA of a model from the PCA table
  virtual bool GetPCA(
	  const Model & model, float pca[9])const;
  //! Save a Model's Grasps into the database.
  virtual bool SaveGrasps(const vector<Grasp*>) const;
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
  virtual bool TaskTypeList(
	  vector<string> * task_type_list) const;
  virtual bool UpdateGrasp(const std::vector<Grasp*> graspList, FILE * fp) const;
  virtual bool UpdateGraspTactileContacts(int grasp_id, std::vector<double> tactileContacts) const;
  virtual bool UpdateTaskResults(int task_id, std::vector<double> taskResults) const;

  //! Gets an experiment to be executed from the list of tasks in the database
  virtual bool GetNextTask(TaskRecord *rec, const string & requestedStatus = "TO_GO", const string & setStatus="RUNNING");
  //! Change the status of a task in the database (e.g. mark it as COMPLETED)
  virtual bool SetTaskStatus(const TaskRecord &rec, const string &status, const bool requestTransaction = true);
  //! Age all tasks with a certain status if the task has hung for a certain time
  virtual bool AgeTaskStatus(const int &tType, const string &currentStatus, 
		const string & ageToStatus, int expirationTime);
  virtual bool setHand(Hand * h) {if (grasp_allocator_) return grasp_allocator_->setHand(h); return true;}

  //! Get the object pose table
  virtual bool GetObjectPoseList(vector<Pose>* pose_list) const;
};

}  // end namespace db_planne

#endif  // DB_PLANNER_SQL_DATABASE_MANAGER
