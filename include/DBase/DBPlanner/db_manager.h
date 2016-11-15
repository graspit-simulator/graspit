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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: db_manager.h,v 1.20 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the %DatabaseManager and %FilterType classes.
*/

#ifndef DB_PLANNER_DB_MANAGER_H
#define DB_PLANNER_DB_MANAGER_H

#include <string>
#include <vector>
#include <utility>
#include <QString>
#include "model.h"
#include "task.h"
#include "grasp.h"
using std::pair;
using std::string;
using std::vector;

class GraspAllocator;
class ModelAllocator;

namespace db_planner {

//! FilterList maps an enum of named filters for filtering a list of Models
//! to a SQL "WHERE" clause that can be tacked onto a query.
class FilterList {
  public:
    //! The list of available filters.
    enum FilterType {NONE, HAS_HUMAN_GRASPS, HAS_HUMAN_REFINED_GRASPS, OLD_STYLE_IV};
    //! Maps a FilterType enum to a SQL "WHERE" clause.
    static string GetWhereClause(FilterType filter) {
      if (filter == HAS_HUMAN_GRASPS) {
        return "WHERE model_name IN (SELECT DISTINCT scaled_model_name "
               "FROM grasp JOIN scaled_model USING(scaled_model_id) "
               "WHERE grasp_source_id = 2)";
      } else if (filter == HAS_HUMAN_REFINED_GRASPS) {
        return "WHERE model_name IN (SELECT DISTINCT scaled_model_name "
               "FROM grasp JOIN scaled_model USING(scaled_model_id) "
               "WHERE grasp_source_id = 3)";
      } else if (filter == OLD_STYLE_IV) {
        return "WHERE model_name LIKE 'matei%'";
      } else {
        return "";
      }
    }
};


//! Pure virtual base class for interfacing GraspIt with an unspecified Grasp Database.
class DatabaseManager {
  protected:

    //! An object to allocate new Model's/
    /*! These will be derived Model types, but we will only know them as Model*'s.*/
    ModelAllocator *model_allocator_;
    //! An object to allocate new Grasp's/
    /*! These will be derived Grasp types, but we will only know them as Grasp*'s.*/
    GraspAllocator *grasp_allocator_;

  public:

    //! Sets a new grasp allocator
    virtual void SetGraspAllocator(GraspAllocator *allocator) = 0;

    //! Sets a new model allocator
    virtual void SetModelAllocator(ModelAllocator *allocator) = 0;

    //! Returns true if the manager has successfully connected to the database
    virtual bool isConnected() const = 0;
    //! Get the 4x4 alignment for 2 Models that are both in the database.
    /*! Returns false if either Model isn't in the database or if no alignment is found.
        Transforms are left-multiply and column major. */
    virtual bool GetAlignment(const Model &source,
                              const Model &dest,
                              const string &alignment_method_name,
                              float alignment[16]) const = 0;
    //! Save a 4x4 alignment into the database.
    /*! Transforms are left-multiply and column major. */
    virtual bool SaveAlignment(const Model &source,
                               const Model &dest,
                               const string &alignment_method_name,
                               const float alignment[16]) const = 0;
    //! Get the neighbors for a model from the database. Returns false if no neighbors are found.
    /*! Note that num_neighbors is misleading; this will return up to 2 x num_neighbors models,
        since whenever possible it returns a model at the scale above and below the query model. */
    virtual bool GetNeighbors(const Model &model,
                              const string &distance_function_name,
                              const int num_neighbors,
                              vector<pair<Model *, double> > *neighbors)
    const = 0;
    //! Save the neighbors for a model into the database.
    virtual bool SaveNeighbors(const Model &model,
                               const string &distance_function_name,
                               const vector<pair<Model *, double> > &neighbors)
    const = 0;
    //! Get a list of the Grasps for a Model.
    virtual bool GetGrasps(const Model &model,
                           const string &hand_name,
                           vector<Grasp *> *grasp_list) const = 0;
    //! Acquires the next experiment to be executed from the list of tasks in the database
    /*! If accepted_types is not empty, it will only get tasks of one of that types. If it's empty,
      it will get any task. Also marks any task it gets as RUNNING in an atomic fashion, so that it
      is not acquired by another process.*/
    virtual bool AcquireNextTask(TaskRecord *rec, std::vector<std::string> accepted_types) = 0;
    //! Change the status of a task in the database (e.g. mark it as COMPLETED)
    virtual bool SetTaskStatus(int task_id, const string &status) = 0;
    //! Fills in the details for a planning task based on the task id
    virtual bool GetPlanningTaskRecord(int task_id, PlanningTaskRecord *rec) = 0;
    //! Fills in the details for an optimzation task based on the task id
    virtual bool GetOptimizationTaskRecord(int /*task_id*/, OptimizationTaskRecord * /*rec*/) {return false;}
    //! Saves the results of an optimization in the database
    virtual bool SaveOptimizationResults(const OptimizationTaskRecord & /*rec*/,
                                         const std::vector<double> & /*parameters*/,
                                         const std::vector<double> & /*results*/,
                                         const std::vector<double> & /*seed*/) {return false;}

    //! Save a grasp into the database
    virtual bool SaveGrasp(const Grasp *) const = 0;
    //! Save a list of Grasps into the database. Convenience function, just calls above version
    virtual bool SaveGrasps(const vector<Grasp *> graspList) const {
      for (size_t i = 0; i < graspList.size(); ++i) {
        if (!SaveGrasp(graspList[i])) {
          return false;
        }
      }
      return true;
    }
    //! Delete a grasp from the database
    virtual bool DeleteGrasp(Grasp *grasp) const = 0;
    //! Sets the cluster_rep field of the grasp in the database
    /*! Note: this is an ugly way of updating the database. There should be a single function
      where a grasp is updated in the database. You check out a grasp from the dbase, do
      whatever you want with it and then call updateToDb or something.
    */
    virtual bool SetGraspClusterRep(Grasp *grasp, bool rep) const = 0;

    //! Inserts into the database info that a pair of existing db grasps can be executed simultaneously
    virtual bool InsertGraspPair(const Grasp *grasp1, const Grasp *grasp2) const = 0;
    //! Sets the table_clearance field of the grasp in the database
    virtual bool SetGraspTableClearance(Grasp *grasp, double clearance) const = 0;
    //! Returns a vector with pointers to every Model in the database, possibly filtered.
    virtual bool ModelList(vector<Model *> *model_list,
                           FilterList::FilterType filter = FilterList::NONE) const = 0;
    //! Gets one individual model from the database based on scaled model id
    virtual bool ScaledModel(Model *&model, int scaled_model_id) const = 0;
    //! Returns a vector of strings representing the available grasp sources.
    virtual bool GraspTypeList(vector<string> *type_list) const = 0;
    //! Returns a vector of strings representing the available neighbor distance functions.
    virtual bool DistanceFunctionList(
      vector<string> *distance_function_list) const = 0;
    //! Returns a vector of strings representing the available precomputed alignment methods.
    virtual bool AlignmentMethodList(
      vector<string> *alignment_method_list) const = 0;
    //! Loads a model's geometry directly from the database
    virtual bool LoadModelGeometry(Model *) const = 0;
    //! Loads the graspit relative path for a hand
    virtual QString getHandGraspitPath(QString handDBName) const = 0;

    virtual ~DatabaseManager() {}

};

}
#endif  // DB_PLANNER_DB_MANAGER
