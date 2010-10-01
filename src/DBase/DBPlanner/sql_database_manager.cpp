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
// $Id: sql_database_manager.cpp,v 1.21 2010/03/04 23:25:29 cmatei Exp $
//
//######################################################################

/*! \file 
    \brief Defines members of the %SqlDatabaseManager class
 */


#include "sql_database_manager.h"

#include <sstream>
#include <vector>
using std::make_pair;    // Included from <utility> in "db_manager.h"
using std::stringstream;
using std::vector;

#include <iostream>
using namespace std;

//#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(GET_GRASPS_SQL);
PROF_DECLARE(GET_GRASPS_GETCOLUMN);
PROF_DECLARE(GET_GRASPS_GETFIELD);

namespace db_planner {

bool SqlDatabaseManager::GetAlignment(const Model& source,
                                      const Model& dest, 
                                      const string& alignment_method_name, 
                                      float alignment[16]) const {
  Table results;
  stringstream query_text;                  
  query_text << "SELECT get_alignment('" << source.ModelName() << "','" 
             << dest.ModelName() << "','" << alignment_method_name << "')";
  if (!database_.Query(query_text.str(), &results)) return false;
  vector<float> alignment_vector;
  alignment_vector.reserve(16);
  if (results.NumColumns() == 0 || results.NumRows() == 0) return false;
  if (!results.GetField(0, 0, &alignment_vector)) return false;
  for (int i = 0; i < 16; ++i) alignment[i] = alignment_vector[i];
  return true;
}

bool SqlDatabaseManager::SaveAlignment(const Model& source,
                                       const Model& dest, 
                                       const string& alignment_method_name, 
                                       const float alignment[16]) const {
  if (alignment[0] == 0) return false;
  if (alignment_method_name == "") return false;
  if (source.ModelName() != dest.ModelName()) return false;
  return false;
}


//! Load a Model from the database and populate its fields.
class LoadModelFunctor {
 private:
  const Table& table_;
  const string& model_root_;
  int model_name_column_, thumbnail_path_column_, scale_column_, 
      tags_column_, grasping_rescale_column_, geometry_path_column_;
  bool initialized_;
 public:
   LoadModelFunctor(const Table& table, const string& model_root) 
       : table_(table), model_root_(model_root) {
     initialized_ = 
         table_.GetColumnIndex("model_name", &model_name_column_) &&
         table_.GetColumnIndex("thumbnail_path", &thumbnail_path_column_) &&
         table_.GetColumnIndex("geometry_path", &geometry_path_column_) &&
         table_.GetColumnIndex("scale", &scale_column_) &&
         table_.GetColumnIndex("tags", &tags_column_) &&
         table_.GetColumnIndex("grasping_rescale", &grasping_rescale_column_);         
   }
  bool operator()(Model* model, const int row) {
    string model_name, thumbnail_path, geometry_path;
    double scale, grasping_rescale;
    vector<string> tags;
    if (!initialized_ ||
        !table_.GetField(model_name_column_, row, &model_name) || 
        !table_.GetField(thumbnail_path_column_, row, &thumbnail_path) ||
        !table_.GetField(geometry_path_column_, row, &geometry_path) ||
        !table_.GetField(scale_column_, row, &scale) ||
        !table_.GetField(tags_column_, row, &tags) ||
        !table_.GetField(grasping_rescale_column_, row, &grasping_rescale))
      return false;
    model->SetModelName(model_name);
    model->SetThumbnailPath(model_root_ + thumbnail_path);
    model->SetScale(scale);
    model->SetTags(tags.begin(), tags.end());
    model->SetGeometryPath(model_root_ + geometry_path);
    model->SetRescaleFactor(grasping_rescale * scale);
	return true;
  }
};



bool SqlDatabaseManager::GetNeighbors(const Model& model, 
                                      const string& distance_function_name, 
                                      const int num_neighbors,
                                      vector<pair<Model*, double> >* neighbors) const {
  if (neighbors == NULL) return false;
  Table results;
  stringstream query_text;                  
  query_text << "SELECT * FROM get_saved_neighbors('" << model.ModelName()
             << "','" << distance_function_name 
             << "'," << num_neighbors << ");";
  if (!database_.Query(query_text.str(), &results)) return false;
  int distance_column;
  if (!results.GetColumnIndex("distance", &distance_column)) return false;
  // Turn each database row into a Model.
  LoadModelFunctor load_model_functor(results, model_root_);
  double distance;
  const int num_rows = results.NumRows();
  neighbors->reserve(num_rows);
  for (int row = 0; row < num_rows; ++row) {
    Model* neighbor_model = model_allocator_->Get();
    if (!load_model_functor(neighbor_model, row) ||
        !results.GetField(distance_column, row, &distance)) return false;
    neighbors->push_back(make_pair(neighbor_model, distance));
  }
  return true;
}

bool SqlDatabaseManager::SaveNeighbors(
    const Model& model, 
    const string& distance_function_name, 
    const vector<pair<Model*, double> >& neighbors) const {
  if (model.ModelName() == "") return false;
  if (distance_function_name == "") return false;
  if (&neighbors == NULL) return false;
  return false;
}

bool SqlDatabaseManager::ModelList(vector<Model*>* model_list, 
								   FilterList::FilterType filter) const {
  if (model_list == NULL) return false;
  Table results;
  if (!database_.Query("SELECT * FROM get_models() " + 
						FilterList::GetWhereClause(filter) + ";", 
						&results)) return false;
  // Turn each database row into a Model.
  LoadModelFunctor load_model_functor(results, model_root_);
  const int num_rows = results.NumRows();
  model_list->reserve(num_rows);
  for (int row = 0; row < num_rows; ++row) {
    Model* model = model_allocator_->Get();
	if (!load_model_functor(model, row)){
		return false;
	}
    model_list->push_back(model);
  }
  return true;
}

//! Return a list of some name_type, such as "distance_function" or "alignment_method"
/*! This is a helper to filter out duplicate code from the GetXXXList functions.
    The name_type is assumed to be the name of the table, and the column in the table
    is assumed to be "<name_type>_name".*/
static bool NameList(const string& name_type, 
                     const DatabaseConnection& database,
                     vector<string>* list) {
  if (list == NULL) return false;
  Table results;
  if (!database.Query("SELECT * FROM get_" + name_type + "_names();", 
      &results)) return false;
  const int num_rows = results.NumRows();
  list->reserve(num_rows);
  int name_column;
  if (!results.GetColumnIndex(name_type + "_name", &name_column)) return false;
  string name;
  for (int row = 0; row < num_rows; ++row) {
    if (!results.GetField(name_column, row, &name)) return false;
    list->push_back(name);
  }
  return true;
}

bool SqlDatabaseManager::DistanceFunctionList(
    vector<string>* distance_function_list) const {
  return NameList("distance_function", database_, distance_function_list);
}

bool SqlDatabaseManager::AlignmentMethodList(
    vector<string>* alignment_method_list) const {
  return NameList("alignment_method", database_, alignment_method_list);
}

bool SqlDatabaseManager::GraspTypeList(
	vector<string>* type_list) const {
		return NameList("grasp_type", database_, type_list);
}

bool SqlDatabaseManager::GetGrasps(const Model& model, 
                                   const string& hand_name, 
                                   vector<Grasp*>* grasp_list) const {
  if (grasp_list == NULL) return false;
  Table results;
  PROF_START_TIMER(GET_GRASPS_SQL);
  if (!database_.Query("SELECT * FROM get_grasps('" + model.ModelName() + 
                       "','" + hand_name + "');", &results)) 
    return false;
  PROF_STOP_TIMER(GET_GRASPS_SQL);
  // Get the column indices for the columns we care about.
  int pregrasp_joints_column, grasp_joints_column, 
      pregrasp_position_column, grasp_position_column,
      grasp_id_column, epsilon_quality_column, volume_quality_column,
	  grasp_contacts_column, grasp_source_name_column;
  PROF_START_TIMER(GET_GRASPS_GETCOLUMN);
  if (!results.GetColumnIndex("grasp_pregrasp_joints", &pregrasp_joints_column) ||
      !results.GetColumnIndex("grasp_grasp_joints", &grasp_joints_column) ||
      !results.GetColumnIndex("grasp_pregrasp_position", 
                              &pregrasp_position_column) ||
      !results.GetColumnIndex("grasp_grasp_position", &grasp_position_column) ||
      !results.GetColumnIndex("grasp_id", &grasp_id_column) ||
      !results.GetColumnIndex("grasp_epsilon_quality", &epsilon_quality_column) ||
      !results.GetColumnIndex("grasp_volume_quality", &volume_quality_column) ||
	  !results.GetColumnIndex("grasp_contacts", &grasp_contacts_column) ||
	  !results.GetColumnIndex("grasp_source_name", &grasp_source_name_column))
    return false;
	PROF_STOP_TIMER(GET_GRASPS_GETCOLUMN);
  // Turn each database row into a Grasp.
  vector<double> pregrasp_joints, grasp_joints, 
      pregrasp_position, grasp_position, grasp_contacts;
  int grasp_id;
  string grasp_source_name;
  double epsilon_quality, volume_quality;
  const int num_rows = results.NumRows();
  grasp_list->reserve(num_rows);
  for (int row = 0; row < num_rows; ++row) {
    grasp_list->push_back(grasp_allocator_->Get());
    Grasp& grasp = *(grasp_list->back());

	pregrasp_joints.clear();
	pregrasp_position.clear();
	grasp_joints.clear();
	grasp_position.clear();

	PROF_START_TIMER(GET_GRASPS_GETFIELD);
    if (!results.GetField(pregrasp_joints_column, row, &pregrasp_joints) || 
        !results.GetField(grasp_joints_column, row, &grasp_joints) || 
        !results.GetField(pregrasp_position_column, row, &pregrasp_position) ||
        !results.GetField(grasp_position_column, row, &grasp_position) || 
        !results.GetField(grasp_id_column, row, &grasp_id) ||
        !results.GetField(epsilon_quality_column, row, &epsilon_quality) ||
        !results.GetField(volume_quality_column, row, &volume_quality) ||
		!results.GetField(grasp_contacts_column, row, &grasp_contacts) ||
		!results.GetField(grasp_source_name_column, row, &grasp_source_name))
      return false;
	PROF_STOP_TIMER(GET_GRASPS_GETFIELD);
    grasp.SetSourceModel(model);
    grasp.SetHandName(hand_name);
    grasp.SetEpsilonQuality(epsilon_quality);
    grasp.SetVolumeQuality(volume_quality);
    grasp.SetGraspId(grasp_id);

	pregrasp_joints.erase(pregrasp_joints.begin());
	pregrasp_position.erase(pregrasp_position.begin());
	grasp_joints.erase(grasp_joints.begin());
	grasp_position.erase(grasp_position.begin());
    
	grasp.SetGraspParameters(pregrasp_joints, 
                             pregrasp_position, 
                             grasp_joints, 
                             grasp_position);
	grasp.SetContacts(grasp_contacts);
	grasp.SetPregraspJoints(pregrasp_joints);
	grasp.SetPregraspPosition(pregrasp_position);
	grasp.SetFinalgraspJoints(grasp_joints);
	grasp.SetFinalgraspPosition(grasp_position);
	grasp.SetSource(grasp_source_name);
  }
  return true;
}

bool SqlDatabaseManager::SaveGrasp(const Grasp* grasp) const {
	Table results;

	stringstream query_text;                  
	query_text << "SELECT * FROM save_grasp('" <<
	  grasp->SourceModel().ModelName() << "','" <<
	  grasp->HandName() << "', '{5, ";
	
	vector<double> tempArray;
	//pregrasp joints
	tempArray = grasp->GetPregraspJoints();
	size_t j;
	for(j = 0; j < tempArray.size() - 1; ++j){
	  query_text << tempArray[j] << ", ";
	}
	query_text << tempArray[j] << "}', ";
	
	//pregrasp position
	query_text << "'{0, ";
	tempArray = grasp->GetPregraspPosition();
	for(j = 0; j < tempArray.size() - 1; ++j){
	  query_text << tempArray[j] << ", ";
	}
	query_text << tempArray[j] << "}', ";
	
	//finalgrasp joints
	query_text << "'{5, ";
	tempArray = grasp->GetFinalgraspJoints();
	for(j = 0; j < tempArray.size() - 1; ++j){
	  query_text << tempArray[j] << ", ";
	}
	query_text << tempArray[j] << "}', ";
	
	//finalgrasp position
	query_text << "'{0, ";
	tempArray = grasp->GetFinalgraspPosition();
	for(j = 0; j < tempArray.size() - 1; ++j){
	  query_text << tempArray[j] << ", ";
	}
	query_text << tempArray[j] << "}', ";
	
	//contacts
	query_text << "'{";
	tempArray = grasp->GetContacts();
	for(j = 0; j < tempArray.size() - 1; ++j){
	  query_text << tempArray[j] << ", ";
	}
	query_text << tempArray[j] << "}', ";
	
	//energy
	query_text << -30.0 * grasp->EpsilonQuality() -100.0 * grasp->VolumeQuality() << ", ";
	
	//epsilon quality
	query_text << grasp->EpsilonQuality() << ", ";
	
	//volume
	query_text << grasp->VolumeQuality() << ", ";
	
	//source
	query_text << "'" << grasp->GetSource().c_str() << "')";
	
	//std::cout << query_text.str().c_str();
	
	if (!database_.Query(query_text.str(), &results)) return false;

	return true;
}

}  // end namespace db_planner
