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
// $Id: ros_database_manager.cpp,v 1.9 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#include "ros_database_manager.h"

#include "household_objects_database/objects_database.h"
#include "household_objects_database/database_scaled_model.h"
#include "household_objects_database/database_grasp.h"
#include "household_objects_database/database_task.h"
#include "household_objects_database/database_file_path.h"

#include <boost/shared_ptr.hpp>
#include <sstream>
#include <QString>

//so that overloaded operators work...
using namespace database_interface;

namespace db_planner {

RosDatabaseManager::RosDatabaseManager(std::string host, std::string port, std::string user,
                                       std::string password, std::string dbname,
                                       ModelAllocator *model_allocator, GraspAllocator *grasp_allocator)
{
  model_allocator_ = model_allocator;
  grasp_allocator_ = grasp_allocator;
  database_ = new household_objects_database::ObjectsDatabase(host, port, user, password, dbname);
}

RosDatabaseManager::~RosDatabaseManager()
{
  delete model_allocator_;
  delete grasp_allocator_;
  delete database_;
}


bool RosDatabaseManager::isConnected() const
{
  return database_->isConnected();
}

void RosDatabaseManager::modelFromDBModel(Model *model,
                                          const household_objects_database::DatabaseScaledModel &db_model,
                                          std::string model_root) const
{
  model->SetModelId(db_model.id_.get());
  model->SetScale(db_model.scale_.get());
  model->SetRescaleFactor(model->Scale());
  std::ostringstream scalestr;
  scalestr << model->Scale();
  std::ostringstream idstr;
  idstr << db_model.original_model_id_.data();
  model->SetModelName(idstr.str() + "_" + db_model.model_.data() + "_" + scalestr.str());
  model->SetTags(db_model.tags_.data().begin(), db_model.tags_.data().end());

  //see what kind of files we have in the database for this model
  std::vector< boost::shared_ptr<household_objects_database::DatabaseFilePath> > paths;
  std::stringstream id;
  id << db_model.original_model_id_.data();
  std::string where_clause("original_model_id=" + id.str());
  if (!database_->getList<household_objects_database::DatabaseFilePath>(paths, where_clause)) {
    std::cerr << "Failed to load file paths from database\n";
    return;
  }
  for (size_t i = 0; i < paths.size(); i++) {
    if (paths[i]->file_type_.data() == "THUMBNAIL_BINARY_PNG") {
      model->SetThumbnailPath(model_root + paths[i]->file_path_.data());
    } else if (paths[i]->file_type_.data() == "GEOMETRY_BINARY_PLY") {
      model->SetGeometryPath(model_root + paths[i]->file_path_.data());
    }
  }
}

bool RosDatabaseManager::ModelList(vector<Model *> *model_list, FilterList::FilterType filter) const
{
  Q_UNUSED(filter);

  std::string model_root;
  if (!database_->getModelRoot(model_root)) { return false; }
  std::vector< boost::shared_ptr<household_objects_database::DatabaseScaledModel>  >db_model_list;
  if (!database_->getScaledModelsList(db_model_list)) { return false; }
  for (size_t i = 0; i < db_model_list.size(); i++)
  {
    Model *model = model_allocator_->Get();
    modelFromDBModel(model, *(db_model_list[i]), model_root);
    model_list->push_back(model);
  }
  return true;
}

bool RosDatabaseManager::ScaledModel(Model *&model, int scaled_model_id) const
{
  std::string model_root;
  if (!database_->getModelRoot(model_root)) { return false; }
  std::stringstream id;
  id << scaled_model_id;
  std::string where_clause("scaled_model_id=" + id.str());
  std::vector< boost::shared_ptr<household_objects_database::DatabaseScaledModel>  >db_model_list;
  if (!database_->getList<household_objects_database::DatabaseScaledModel>(db_model_list, where_clause)) { return false; }
  if (db_model_list.size() != 1) { return false; }
  model = model_allocator_->Get();
  modelFromDBModel(model, *(db_model_list[0]), model_root);
  return true;
}

bool RosDatabaseManager::AcquireNextTask(TaskRecord *rec, std::vector<std::string> accepted_types)
{
  std::vector< boost::shared_ptr<household_objects_database::DatabaseTask> > db_task;
  if (!database_->acquireNextTask(db_task, accepted_types)) { return false; }
  if (db_task.empty())
  {
    //no task to be run
    rec->taskType = "";
    return true;
  }
  //populate the entry
  rec->taskType = db_task[0]->type_.data();
  rec->taskId = db_task[0]->id_.data();
  return true;
}

bool RosDatabaseManager::SetTaskStatus(int task_id, const string &status)
{
  household_objects_database::DatabaseTask db_task;
  db_task.id_.data() = task_id;
  db_task.outcome_name_.data() = status;
  if (!database_->saveToDatabase(&(db_task.outcome_name_))) { return false; }
  return true;
}

bool RosDatabaseManager::GetPlanningTaskRecord(int task_id, PlanningTaskRecord *rec)
{
  household_objects_database::DatabasePlanningTask planningTask;
  planningTask.id_.data() = task_id;
  if (!database_->loadFromDatabase(&(planningTask.scaled_model_id_)) ||
      !database_->loadFromDatabase(&(planningTask.hand_name_)) ||
      !database_->loadFromDatabase(&(planningTask.time_)))
  {
    std::cout << "Failed to load planning task for task id " << task_id << "\n";
    return false;
  }
  rec->taskId = task_id;
  rec->taskTime = planningTask.time_.data();
  rec->handName = planningTask.hand_name_.data();
  //load the actual model
  //a fairly hacky way to do it for now, field-by-field
  household_objects_database::DatabaseScaledModel db_model;
  db_model.id_.data() = planningTask.scaled_model_id_.data();
  //first we need to read the original model id so we can then read the rest
  if (!database_->loadFromDatabase(&(db_model.original_model_id_))) { return false; }
  if (!database_->loadFromDatabase(&(db_model.scale_)) ||
      !database_->loadFromDatabase(&(db_model.tags_)))
  {
    return false;
  }
  Model *model = model_allocator_->Get();
  std::string model_root;
  if (!database_->getModelRoot(model_root)) { return false; }
  modelFromDBModel(model, db_model, model_root);
  rec->model = model;
  return true;
}

bool RosDatabaseManager::GetOptimizationTaskRecord(int task_id, OptimizationTaskRecord *rec)
{
  household_objects_database::DatabaseOptimizationTask optTask;
  optTask.dbase_task_id_.data() = task_id;
  if (!database_->loadFromDatabase(&(optTask.parameters_)) ||
      !database_->loadFromDatabase(&(optTask.hand_name_)))
  {
    std::cout << "Failed to load optimization task for task id " << task_id << "\n";
    return false;
  }
  rec->taskId = task_id;
  rec->parameters = optTask.parameters_.data();
  rec->hand_name = optTask.hand_name_.data();
  return true;
}

bool RosDatabaseManager::SaveOptimizationResults(const OptimizationTaskRecord &rec,
                                                 const std::vector<double> &parameters,
                                                 const std::vector<double> &results,
                                                 const std::vector<double> &seed)
{
  household_objects_database::DatabaseOptimizationResult optResult;
  optResult.dbase_task_id_.data() = rec.taskId;
  optResult.parameters_.data() = parameters;
  optResult.hand_name_.data() = rec.hand_name;
  optResult.results_.data() = results;
  optResult.seed_.data() = seed;

  if (!database_->insertIntoDatabase(&optResult))
  {
    std::cout << "Failed to save optimization results for task id " << rec.taskId << "\n";
    return false;
  }
  return true;
}

bool RosDatabaseManager::GraspTypeList(vector<string> *type_list) const
{
  type_list->push_back("ALL");
  return true;
}

bool RosDatabaseManager::GetGrasps(const Model &model, const string &hand_name,
                                   vector<Grasp *> *grasp_list) const
{
  std::vector< boost::shared_ptr<household_objects_database::DatabaseGrasp> > db_grasp_list;
  if (!database_->getGrasps(model.ModelId(), hand_name, db_grasp_list)) { return false; }
  std::cerr << "Loading grasps for hand " << hand_name << " on model " << model.ModelId() << "\n";
  for (size_t i = 0; i < db_grasp_list.size(); i++)
  {
    Grasp *grasp = grasp_allocator_->Get();
    const household_objects_database::DatabaseGrasp &db_grasp = *(db_grasp_list[i]);

    grasp->SetSourceModel(model);
    grasp->SetHandName(hand_name);

    grasp->SetGraspId(db_grasp.id_.get());
    grasp->SetEnergy(db_grasp.quality_.get());
    grasp->SetEpsilonQuality(0.0);
    grasp->SetVolumeQuality(0.0);
    grasp->SetClearance(db_grasp.pre_grasp_clearance_.get());
    grasp->SetClusterRep(db_grasp.cluster_rep_.get());
    grasp->SetHandName(db_grasp.hand_name_.get());
    grasp->SetTableClearance(db_grasp.table_clearance_.get());
    grasp->SetCompliantCopy(db_grasp.compliant_copy_.get());
    grasp->SetCompliantOriginalId(db_grasp.compliant_original_id_.get());

    std::stringstream str;
    std::vector<double> pregrasp_joints;
    str << db_grasp.pre_grasp_posture_.get();
    str >> pregrasp_joints;
    if (str.fail()) { return false; }
    std::vector<double> pregrasp_position;
    str << db_grasp.pre_grasp_pose_.get();
    str >> pregrasp_position;
    if (str.fail()) { return false; }
    std::vector<double> grasp_joints;
    str << db_grasp.final_grasp_posture_.get();
    str >> grasp_joints;
    if (str.fail()) { return false; }
    std::vector<double> grasp_position;
    str << db_grasp.final_grasp_pose_.get();
    str >> grasp_position;
    if (str.fail()) { return false; }

    //convert from ROS units to Graspit units
    pregrasp_position[0] *= 1000;
    pregrasp_position[1] *= 1000;
    pregrasp_position[2] *= 1000;
    grasp_position[0] *= 1000;
    grasp_position[1] *= 1000;
    grasp_position[2] *= 1000;

    grasp->SetGraspParameters(pregrasp_joints,
                              pregrasp_position,
                              grasp_joints,
                              grasp_position);
    grasp->SetPregraspJoints(pregrasp_joints);
    grasp->SetPregraspPosition(pregrasp_position);
    grasp->SetFinalgraspJoints(grasp_joints);
    grasp->SetFinalgraspPosition(grasp_position);

    grasp_list->push_back(grasp);
  }
  std::cerr << "Loaded " << grasp_list->size() << " grasps\n";
  return true;
}

bool RosDatabaseManager::SaveGrasp(const Grasp *grasp) const
{
  household_objects_database::DatabaseGrasp db_grasp;
  db_grasp.scaled_model_id_.get() = grasp->SourceModel().ModelId();
  db_grasp.quality_.get() = grasp->Energy();
  db_grasp.pre_grasp_clearance_.get() = grasp->Clearance();
  db_grasp.cluster_rep_.get() = grasp->ClusterRep();
  db_grasp.hand_name_.get() = grasp->HandName();
  db_grasp.table_clearance_.get() = grasp->TableClearance();
  db_grasp.compliant_copy_.get() = grasp->CompliantCopy();
  db_grasp.compliant_original_id_.get() = grasp->CompliantOriginalId();
  db_grasp.scaled_quality_.data() = std::max(0.0, 1.0 - std::max(0.0, grasp->Energy()) / 68.6);

  std::stringstream str;
  str << grasp->GetPregraspJoints();
  str >> db_grasp.pre_grasp_posture_.get();
  if (str.fail()) { return false; }
  str << grasp->GetPregraspPosition();
  str >> db_grasp.pre_grasp_pose_.get();
  if (str.fail()) { return false; }
  //convert from Graspit units to ROS units
  db_grasp.pre_grasp_pose_.get().pose_.position.x /= 1000;
  db_grasp.pre_grasp_pose_.get().pose_.position.y /= 1000;
  db_grasp.pre_grasp_pose_.get().pose_.position.z /= 1000;

  str << grasp->GetFinalgraspJoints();
  str >> db_grasp.final_grasp_posture_.get();
  if (str.fail()) { return false; }
  str << grasp->GetFinalgraspPosition();
  str >> db_grasp.final_grasp_pose_.get();
  if (str.fail()) { return false; }
  //convert from Graspit units to ROS units
  db_grasp.final_grasp_pose_.get().pose_.position.x /= 1000;
  db_grasp.final_grasp_pose_.get().pose_.position.y /= 1000;
  db_grasp.final_grasp_pose_.get().pose_.position.z /= 1000;

  if (!database_->insertIntoDatabase(&db_grasp)) { return false; }
  return true;
}

bool RosDatabaseManager::DeleteGrasp(Grasp *grasp) const
{
  household_objects_database::DatabaseGrasp db_grasp;
  db_grasp.id_.get() = grasp->GraspId();
  if (!database_->deleteFromDatabase(&db_grasp)) { return false; }
  return true;
}

bool RosDatabaseManager::SetGraspClusterRep(Grasp *grasp, bool rep) const
{
  household_objects_database::DatabaseGrasp db_grasp;
  db_grasp.id_.get() = grasp->GraspId();
  db_grasp.cluster_rep_.get() = rep;
  if (!database_->saveToDatabase(&db_grasp.cluster_rep_)) { return false; }
  return true;
}
bool RosDatabaseManager::SetGraspTableClearance(Grasp *grasp, double clearance) const
{
  household_objects_database::DatabaseGrasp db_grasp;
  db_grasp.id_.get() = grasp->GraspId();
  db_grasp.table_clearance_.get() = clearance;
  if (!database_->saveToDatabase(&db_grasp.table_clearance_)) { return false; }
  return true;
}

bool RosDatabaseManager::InsertGraspPair(const Grasp *grasp1, const Grasp *grasp2) const
{
  Q_UNUSED(grasp1);
  Q_UNUSED(grasp2);
  //grasp pair not used anymore
  return false;
  /*
  household_objects_database::DatabaseGraspPair db_pair;
  db_pair.grasp1_id_.get() = grasp1->GraspId();
  db_pair.grasp2_id_.get() = grasp2->GraspId();
  if ( !database_->insertIntoDatabase(&db_pair) ) return false;
  return true;
  */
}

bool RosDatabaseManager::LoadModelGeometry(Model *model) const
{
  household_objects_database::DatabaseMesh mesh;
  if (!database_->getScaledModelMesh(model->ModelId(), mesh)) { return false; }
  //this gets the geometry in ROS units, which are meters
  //scale to mm
  for (size_t i = 0; i < mesh.vertices_.data().size(); i++) {
    mesh.vertices_.data().at(i) = 1000 * mesh.vertices_.data().at(i);
  }
  model->SetGeometry(mesh.vertices_.data(), mesh.triangles_.data());
  return true;
}

QString RosDatabaseManager::getHandGraspitPath(QString handDBName) const
{
  std::string path;
  std::vector<boost::shared_ptr<household_objects_database::HandFilePath> > file_paths;
  std::stringstream where;
  where << "hand_name = '" << handDBName.toStdString() << "'";
  std::string where_clause(where.str());
  if (!database_->getList<household_objects_database::HandFilePath>(file_paths, where_clause) ||
      file_paths.empty() ||
      file_paths[0]->file_path_.data().empty()) {
    path = "/models/robots/" + handDBName.toStdString() + "/" + handDBName.toStdString() + ".xml";
  } else {
    path = file_paths[0]->file_path_.data();
  }
  return QString::fromStdString(path);
}

} //namespace db_planner
