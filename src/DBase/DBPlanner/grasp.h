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
// $Id: grasp.h,v 1.21 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %Grasp and &GraspAllocator classes.
 */


#ifndef DB_PLANNER_GRASP_H
#define DB_PLANNER_GRASP_H

#include <string>
#include <vector>
#include "model.h"
using std::string;
using std::vector;

namespace db_planner {

class Grasp {
 private: 
  //! The model being grasped
  const Model* source_model_;
  //! The id of the grasp from the database.
  int grasp_id_;
  //! The source of this grasp: EGPlanner, human operator, etc.
  string source_;
  //! The name of the hand the grasp is for.
  string hand_name_;
  //! Ferrari-Canny quality measures.
  double epsilon_quality_, volume_quality_;
  //! Quality value as used by automated grasp planners
  double energy_;
  //! The min distance between hand and object for pre-grasp
  double pregrasp_clearance_;
  //! Whether this grasp is the representative of a cluster around it
  bool cluster_rep_;
  //! The distance between the hand and a table below the object oriented along the z axis
  double table_clearance_;
  //! The actual parameters of the grasp.
  vector<double> pregrasp_joints_, final_grasp_joints_, pregrasp_position_, final_grasp_position_, contacts_;
  //! Whether this grasp is the same (modulo compliant close) with another grasp in the database (PR2 specific)
  bool compliant_copy_;
  //! If compliant_copy_ is true, the id of the original grasp that this one is similar to
  int compliant_original_id_;

 public:
  virtual ~Grasp(){}
  void SetSourceModel(const Model& source_model) { 
    source_model_ = &source_model; 
  }
  void SetHandName(const string& hand_name) { hand_name_ = hand_name; }
  void SetPregraspJoints(const vector<double>& pregraspJoints) { pregrasp_joints_ = pregraspJoints; }
  void SetPregraspPosition(const vector<double>& pregraspPosition) { pregrasp_position_ = pregraspPosition; }
  void SetFinalgraspJoints(const vector<double>& finalgraspJoints) { final_grasp_joints_ = finalgraspJoints; }
  void SetFinalgraspPosition(const vector<double>& finalgraspPosition) { final_grasp_position_ = finalgraspPosition; }
  void SetContacts(const vector<double>& contacts) { contacts_ = contacts; }

  vector<double> GetPregraspJoints() const { return pregrasp_joints_; }
  vector<double> GetPregraspPosition() const { return pregrasp_position_; }
  vector<double> GetFinalgraspJoints() const { return final_grasp_joints_; }
  vector<double> GetFinalgraspPosition() const { return final_grasp_position_; }
  vector<double> GetContacts() const { return contacts_; }

  void SetEpsilonQuality(const double epsilon_quality) {
    epsilon_quality_ = epsilon_quality;
  }
  void SetVolumeQuality(const double volume_quality) {
    volume_quality_ = volume_quality;
  }
  void SetEnergy(const double energy) {
    energy_ = energy;
  }
  void SetGraspId(const int grasp_id) { grasp_id_ = grasp_id; }
  void SetSource(string source){source_ = source;}
  string GetSource() const {return source_;}
  const Model& SourceModel() const { return *source_model_; }
  const string& HandName() const { return hand_name_; }
  double EpsilonQuality() const { return epsilon_quality_; }
  double VolumeQuality() const { return volume_quality_; }
  double Energy() const {return energy_;}
  int GraspId() const { return grasp_id_; }
  double Clearance() const {return pregrasp_clearance_;}
  void SetClearance(double clearance) {pregrasp_clearance_=clearance;}
  double TableClearance() const {return table_clearance_;}
  void SetTableClearance(double clearance) {table_clearance_ = clearance;}
  bool ClusterRep() const {return cluster_rep_;}
  void SetClusterRep(bool c) {cluster_rep_ = c;}
  bool CompliantCopy() const {return compliant_copy_;}
  void SetCompliantCopy(bool c) {compliant_copy_ = c;}
  int CompliantOriginalId() const {return compliant_original_id_;}
  void SetCompliantOriginalId(int id) {compliant_original_id_ = id;}

  //! Transform a grasp by some 4x4 transform.
  /*! Transforms are and column major (?).
      This base implementation always fails. */
  virtual bool Transform(const float /*transform*/[16]) { return false; }
  virtual bool SetGraspParameters(const vector<double>& /*pregrasp_joints*/, 
                                  const vector<double>& /*pregrasp_position*/, 
                                  const vector<double>& /*grasp_joints*/,
                                  const vector<double>& /*grasp_poisition*/) {
    return false;
  }

  static bool CompareEnergy(const Grasp* g1, const Grasp *g2) {
	  return g1->energy_ < g2->energy_;
  }                    
  static bool CompareEpsilon(const Grasp* g1, const Grasp *g2) {
	  return g1->epsilon_quality_ < g2->epsilon_quality_;
  }                    
  static bool CompareVolume(const Grasp* g1, const Grasp *g2) {
	  return g1->volume_quality_ < g2->volume_quality_;
  }                    
};

//! A class to allocate a new Grasp object, which may be a derived type.
class GraspAllocator {
public:
	virtual Grasp* Get() const = 0;
};

}
// end namespace db_planner

#endif  // DB_PLANNER_GRASP_H
