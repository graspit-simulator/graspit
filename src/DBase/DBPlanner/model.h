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
// $Id: model.h,v 1.12 2009/07/22 17:59:32 coreyg Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %Model class and the %ModelAllocator class.
 */

#ifndef DB_PLANNER_MODEL_H
#define DB_PLANNER_MODEL_H

#include <string>
#include <set>
#include <vector>
using std::string;
using std::set;
using std::vector;
namespace db_planner {

//! An object representing a model from the grasp database.
/*! A Model represents a 3D model in the database along with some associated
    information that is stored when the model is created rather than
    repeatedly querying the database. */
class Model {
 private:
   //! The name of the model and absolute paths to its geometry and to a thumbnail.
   string model_name_, geometry_path_, thumbnail_path_;
   //! The approximate radius of a ball that statistically contains the model.
   double scale_, rescale_factor_;
   vector<double> symmetry_axis_;
   //! A set of descriptive tags associated with the model.
   set<string> tags_;
 public:
   virtual ~Model(){}
   //! Getter and Setters.
   const string& ModelName() const { return model_name_; }
   const string& ThumbnailPath() const { return thumbnail_path_; }
   const string& GeometryPath() const { return geometry_path_; }
   double Scale() const { return scale_; }
   double RescaleFactor() const { return rescale_factor_; }
   const set<string>& Tags() { return tags_; }
   const vector<double> &SymmetryAxis(){return symmetry_axis_;}
   void SetModelName(const string& model_name) { model_name_ = model_name; }
   void SetThumbnailPath(const string& thumbnail_path) { 
     thumbnail_path_ = thumbnail_path; 
   }
   void SetGeometryPath(const string& geometry_path) { 
     geometry_path_ = geometry_path; 
   }
   void SetScale(const double scale) { scale_ = scale; }
   void SetRescaleFactor(const double rescale_factor) { 
     rescale_factor_ = rescale_factor; 
   }
   template <class Iterator>
     void SetSymmetryAxis(const Iterator& start, const Iterator& end) {
     symmetry_axis_.clear();
     symmetry_axis_.insert(symmetry_axis_.begin(), start, end);
   }
   template <class Iterator> 
   void SetTags(const Iterator& start, const Iterator& end) { 
     tags_.clear();
     tags_.insert(start, end); 
   }
};


//! Base class to allocate Model objects.
/*! A ModelAllocator is a class that can produce new models on demand. It is
    necessary to allow a class to create derived Model types without having
    to know what those types are.*/
class ModelAllocator {
public:
  //! Return a new object of a class that derives from Model.
	virtual Model* Get() const = 0;
  virtual ~ModelAllocator() {};
};


}  // end namespace db_planner

#endif  // DB_PLANNER_MODEL_H
