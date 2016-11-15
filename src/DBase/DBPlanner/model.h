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
// $Id: model.h,v 1.14 2010/08/10 17:24:56 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the %Model class and the %ModelAllocator class.
 */

#ifndef DB_PLANNER_MODEL_H
#define DB_PLANNER_MODEL_H

#include <string>
#include <vector>
#include <set>
using std::string;
using std::set;

namespace db_planner {

//! An object representing a model from the grasp database.
/*! A Model represents a 3D model in the database along with some associated
    information that is stored when the model is created rather than
    repeatedly querying the database. */
class Model {
 private:
   //! The id of the model in the database
    int model_id_;
   //! The name of the model and absolute paths to its geometry and to a thumbnail.
   string model_name_, geometry_path_, thumbnail_path_;
   //! The approximate radius of a ball that statistically contains the model.
   double scale_, rescale_factor_;
   //! A set of descriptive tags associated with the model.
   set<string> tags_;
  //! The vertices of the model's mesh, if loaded directly from the database
  std::vector<double> vertices_;
  //! The triangles of the model's mesh, if loaded directly from the database
  std::vector<int> triangles_;
 public:
   virtual ~Model(){}
   //! Getter and Setters.
   const string& ModelName() const { return model_name_; }
   const string& ThumbnailPath() const { return thumbnail_path_; }
   const string& GeometryPath() const { return geometry_path_; }
   double Scale() const { return scale_; }
   double RescaleFactor() const { return rescale_factor_; }
   int ModelId() const { return model_id_;}
   void SetModelId(int id) {model_id_ = id;}
   const set<string>& Tags() { return tags_; }
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
   void SetTags(const Iterator& start, const Iterator& end) { 
     tags_.clear();
     tags_.insert(start, end); 
   }
  void SetGeometry(const std::vector<double> &vertices, const std::vector<int> &triangles) {
    vertices_ = vertices;
    triangles_ = triangles;
  }
  const std::vector<double>& GetVertices() const {return vertices_;}
  const std::vector<int>& GetTriangles() const {return triangles_;}
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
