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
// $Id: graspit_db_model.h,v 1.10 2010/08/11 02:45:37 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBModel class
 */

#ifndef _GRASPIT_DB_MODEL_ENTRY_H_
#define _GRASPIT_DB_MODEL_ENTRY_H_

#include "DBPlanner/model.h"

class World;
class GraspableBody;

/*! This is the class to define the data entry of model in CGDB
*/
class GraspitDBModel : public db_planner::Model{
protected:
	//! This is the body representation in GraspIt
	GraspableBody* mGraspableBody;
	//! Tells us if the scene graph geometry of this object has been loaded
	bool mGeometryLoaded;

        //! Loads the geometry for this body
        virtual int loadGeometry();
public:
	GraspitDBModel() : mGraspableBody(NULL), mGeometryLoaded(false){}
	~GraspitDBModel();
	//! Loads the geometry and initializes the corresponding GraspableBody
	int load(World* w);
	//! Deletes the loaded geometry; 
	void unload();
	//! Returns the flag that tells us if geometry has been loaded
	bool geometryLoaded() const {return mGeometryLoaded;}
	//! Returns the Graspable body
	GraspableBody* getGraspableBody() const { return mGraspableBody; }
};

//! An implementation of ModelAllocator that returns new GraspitDBModel objects.
class GraspitDBModelAllocator : public db_planner::ModelAllocator
{
public:
	GraspitDBModelAllocator(){}
	db_planner::Model* Get() const {
		return new GraspitDBModel();
	}
};

namespace db_planner {class DatabaseManager;}

//! Specialized class a version of the model which loads its geometry dirrectly from the database
/*! Might be integrated into the class above at some point */
class GeomGraspitDBModel : public GraspitDBModel
{
private:
  //! Pointer to the db manager which can load the geometry on demand
  db_planner::DatabaseManager *mManager;
  //! Gets the geometry directly form the database, in binary format
  virtual int loadGeometry();
public:
  //! Emty Stub
  GeomGraspitDBModel(db_planner::DatabaseManager* manager) : GraspitDBModel(), mManager(manager) {}
};

//! Allocator for the specialized model that loads geometry from the database
class GeomGraspitDBModelAllocator : public db_planner::ModelAllocator
{
private:
  //! Pointer to the db manager to be passed onto the models
  db_planner::DatabaseManager *mManager;
public:
  //! Empty stub
  GeomGraspitDBModelAllocator(db_planner::DatabaseManager *manager) : mManager(manager) {}

  //! Returns a Ros db model with the manager set correctly
  db_planner::Model* Get() const {
    return new GeomGraspitDBModel(mManager);
  }
};

#endif
