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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: graspit_db_model.h,v 1.8 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBModel class
 */

#ifndef _GRASPIT_DB_MODEL_ENTRY_H_
#define _GRASPIT_DB_MODEL_ENTRY_H_

#include <QString>

#include "DBPlanner/model.h"

class World;
class GraspableBody;

/*! This is the class to define the data entry of model in CGDB
*/
class GraspitDBModel : public db_planner::Model{
private:
	//! Tells us if the scene graph geometry of this object has been loaded
	bool mGeometryLoaded;
	//! This is the body representation in GraspIt
	GraspableBody* mGraspableBody;
public:
	GraspitDBModel() : mGraspableBody(NULL), mGeometryLoaded(false){}
	GraspitDBModel(GraspableBody* b);
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

#endif