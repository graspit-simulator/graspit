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
// $Id: graspit_db_model.cpp,v 1.16 2010/08/10 17:23:59 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBModel class
 */

#include "graspit_db_model.h"

#include <Inventor/nodes/SoScale.h>

#include <string>

#include "mytools.h"
#include "body.h"
#include "world.h"
//#define GRASPITDBG
#include "debug.h"

// for loading geometry directly from the database
#include "DBPlanner/db_manager.h"

GraspitDBModel::~GraspitDBModel()
{
	if(mGraspableBody) delete mGraspableBody;
}

int GraspitDBModel::loadGeometry()
{
	//load the geometry itself directly from the geometry file stored in the database
	QString filename = QString(GeometryPath().c_str());
	QString extension = filename.section('.',-1,-1);
	if (extension == "off") {
		DBGA("Failed to load .off geometry from file " << GeometryPath());
		return mGraspableBody->loadGeometryOFF(filename);
	} else if (extension == "iv") {
		DBGA("Failed to load .iv geometry from file " << GeometryPath());
		return mGraspableBody->loadGeometryIV(filename);
	} else if (extension == "ply") {
		DBGA("Failed to load .ply geometry from file " << GeometryPath());
		return mGraspableBody->loadGeometryPLY(filename);
	} else {
		DBGA("Uknown geometry file extension: " << extension.latin1());
		return FAILURE;
	}
}

int GraspitDBModel::load(World* w)
{
	// delete the previously loaded graspabody
	if(mGraspableBody) delete mGraspableBody;
	// load the body
	mGraspableBody = new GraspableBody(w, ModelName().c_str());
	mGraspableBody->setDBModel(this);
	//material is default
	mGraspableBody->setMaterial(w->getMaterialIdx("wood"));

	//PSB objects have a scale of their own. To get to "graspable size"
	//we manually set a scaling factor for each of them, which is in the
	//database as well. We need to scale the geometry appropriately
	SoScale* scale = new SoScale();
	scale->scaleFactor.setValue(RescaleFactor(), RescaleFactor(), RescaleFactor());
	mGraspableBody->getIVGeomRoot()->addChild(scale);

	if (loadGeometry() != SUCCESS) {
		mGeometryLoaded = false;
		return FAILURE;
	}
	mGeometryLoaded = true;
	mGraspableBody->addIVMat();

	//set the dynamic properties. This needs a better solution...
	mGraspableBody->setDefaultDynamicParameters();
	//hard-coded inertia matrix of the flask...
	double I[] = {4853.0, -1.1196, -6.5156, -1.1196, 4853.0, 47.542, -6.5156, 0.0, 2357.6};
	mGraspableBody->setInertiaMatrix(I);
	mGraspableBody->setMaxRadius(mGraspableBody->computeDefaultMaxRadius());
	mGraspableBody->setMass(300);

	return SUCCESS;
}

/*! It will NOT remove the GraspIt equivalent of this model from its world, or
    from the collision detection. It is the CALLER's responsability to take
    care of that. This is symmetrical with load(), which prepares the geometry
    but does not add it to the world or the collision detection.
*/
void GraspitDBModel::unload()
{
	delete mGraspableBody; mGraspableBody = NULL;
	mGeometryLoaded = false;
}

/*! Uses the stored vectors of vertices and triangles to initialize geometry; if
  these are not yet in memory, will first ask the manager to load them.
*/
int GeomGraspitDBModel::loadGeometry()
{
  // if geometry has not been already loaded, ask the manager to load it
  if (GetVertices().empty()) {
    if (!mManager) {
      DBGA("Cannot load geometry from database; missing manager");
      return FAILURE;
    }
    if (!mManager->LoadModelGeometry(this)) {
      DBGA("Manager failed to load geometry for model from database");
      return FAILURE;
    }
    if (GetVertices().empty() || GetTriangles().empty()) {
      DBGA("Empty geometry loaded from database");
      return FAILURE;
    }
  }
  //convert geometry to a format that graspit understands and load it
  std::vector<position> vertices;
  if ( GetVertices().size() % 3 != 0 ) {
    DBGA("Load model geometry from database: size of vertices vector is not a multiple of 3");
    return FAILURE;
  }
  for (size_t i=0; i<GetVertices().size()/3; i++) {
    vertices.push_back( position(GetVertices().at(3*i+0), GetVertices().at(3*i+1), GetVertices().at(3*i+2)) );
  }

  return mGraspableBody->loadGeometryMemory(vertices, GetTriangles());
}

