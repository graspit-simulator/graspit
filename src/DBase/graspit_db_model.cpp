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
// $Id: graspit_db_model.cpp,v 1.15 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %GraspitDBModel class
 */

#include "graspit_db_model.h"

#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoMaterial.h>
#include <qfile.h>
#include <string>
#include "model_utils.h"
#include "mytools.h"
#include "body.h"
#include "world.h"
//#define GRASPITDBG
#include "debug.h"

GraspitDBModel::GraspitDBModel(GraspableBody *b){
	mGraspableBody = b;
	mGeometryLoaded = true;
	this->SetModelName(b->getName().toStdString());
}

GraspitDBModel::~GraspitDBModel()
{
	if(mGraspableBody) delete mGraspableBody;
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



	//load the geometry itself directly from the PSB file
	int result;
	QString filename = QString(GeometryPath().c_str());
	QString extension = filename.section('.',-1,-1);
	if(QFile(filename).exists() || modelUtils::repairModelGeometry(this)){
		if (extension == "off") {
			result = mGraspableBody->loadGeometryOFF(filename);
	} else if (extension == "iv" || extension == "wrl") {
			result = mGraspableBody->loadGeometryIV(filename);
		} else if (extension == "ply") {
			result = mGraspableBody->loadGeometryPLY(filename);
		} else {
			DBGA("Uknown extension: " << extension.latin1());
			result = FAILURE;
		}
	}
	else {
		DBGA("Uknown file " << filename.latin1());
		result = FAILURE;
	}
	if (result != SUCCESS) {
		mGeometryLoaded = false;
		DBGA("Failed to load CGDB model from file " << GeometryPath());
		return result;
	}
	mGeometryLoaded = true;
	mGraspableBody->addIVMat();

	//PSB objects have a scale of their own. To get to "graspable size"
	//we manually set a scaling factor for each of them, which is in the
	//database as well. We need to scale the geometry appropriately
	SoScale* scale = new SoScale();
	scale->scaleFactor.setValue(RescaleFactor(), RescaleFactor(), RescaleFactor());
	//std::cout << "scale is : " << RescaleFactor() << std::endl;
	mGraspableBody->getIVGeomRoot()->insertChild(scale,0);

	mGeometryLoaded = true;
	mGraspableBody->addIVMat();

	//set the dynamic properties. This needs a better solution...
	mGraspableBody->setDefaultDynamicParameters();
	//hard-coded inertia matrix of the flask...
	double I[] = {4853.0, -1.1196, -6.5156, -1.1196, 4853.0, 47.542, -6.5156, 0.0, 2357.6};
	mGraspableBody->setInertiaMatrix(I);
	mGraspableBody->setMaxRadius(mGraspableBody->computeDefaultMaxRadius());
	mGraspableBody->setMass(300);

	return result;
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
