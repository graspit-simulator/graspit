/*
 * UncertaintyPlannerUtil.h
 *
 *  Created on: Jun 30, 2011
 *      Author: dang
 */

#ifndef UNCERTAINTYPLANNERUTIL_H_
#define UNCERTAINTYPLANNERUTIL_H_

#include <Inventor/nodes/SoIndexedPointSet.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/fields/SoMFInt32.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>

struct augmentedVec3
{
	vec3 vector;
	double value;
};

struct UncertaintySpaceVisualizer
{
	 SoSeparator* obj;
	 SoIndexedPointSet* ps;
	 SoIndexedFaceSet* fs;
	 SoCoordinate3* coordinate;
	 SoMaterial* material;
	 SoMaterialBinding* binding;
	 SoDrawStyle* style;

	 UncertaintySpaceVisualizer(): obj(NULL), ps(NULL), coordinate(NULL), material(NULL), binding(NULL), style(NULL)
	 {}
};

#endif /* UNCERTAINTYPLANNERUTIL_H_ */
