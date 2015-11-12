#ifndef _GEOMETRY_LIB_INTERFACE_H_
#define _GEOMETRY_LIB_INTERFACE_H_


#include <vector>
/*
	This file interface GraspIt! with the Primitive utilities and fitter
	developed in the Columbia Robotics Lab
*/

//#include "../GeometryLibrary/geometry/Vertex.h"
//#include "../GeometryLibrary/geometry/triangle.h"
#include "geometry/gaussian_integrable_triangle.h"

#include "geometry/mesh.h"
#include "primitives/superquadric.h"
#include "primitives/superquadric_cost.h"
#include "fitter/threaded_residualizer.h"
#include "levmar_fitter/levmar_fitter.h"
#include "split_merge_fitter/split_merge_fitter.h"
#include "globals/debug.h"

//namespace geometry { template<class real=double> class Triangle{ real a; };}


class SoNode;
class SoGroup;

void SoGeometryToTriangleList(SoNode *root, std::vector<geometry::IntegrableTriangle<>*> *triangles);
void triangleListToSoGeometry(SoGroup *root, std::vector<geometry::IntegrableTriangle<>*> *triangles);
void primitivesToTriangleList(std::vector<geometry::IntegrableTriangle<>*> *output_vec,
	std::list<const primitives::Primitive<>*>* primitives);

#endif