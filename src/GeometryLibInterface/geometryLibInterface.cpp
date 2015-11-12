#include "GeometryLibInterface/geometryLibInterface.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoMaterial.h>

#include "debug.h"

void addTriangleCallBack(void* info, SoCallbackAction * action,
               const SoPrimitiveVertex * v1, const SoPrimitiveVertex * v2, const SoPrimitiveVertex * v3)
{
	std::vector<geometry::IntegrableTriangle<>*> *triangles = (std::vector<geometry::IntegrableTriangle<>*>*) info;

	SbVec3f p1, p2, p3;
	SbMatrix mm = action->getModelMatrix();

	// Transform verticies (remember verticies are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	mm.multVecMatrix( v2->getPoint(), p2 );
	mm.multVecMatrix( v3->getPoint(), p3 );

	// Don't add degenerate triangles!
	if ((p1 == p2) || (p2 == p3) || (p1 == p3)) {    
		return;
	}

	geometry::Vertex<>* vert1 = new geometry::Vertex<>(p1[0], p1[1], p1[2]);
	geometry::Vertex<>* vert2 = new geometry::Vertex<>(p2[0], p2[1], p2[2]);
	geometry::Vertex<>* vert3 = new geometry::Vertex<>(p3[0], p3[1], p3[2]);
	geometry::IntegrableTriangle<>* newTri = new geometry::GaussianIntegrableTriangle<7>(*vert1, *vert2, *vert3);
	triangles->push_back( newTri );
}

void SoGeometryToTriangleList(SoNode *root, std::vector<geometry::IntegrableTriangle<>*> *triangles)
{ 
	//empty triangle list
	triangles->clear();
	
	//set up the callback and run it
	SoCallbackAction ca;
	ca.addTriangleCallback(SoShape::getClassTypeId(), addTriangleCallBack, triangles);
	ca.apply(root);
	
}

void triangleListToSoGeometry(SoGroup *root, std::vector<geometry::IntegrableTriangle<>*> *triangles)
{ 
	int numTriangles = triangles->size();
	fprintf(stderr,"Triangles: %d \n",numTriangles);
	SbVec3f *points = new SbVec3f[ 3*numTriangles ];
	int32_t *coordIndex = new int32_t[ 4*numTriangles ];

	std::vector<geometry::IntegrableTriangle<>*>::iterator it;
	int index = 0;
	for (it = triangles->begin(); it != triangles->end(); it++) {
		for (int j=0; j<3; j++) {
			points[3*index+j].setValue( (float)(*it)->GetVertex(j).x(), 
										(float)(*it)->GetVertex(j).y(), 
										(float)(*it)->GetVertex(j).z());

			coordIndex[4*index+j] = 3*index+j;
		}
		coordIndex[4*index+3] = -1;
		index++;
	}

	SoMaterial *mat = new SoMaterial;
    mat->diffuseColor = SbColor(0.8f,0.0f,0.0f);
    mat->ambientColor = SbColor(0.2f,0.0f,0.0f);
    mat->emissiveColor = SbColor(0.4f,0.0f,0.0f);
    mat->transparency = 0.0f;
	root->addChild(mat);


	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 3*numTriangles, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 4*numTriangles, coordIndex);

	root->addChild(coords);
	root->addChild(ifs);
	
}


void primitivesToTriangleList(std::vector<geometry::IntegrableTriangle<>*> *output_vec,
	std::list<const primitives::Primitive<>*>* primitives) {
 for (std::list<const primitives::Primitive<>*>::iterator p = primitives->begin();
     p != primitives->end(); ++p) {
   std::list<geometry::GaussianIntegrableTriangle<>*> triangle_list;
   (*p)->Triangulate(triangle_list,30);
   for(std::list<geometry::GaussianIntegrableTriangle<>*>::iterator tri = triangle_list.begin();
	   tri != triangle_list.end(); ++tri) {
     output_vec->push_back(*tri);
   }
 }
}


