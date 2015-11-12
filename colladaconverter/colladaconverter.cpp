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
// Modified from ColladaLoader by Ricardo Ruiz Lopez
//
//
// Author(s): Norases (Saint) Vesdapunt
//
//
//######################################################################
#include "colladaconverter.h"

int main(int argc, char* argv[]){
	if(argc<2){
		DBGA("Usage: colladaconverter filename");
		return FAILURE;
	}
	for(int i=1;i<argc;i++){
		string filename = argv[i];
		//replace '\' with '/'
		string sought = "\\";
		while(filename.find(sought)!=-1){
			filename.replace(filename.find(sought),sought.length(),"/");
		}
		DBGA("Converting "<<filename);
		string fileType = filename.substr(filename.find_last_of(".")+1);
		if(fileType == "dae"){
			if(convertCollada2Off(filename)==FAILURE){
				DBGA("Error Converting COLLADA to OFF");
				return FAILURE;
			}
		}
		else {
			DBGA("Incompaptible file format: dae only");
			return FAILURE;
		}
	}
	return SUCCESS;
}


int convertCollada2Off(string filename){
	// generic vector 3d
	struct vector3d {   
	   float x;   
	   float y;
	   float z;
	};
	int m_num_vertices;
	struct vector3d *m_ptrs_vertices;

	FCDocument* m_document;
	FCollada::Initialize();
	m_document = FCollada::NewTopDocument();
	//open dae file
	fstring input_filename = TO_FSTRING(filename.c_str());
	bool ret=FCollada::LoadDocumentFromFile(m_document, input_filename);
	
	if (ret==false || m_document==NULL) {
		DBGA("File not found");
		return FAILURE;
	}
	std::ofstream myfile;
	string writeFile = filename.substr(0,filename.find_last_of("."))+".off";
	myfile.open(writeFile.c_str());
	myfile<<"OFF"<<std::endl;
	FCDGeometryLibrary* geolib=m_document->GetGeometryLibrary();
	// count the number of meshes (nurbs and splines are included in geolib, but discard them)
	int m_num_geometries=0;
	int i,j,k;
	for (i=0; i< (int) geolib->GetEntityCount(); i++)
		if (geolib->GetEntity(i)->IsMesh()==true) m_num_geometries++;
	// copy all geometry objects from fcollada to my structures
	FCDGeometry* geo;
	FCDGeometryMesh* mesh;
	int m_num_polygons=0;
	FCDGeometryPolygons* ptr_polygons;
	float* p;
	int index;
	FCDGeometrySource* source;
	FCDGeometryPolygonsInput* geometrypolygonsinput;
	uint32* indices;
	//triangulate all of the meshes
	//count number of vertices (a triangle = 3 vertices) and faces
	int total_vertices=0;
	std::stringstream myStringStream;
	for (i=0; i<(int) geolib->GetEntityCount(); i++) { 
		geo = geolib->GetEntity(i);
		// there are 3 types of geometries, nurbs, splines and meshes
		// at the moment JUST meshes, neither nurbs nor splines

		// meshes
		if (geo->IsMesh()) {
			mesh = geo->GetMesh();
			// triangulate this mesh if it was not triangulated
			if (!mesh->IsTriangles()) FCDGeometryPolygonsTools::Triangulate(mesh);
			m_num_polygons=(int) mesh->GetPolygonsCount();
			for (j=0;j<m_num_polygons;j++) {
				ptr_polygons = mesh->GetPolygons(j);

				/////////////////////////////////////////////
				//
				//            VERTEX
				//
				/////////////////////////////////////////////

				// indices to vertex
				geometrypolygonsinput=ptr_polygons->FindInput(FUDaeGeometryInput::POSITION);
				indices = geometrypolygonsinput->GetIndices();

				//m_num_vertices=(int) indices->size();
				m_num_vertices=(int) geometrypolygonsinput->GetIndexCount();
				total_vertices += m_num_vertices;
				// source of vertex
				source = ptr_polygons->GetParent()->FindSourceByType(FUDaeGeometryInput::POSITION);

				// allocate memory for triangles and its vertex (a triangle has 3 vertex)
				m_ptrs_vertices = (struct vector3d *)malloc( m_num_vertices * sizeof(struct vector3d) );

				// look for vertices using indices
				// 3 contiguous vertex form a triangle
				for (k=0; k<m_num_vertices; k++) {
					// a vertex index
					index=(int) indices[k];
					// a vertex values from it index
					p=&source->GetData()[index*3];
					m_ptrs_vertices[k].x=p[0];
					m_ptrs_vertices[k].y=p[1];
					m_ptrs_vertices[k].z=p[2];
					myStringStream<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;
				}

			}
		}

	}
	int total_faces = total_vertices/3;//triangles
	//OFF Format: numVertices numFaces numEdges
	myfile<<total_vertices<<" "<<total_faces<<" 0"<<std::endl;
	//vertices
	myfile<<myStringStream.str();
	// triangle faces
	for(i=0;i<total_faces*3;i+=3){
		myfile<<"3 "<<i<<" "<<i+1<<" "<<i+2<<std::endl;
	}
	SAFE_DELETE(m_document);
	myfile.close();
	writeFile = writeFile.substr(writeFile.find_last_of("/")+1);
	//xml file is also generated
	string xmlFile = filename.substr(0,filename.find_last_of("."))+".xml";
	myfile.open(xmlFile.c_str());
	myfile<<"<?xml version=\"1.0\" ?>"<<endl;
	myfile<<"<root>"<<endl;
	myfile<<"\t<geometryFile type=\"off\">"<<writeFile<<"</geometryFile>"<<endl;
	myfile<<"</root>"<<endl;
	myfile.close();

	return SUCCESS;
}