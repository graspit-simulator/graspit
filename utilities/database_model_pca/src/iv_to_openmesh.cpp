#include "iv_to_openmesh.h"
#include <fstream>
#include <string>
#include "openmesh.h"
#include <vector>

using std::string;
ivMeshifier::ivMeshifier(){}

bool ivMeshifier::meshify(std::ifstream & ivstream, Mesh * m){
  //move forward until you find the vertex list
  string current;
  do{
    ivstream >> current;
  }while(current.compare("point"));
  
  ivstream >> current;
  if(current.compare("["))
    return false;
  

  while(1)
    {
      double x,y,z;
      ivstream >> x;
      if (ivstream.fail())
	{
	  ivstream.clear();
	  break;
	}
      ivstream >> y;
      ivstream >> z;
      Mesh::VertexHandle vh = m->add_vertex(Mesh::Point(x,y,z)); 
      vHandleList.push_back(vh);
    }
  
  do{
    ivstream >> current;
  }while(current.compare("coordIndex"));
  
  ivstream >> current;
  if(current.compare("["))
    return false;
  
  while(1)
    {
      std::vector<Mesh::VertexHandle> face_vhandles;
      unsigned int x;
      face_vhandles.clear();
      for( ivstream >> x; x!=-1 && (!ivstream.fail()); ivstream >>x)
	{  
	  face_vhandles.push_back(vHandleList[x]);
	}
      if(!ivstream.fail())
	m->add_face(face_vhandles);
      else
	break;
    }
  return true;
}

