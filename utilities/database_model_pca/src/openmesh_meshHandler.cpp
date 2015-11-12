#include "openmesh_meshHandler.h"
#include <fstream>
#include <string.h>
#include "iv_to_openmesh.h"

void openmesh_meshHandler::mean(float mn[3])
{
      mn[0] = 0.0f; mn[1] = 0.0f; mn[2] = 0.0f;
      Mesh::Point mp(0.0,0.0,0.0);
      for (Mesh::ConstVertexIter vertex_iter = mesh.vertices_begin();
	   vertex_iter != mesh.vertices_end(); ++vertex_iter){
	mp+=mesh.point(vertex_iter);
      }
      mp/= mesh.n_vertices()*1.0f;
      mn[0] = mp[0]; mn[1] = mp[1]; mn[2] = mp[2];
}

void openmesh_meshHandler:: covarianceMatrix(float cov[9], float m[3])
{

  memset(cov,0,9*sizeof(float));
  float *mn;
  if (!m){
    mn = new float[3];
    mean(mn);
  }
  else
      mn = &m[0];
  for (Mesh::ConstVertexIter vertex_iter = mesh.vertices_begin();
       vertex_iter != mesh.vertices_end(); ++vertex_iter){
    Mesh::Point p(mesh.point(vertex_iter)[0] - mn[0],mesh.point(vertex_iter)[1] - mn[1], mesh.point(vertex_iter)[2] - mn[2]);
    cov[0]+= p[0] *p[0] ;
    cov[4] += p[1] *p[1] ;
    cov[8] += p[2] *p[2] ;
    cov[1] += p[1] *p[0] ;
    cov[2] += p[2] *p[0] ;
    cov[5] += p[2] *p[1] ;
  }
  cov[0] /= mesh.n_vertices()-1;
  cov[1] /= mesh.n_vertices()-1;
  cov[2] /= mesh.n_vertices()-1;
  cov[3] = cov[1];
  cov[4] /= mesh.n_vertices()-1;
  cov[5] /= mesh.n_vertices()-1;
  cov[6] = cov[2];
  cov[7]=cov[5];
  cov[8] /= mesh.n_vertices()-1;
  
  if(!m)
    delete[] mn;
}

/* return a new pointcloud that is the standard score of 
   each point */
void openmesh_meshHandler::zScores(meshHandler * child, float * m, float * cov){
  float * mn, *covar;
  if (!m){
    mn = new float[3];
    mean(mn);
  }
  else{
    mn = &m[0];
  } 
    if (!cov){
      covar = new float[9];
      covarianceMatrix(covar, mn);
    }
    else{
      covar = &cov[0];
      
 }    
  float s1 = sqrt(covar[0]);
  float s2 = sqrt(covar[4]);
  float s3 = sqrt(covar[8]);
  for (Mesh::ConstVertexIter vertex_iter = mesh.vertices_begin();
       vertex_iter != mesh.vertices_end(); ++vertex_iter){
    Mesh::Point p(mesh.point(vertex_iter)[0] - mn[0],mesh.point(vertex_iter)[1] - mn[1],mesh.point(vertex_iter)[2] - mn[2]);
    child->addPoint(p[0]/s1, p[1]/s2, p[2]/s3 );
  }

  if(!m)
    delete[] mn;
  if (!cov)
    delete[] covar;
}


bool openmesh_meshHandler::addPoint(float x, float y, float z){
  mesh.add_vertex(Mesh::Point(x,y,z));
  return true;
}

bool openmesh_meshHandler::readMesh(const std::string & filename){
	//check if file exists
	std::ifstream fs(filename.c_str());
	if(!fs)
		return false;
	int period_index = filename.find_last_of('.');
	std::string s = filename.substr(period_index+1,3);
	if(s.compare("iv") && s.compare("wrl")){
	  ReadMesh(filename, &mesh);
	}
	else{
	  //if the file is an iv or wrl file, call this
	  //meshifier.  Currently, it is very lame.
	  ivMeshifier ivm;
	  ivm.meshify(fs, &mesh);
	}
	  fs.close();	
	  return mesh.n_vertices();
}
openmesh_meshHandler::openmesh_meshHandler(const std::string & filename){
	readMesh(filename);
}
