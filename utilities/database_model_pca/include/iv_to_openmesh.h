#include "openmesh.h"
#include <vector>
#include "matvec3D.h"
#include <fstream>


class ivMeshifier{
 public:
  ivMeshifier();
  bool meshify(std::ifstream &ivstream, Mesh * m);
 private:
  std::vector<Mesh::VertexHandle> vHandleList;
  Mesh::VertexHandle vh; 
};
