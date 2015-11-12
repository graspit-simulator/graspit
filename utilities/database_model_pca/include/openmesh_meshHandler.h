#include "meshHandler.h"
#include "openmesh.h"


class openmesh_meshHandler:public meshHandler
{
 public:
    virtual bool readMesh(const std::string & filename);
    openmesh_meshHandler(const std::string & filename);
	openmesh_meshHandler(){};
    //output mean of the mesh
    virtual void mean(float mn[3]);
    //output the covatiance matrix of the mesh as a row major column
    virtual void covarianceMatrix(float *cov, float *m= NULL);
    //generate a new mesh that is standardized.
    virtual void zScores(meshHandler *child, float * m = NULL, float * cov = NULL);
    //add a point to the mesh.  return 0 if failed.
    virtual bool addPoint(float x, float y, float z);
  private:
    Mesh mesh;
};
