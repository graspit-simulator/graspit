#ifndef MESHHANDLER_H
#define MESHHANDLER_H
#include <string>


//A simple geometry handling base class
class meshHandler{
 public:
    virtual bool readMesh(const std::string & filename) = 0;
	//output mean of the mesh
    virtual void mean(float [3]) = 0;
    //output the covatiance matrix of the mesh as a row major column
    virtual void covarianceMatrix(float cov[9], float m[3] = NULL) = 0;
    //generate a new mesh that is standardized.
    virtual void zScores(meshHandler *child, float m[3] = NULL, float cov[9] = NULL) = 0;
    //add a point to the mesh.  return 0 if failed.
    virtual bool addPoint(float x, float y, float z) = 0;
};


#endif
