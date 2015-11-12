#include "scan_manager.h"

#ifdef SEMANTIC_PLANNER_ENABLED
#include <direct.h>
#include "../SemanticPlanner/Utilities.h"
#include "../SemanticPlanner/Views.h"
#endif

void ScanManager::scan1()
{
	mCloud.clear();
	mRawData.clear();
	sim.scan(&mCloud, &mRawData);
}

void ScanManager::scanToFile(std::string rawFilePath, std::string modelFilePath)
{
	scan1();

	FILE *fp_m, *fp_r;
	fp_m = fopen(modelFilePath.c_str(),"w");
	fp_r = fopen(rawFilePath.c_str(),"w");
	for (int k=0; k<(int)mCloud.size(); k++) {
		fprintf(fp_m,"%f %f %f\n",mCloud[k].x(), mCloud[k].y(), mCloud[k].z());
	}
	fclose(fp_m);

	for (int k=0; k<(int)mRawData.size(); k++) {
		fprintf(fp_r,"%f %f %f %f %f %f\n",mRawData[k].hAngle, mRawData[k].vAngle, mRawData[k].dx, mRawData[k].dy, mRawData[k].dz, mRawData[k].distance);
	}
	fclose(fp_r);
}

void ScanManager::setupCameraPose(vec3 location, vec3 dir, vec3 up, vec3 at)
{
	mLocation = location;
	mDir = dir;
	mUp = up;
	mAt = at;
	sim.setPosition(location,dir,up);

}

void ScanManager::saveParamsToFile(FILE *fp)
{
	fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", mLocation[0], mLocation[1], mLocation[2],
		mDir[0], mDir[1], mDir[2],
		mUp[0], mUp[1], mUp[2]);
}

int ScanManager::scan2(std::string model_file_path, std::string output_directory_path, double model_rescale_factor)
{
#ifdef SEMANTIC_PLANNER_ENABLED
  cu::robotics::utilities::DisableOpenMeshLogs();

  // Setup manual inputs
  cu::robotics::utilities::Mesh mesh, views;
  const std::string model_mesh_file = model_file_path;

  // Read in the model mesh.
  if (!OpenMesh::IO::read_mesh(mesh, model_mesh_file)) 
  {
    std::cerr << "Error: Cannot read mesh from " << model_mesh_file << "\n" << std::endl;
    return 1;
  }
  else
  {
    std::cout << "Successfully loaded mesh file: " << model_mesh_file << "\n" << std::endl;
  }

  // rescale
  for (OpenMesh::TriMesh_ArrayKernelT<>::VertexIter v_it = mesh.vertices_begin();
       v_it != mesh.vertices_end(); ++v_it)
  {
    //std::cout << "Vertex #" << v_it << ": " << mesh.point( v_it );
	  OpenMesh::DefaultTraits::Point p = mesh.point(v_it) * model_rescale_factor;
	  p[2] = p[2];
	  mesh.set_point( v_it, p);
    //std::cout << " moved to " << mesh.point( v_it ) << std::endl;
  }

  mDepth.clear();
  cu::robotics::render::Views v;
  if (!v.RenderViews(mesh, output_directory_path, model_rescale_factor, mLocation.x(), mLocation.y(), mLocation.z(),
	  mAt.x(), mAt.y(), mAt.z(), mUp.x(), mUp.y(), mUp.z(), mDepth))
  {
    std::cerr << "Error: Could not create views of object mesh from views mesh." << "\n" << std::endl;
    return 1;
  }
  else
  {
    std::cout << "Successfully created views of object mesh from views mesh." << "\n" << std::endl;
  }
#endif

  // Exit application
  return 0;
}

bool ScanManager::checkWithin(vec3 origin, vec3 corner, double vertical, double horizontal, vec3 up, vec3 approach)
{
	//std::cout << "corner is: " << corner.x() << " " << corner.y() << " " << corner.z() << std::endl;
	vec3 origin2Corner = (corner - origin);
	origin2Corner = origin2Corner/origin2Corner.len();

	double approachProj = approach % origin2Corner;
	double upProj = up % origin2Corner;
	vec3 horizon = up * approach;
	double horizonProj = horizon % origin2Corner;
	
	if(approachProj < 0)
		return false;

	double v = atan(fabs(upProj/approachProj));
	if(v > vertical * 3.14159265 / 180.0)
		return false;

	double h = atan(fabs(horizonProj/approachProj));
	if(h > horizontal * 3.14159265 / 180.0)
		return false;
	return true;
}
