#include <iostream>
#include <string>
#include <direct.h>

#include "Utilities.h"
#include "Views.h"
//
//int renderViews_old(std::string model_file_path, std::string output_directory_path, double model_rescale_factor,
//				double camX, double camY, double camZ, double cogX, double cogY, double cogZ, double upX, double upY, double upZ) 
//{
//  cu::robotics::utilities::DisableOpenMeshLogs();
//
//  // Setup manual inputs
//  cu::robotics::utilities::Mesh mesh, views;
//  const std::string model_mesh_file = model_file_path;
//
//  // Read in the model mesh.
//  if (!OpenMesh::IO::read_mesh(mesh, model_mesh_file)) 
//  {
//    std::cerr << "Error: Cannot read mesh from " << model_mesh_file << "\n" << std::endl;
//    return 1;
//  }
//  else
//  {
//    std::cout << "Successfully loaded mesh file: " << model_mesh_file << "\n" << std::endl;
//  }
//
//  // rescale
//  for (OpenMesh::TriMesh_ArrayKernelT<>::VertexIter v_it = mesh.vertices_begin();
//       v_it != mesh.vertices_end(); ++v_it)
//  {
//    //std::cout << "Vertex #" << v_it << ": " << mesh.point( v_it );
//	  OpenMesh::DefaultTraits::Point p = mesh.point(v_it) * model_rescale_factor;
//	  p[2] = p[2];
//	  mesh.set_point( v_it, p);
//    //std::cout << " moved to " << mesh.point( v_it ) << std::endl;
//  }
//
//  // Construct 2-D views
//  cu::robotics::render::Views v;
//  if (!v.RenderViews(mesh, output_directory_path, camX, camY, camZ, cogX, cogY, cogZ, upX, upY, upZ))
//  {
//    std::cerr << "Error: Could not create views of object mesh from views mesh." << "\n" << std::endl;
//    return 1;
//  }
//  else
//  {
//    std::cout << "Successfully created views of object mesh from views mesh." << "\n" << std::endl;
//  }
//
//  // Exit application
//  return 0;
//}
//
