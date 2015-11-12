#include "openmesh.h"
//#include <OpenMesh/Core/System/omstream.hh>
#include <iostream>

void ReadMesh(const std::string& mesh_file, Mesh* mesh) {
  DisableOpenMeshLogs();
  std::cout << "Reading Mesh from: \"" << mesh_file << "\"\n";
  if (!OpenMesh::IO::read_mesh(*mesh, mesh_file.c_str())) {
    std::cerr << "Error: Cannot read mesh from " << mesh_file << std::endl;
    exit(1);
  }
}

void WriteMesh(const std::string& mesh_file, const Mesh& mesh) {
  DisableOpenMeshLogs();
  std::cout << "Writing Mesh to: \"" << mesh_file << "\"\n";
  if (!OpenMesh::IO::write_mesh(mesh, mesh_file.c_str())) {
    std::cerr << "Error: Cannot write mesh to " << mesh_file << std::endl;
    exit(1);
  }
}

void DisableOpenMeshLogs() {
  omlog().disable();
  omout().disable();
  omerr().disable();
}


