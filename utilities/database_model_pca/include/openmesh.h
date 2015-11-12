#ifndef OPENMESH_H_
#define OPENMESH_H_

#define _USE_MATH_DEFINES

#pragma warning(disable:4100)
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Types/TriMesh_ArrayKernelT.hh>

#pragma warning(default:4100)

typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;
typedef Mesh PointCloud;

void ReadMesh(const std::string& mesh_file, Mesh* mesh);
void WriteMesh(const std::string& mesh_file, const Mesh& mesh);
void DisableOpenMeshLogs();

#endif  // OPENMESH_H_
