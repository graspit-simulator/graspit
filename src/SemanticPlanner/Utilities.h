#ifndef CU_ROBOTICS_UTILITIES_H
#define CU_ROBOTICS_UTILITIES_H

#pragma warning(disable:4100)
#include "C:/dev/OpenMesh-2.0-RC5/src/OpenMesh/Core/IO/MeshIO.hh"
#include "C:/dev/OpenMesh-2.0-RC5/src/OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
#include "C:/dev/OpenMesh-2.0-RC5/src/OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh"
#pragma warning(default:4100)

namespace cu
{
  namespace robotics
  {
    namespace utilities
    {
      typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;

      static void DisableOpenMeshLogs() 
      {
        omlog().disable();
        omout().disable();
        omerr().disable();
      }

#if 0
      //
      // Prosilica (left) camera
      //

      // Desired image dimensions
      static const double ImageWidth  = 1620.0;
      static const double ImageHeight = 1220.0;

      // Camera intrinsics to re-create perspective correctly
      static const double FocalLengthX    = 5839.956931;
      static const double FocalLengthY    = 5846.087845;
      static const double PrincipalPointX = 882.409399;
      static const double PrincipalPointY = 590.8345832;
 
      static const int ViewingDistance = 15;   // viewing distance
#else
      //
      // In-Vivo (left) camera
      //

      // Desired image dimensions
	  // Number of pixels
	  // X goes with width
	  // Y goes with height
      static const double ImageWidth  = 144.0;
      static const double ImageHeight = 176.0;
	  static const double FOV_X = 39.6 * M_PI / 180.0;
	  static const double FOV_Y = 47.5 * M_PI / 180.0;

      // Camera intrinsics to re-create perspective correctly
      static const double FocalLengthX    = ImageWidth / (2 * tan(FOV_X / 2));
      static const double FocalLengthY    = ImageHeight / (2 * tan(FOV_Y / 2));
      static const double PrincipalPointX = ImageWidth/2.0;
      static const double PrincipalPointY = ImageHeight/2.0;

      static const int ViewingDistance = 2;   // viewing distance
#endif
    }
  }
}
#endif
