#ifndef CU_ROBOTICS_CAMERA_FRUSTRUM_H
#define CU_ROBOTICS_CAMERA_FRUSTRUM_H

#include <QtOpenGL>
#include <cv.h>
#include "Utilities.h"


namespace cu
{
  namespace robotics
  {
    namespace camera_frustrum
    {
      static CvPoint3D64f CameraPixelRay(double u, double v) 
      {
        return cvPoint3D64f((u - utilities::PrincipalPointX) / utilities::FocalLengthX, 
          (v - utilities::PrincipalPointY) / utilities::FocalLengthY, 
          1.0);
      }

      // The glFrustum function describes a perspective matrix that produces a
      // perspective projection. The (left, bottom, znear) and (right, top, znear)
      // parameters specify the points on the near clipping plane that are mapped to
      // the lower-left and upper-right corners of the window, respectively, assuming
      // that the eye is located at (0,0,0). The zfar parameter specifies the 
      // location of the far clipping plane. 
      static void SetFrustumFromCamera() 
      {
        // Set the near plane (~ 10 cm) and the far plane (~ 1.0-1.25 meters)
        //const double near_plane = 0.1, far_plane = 1.25;
        const double near_plane = 30.0, far_plane = 1500.0;

        // Get the corners of the image in the near plane
        CvPoint3D64f top_left = CameraPixelRay(-0.5, -0.5),
          bottom_right = CameraPixelRay(utilities::ImageWidth-0.5, utilities::ImageHeight-0.5);

        const double left = top_left.x * near_plane,
          right = bottom_right.x * near_plane,
          bottom = bottom_right.y * near_plane,
          top = top_left.y * near_plane;

        // Switch top and bottom because OpenGL's y-axis starts from the bottom.
        glFrustum(left, right, top, bottom, near_plane, far_plane);
      }
    }
  }
}
#endif
