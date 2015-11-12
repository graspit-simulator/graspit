#ifndef CU_ROBOTICS_RENDER_VIEWS_H
#define CU_ROBOTICS_RENDER_VIEWS_H

#include <string>
#include "Utilities.h"
#include <cv.h>
#include <QtOpenGL>

using cu::robotics::utilities::Mesh;

namespace cu
{
  namespace robotics
  {
    namespace render
    {
      class Views
      {
      public:
        Views();
        ~Views();

        bool RenderViews(const Mesh& object_mesh, const std::string& out_directory, double model_rescale_factor,
			double camX, double camY, double camZ, double cogX, double cogY, double cogZ, double upX, double upY, double upZ,
			std::vector<double>& depthVec);

      protected:
        void InitGL(const int width, const int height);
        void RenderObject(const Mesh& object_mesh, const float eye[3]);
        void ComputeNormal(const Mesh::Point v[3], Mesh::Point& n);
        void NormalizeVector(Mesh::Point& p);
        CvPoint3D64f LookAtPoint(const Mesh& object_mesh);
        void AddNoiseToImage(CvRNG& rng, IplImage* noise, IplImage* image);

        void SaveImageView(const std::string& out_directory, const int index);
        void SaveModelView(const std::string& out_directory, const int index);
        void SaveMapping(const std::string& out_directory, const int index);
		void SaveDepthImage(const std::string& out_directory, const int index);
		void SaveDepth(std::vector<double>&depthVec);

      private:
        // The renderer
        QGLPixelBuffer* mRenderer;

        // The render image
        IplImage* mImage;

        // Filenames
        char mImageViewFileName[1024];
        char mModelViewFileName[1024];
        char mMappingFileName[1024];
		char mDepthFileName[1024];

        // The viewport, modelview matrix, projection matrix, and a utility identity matrix,
        // respectively.
        GLint    mViewport[4];
        GLdouble mModelview[16];
        GLdouble mProjection[16];
        GLdouble mIdentity[16];

        // Lighting setup
        GLfloat mAmbientLight[4];
        GLfloat mDiffuseLight[4];
        GLfloat mSpecularLight[4];
        GLfloat mLightPosition0[4];
        GLfloat mLightDirection0[3];

		double mCam_x, mCam_y, mCam_z;

        // For creating 2D-to-3D unprojection mappings
        GLdouble mObject[3];
        GLdouble mWindow[3];

        // For adding random noise to rendered image
        CvRNG mRNG_state;
        IplImage* mNoise;
      };
    }
  }
}

#endif
