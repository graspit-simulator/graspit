#include "Views.h"
#include "Camera_Frustrum.h"
#include <iostream>
#include <time.h>
#include <fstream>
#include <cmath>
#include <highgui.h>


namespace cu
{
	namespace robotics
	{
		namespace render
		{
			//------------------------------------------------------------
			Views::Views()
				: mImage(NULL), 
				mNoise(NULL),
				mRenderer(NULL)
			{
				// Setup the identity matrix
				memset(mIdentity, 0, 16*sizeof(GLdouble));
				mIdentity[0] = mIdentity[5] = mIdentity[10] = mIdentity[15] = 1.0;

				mRNG_state = cvRNG((int64)time(NULL));
			}

			//------------------------------------------------------------
			Views::~Views()
			{
				if (mRenderer != NULL)
				{
					delete mRenderer;
				}

				if (mImage != NULL)
				{
					cvReleaseImage(&mImage);
				}
				if (mNoise != NULL)
				{
					cvReleaseImage(&mNoise);
				}
			}

			//------------------------------------------------------------
			void Views::InitGL(const int width, const int height)
			{
				// Setup the renderer
				QGLFormat gl_format;
				gl_format.setDoubleBuffer(false);
				gl_format.setDepth(false);
				gl_format.setRedBufferSize(8);
				gl_format.setGreenBufferSize(8);
				gl_format.setBlueBufferSize(8);

				if (mRenderer != NULL)
				{
					delete mRenderer;
					mRenderer = NULL;
				}
				mRenderer = new QGLPixelBuffer(width, height, gl_format);
				mRenderer->makeCurrent();

				// Initialize OpenGL scene
				glShadeModel(GL_SMOOTH);
				glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
				glClearDepth(1.0f);
				glEnable(GL_DEPTH_TEST);
				glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

				glEnable(GL_NORMALIZE);


				glEnable(GL_NORMALIZE);
				//If we want to see only the front side
				//glEnable(GL_CULL_FACE);
				//If we want to see both sides
				glDisable(GL_CULL_FACE);
				glDepthFunc(GL_LEQUAL);

				glViewport(0, 0, width, height);
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				camera_frustrum::SetFrustumFromCamera();
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
			}

			//------------------------------------------------------------
			bool Views::RenderViews(const Mesh& object_mesh, const std::string& out_directory, double model_rescale_factor,
				double camX, double camY, double camZ, double cogX, double cogY, double cogZ, double upX, double upY, double upZ,
				std::vector<double>& depthVec)
			{
				int s = 0;

				if (!QGLPixelBuffer::hasOpenGLPbuffers()) 
				{
					std::cerr << "Error: No Open GL pbuffer support.\n" << std::endl;
					return false;
				}

				// Store desired image dimensions in int format
				const int img_width  = static_cast<int>(utilities::ImageWidth);
				const int img_height = static_cast<int>(utilities::ImageHeight);

				// Initialize the OpenGL scene
				InitGL(img_width, img_height);

				// Allocate the render image, if necessary
				if (mImage == NULL)
				{
					mImage = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_8U, 3);
				}

				// Allocate the noise image, if necessary
				if (mNoise == NULL)
				{
					mNoise = cvCreateImage(cvGetSize(mImage), IPL_DEPTH_32F, mImage->nChannels);
				}

				// Get the look-at point
				//
				// :TODO: Right now this chooses the (approximate) center-of-mass of the
				//        object to look-at.  Consider changing this (randomly and slightly),
				//        for each successive view, to add some changing perspective to the views.
				//
				//        const CvPoint3D64f lookAt_pt = LookAtPoint(object_mesh);
				//        std::cout << "Look-at point: [" << lookAt_pt.x << ", " << lookAt_pt.y << ", " << lookAt_pt.z << "]\n" << std::endl;

				// The eye position
				float eye[3];

				char fileName[200];
				int numSamples;

				double scale;

				int i = 0;

				// Get the eye position
				eye[0] = camX;
				eye[1] = camY;
				eye[2] = camZ;

				mCam_x = camX;
				mCam_y = camY;
				mCam_z = camZ;

				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				// Reset modelview transformation
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();

				gluLookAt(eye[0], eye[1], eye[2], cogX, cogY, cogZ, upX, upY, upZ);

				// Render the object in world space
				RenderObject(object_mesh, eye);

				// Retrieve the viewport, modelview, and projection matrices
				glGetDoublev (GL_MODELVIEW_MATRIX,  mModelview);		  
				glGetDoublev (GL_PROJECTION_MATRIX, mProjection);
				glGetIntegerv(GL_VIEWPORT,          mViewport);

				// Save the image view
				SaveImageView(out_directory, s);

				// Save the modelview matrix
				//SaveModelView(out_directory, s);

				////// Save the 2D-to-3D mappings file
				SaveMapping(out_directory, s);

				SaveDepth(depthVec);

				//SaveDepthImage(out_directory, s);

				std::cout << std::endl;

				// Disable lighting
				glDisable(GL_BLEND);
				glDisable(GL_CULL_FACE);
				glDisable(GL_NORMALIZE);
				glDisable(GL_DEPTH_TEST);

				// Return success
				return true;
			}

			//------------------------------------------------------------
			void Views::RenderObject(const Mesh& object_mesh, const float eye[3])
			{
				// First let's draw the 3-D scene.  Then we'll iterate over it to render 2-D views.
				//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				//glColor3f(0.5f, 1.0f, 0.5f);
				glBegin(GL_TRIANGLES);

				// Iterate over all the faces in the mesh
				for (Mesh::ConstFaceIter face = object_mesh.faces_begin(); face != object_mesh.faces_end(); ++face)
				{
					// Iterate over all vertices
					// ConstFaceVertexIter overloads operator bool, so the termination condition
					// is that the iterator evaluates as a bool to false.
					int c = 0;
					Mesh::Point v[3];
					for (Mesh::ConstFaceVertexIter vertex = object_mesh.cfv_iter(face); vertex; ++vertex, ++c)
					{
						v[c] = object_mesh.point(vertex);
					}

					// Set the normal vector--this is important to allow the lighting to make the object look "real"
					Mesh::Point normVec;
					ComputeNormal(v, normVec);

					Mesh::Point center;
					center[0] = (v[0][0] + v[1][0] + v[2][0]) / 3.0;
					center[1] = (v[0][1] + v[1][1] + v[2][1]) / 3.0;
					center[2] = (v[0][2] + v[1][2] + v[2][2]) / 3.0;

					Mesh::Point dir;
					dir[0] = center[0] - eye[0];
					dir[1] = center[1] - eye[1];
					dir[2] = center[2] - eye[2];
					float dirLen = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
					dir[0] /= dirLen;
					dir[1] /= dirLen;
					dir[2] /= dirLen;

					// Compute dot-product with eye vector
					const float diffuse = abs(normVec[0]*dir[0] + normVec[1]*dir[1] + normVec[2]*dir[2]);
					glColor3f(diffuse*1.0f, diffuse*1.0f, diffuse*1.0f);

					// Draw the triangle
					glVertex3f(v[0][0], v[0][1], v[0][2]);
					glVertex3f(v[1][0], v[1][1], v[1][2]);
					glVertex3f(v[2][0], v[2][1], v[2][2]);

					//std::cout << v[0][0] << " " << v[0][1] << " " << v[0][2] << "\n" <<
					//	v[1][0] << " " << v[1][1] << " " << v[1][2] << "\n" <<
					//	v[2][0] << " " << v[2][1] << " " << v[2][2] << "\n";
				}

				glEnd();
			}

			//------------------------------------------------------------
			void Views::ComputeNormal(const Mesh::Point v[3], Mesh::Point& n)
			{
				Mesh::Point a, b;

				// calculate the vectors A and B
				// note that v[3] is defined with counterclockwise winding in mind
				// a
				a[0] = v[0][0] - v[1][0];
				a[1] = v[0][1] - v[1][1];
				a[2] = v[0][2] - v[1][2];
				// b
				b[0] = v[1][0] - v[2][0];
				b[1] = v[1][1] - v[2][1];
				b[2] = v[1][2] - v[2][2];

				// calculate the cross product and place the resulting vector
				// into the address specified by vertex_t *normal
				n[0] = (a[1] * b[2]) - (a[2] * b[1]);
				n[1] = (a[2] * b[0]) - (a[0] * b[2]);
				n[2] = (a[0] * b[1]) - (a[1] * b[0]);

				// normalize
				NormalizeVector(n);
			}

			//------------------------------------------------------------
			void Views::NormalizeVector(Mesh::Point& p)
			{
				// calculate the length of the vector
				float len = static_cast<float>(sqrt( (p[0] * p[0]) + (p[1] * p[1]) + (p[2] * p[2]) ));

				// avoid division by 0
				if (len == 0.0f)
				{
					len = 1.0f;
				}

				// reduce to unit size
				p[0] /= len;
				p[1] /= len;
				p[2] /= len;
			}

			//------------------------------------------------------------
			CvPoint3D64f Views::LookAtPoint(const Mesh& object_mesh)
			{
				CvPoint3D64f look;
				look.x = 0.0;
				look.y = 0.0;
				look.z = 0.0;

				int num_pts = 0;

				std::vector<double> xs, ys, zs;

				// Iterate over all the faces in the mesh
				for (Mesh::ConstFaceIter face = object_mesh.faces_begin(); face != object_mesh.faces_end(); ++face)
				{
					// Iterate over all vertices
					// ConstFaceVertexIter overloads operator bool, so the termination condition
					// is that the iterator evaluates as a bool to false.
					for (Mesh::ConstFaceVertexIter vertex = object_mesh.cfv_iter(face); vertex; ++vertex)
					{
						const Mesh::Point p = object_mesh.point(vertex);

						look.x += p[0];
						look.y += p[1];
						look.z += p[2];
						++num_pts;

						xs.push_back(p[0]);
						ys.push_back(p[1]);
						zs.push_back(p[2]);
					}
				}

				std::sort(xs.begin(), xs.end());
				std::sort(ys.begin(), ys.end());
				std::sort(zs.begin(), zs.end());

#if 0
				look.x = xs[num_pts/2];
				look.y = ys[num_pts/2];
				look.z = zs[num_pts/2];
#else
				look.x /= (double)num_pts;
				look.y /= (double)num_pts;
				look.z /= (double)num_pts;
#endif

				return look;
			}

			//------------------------------------------------------------
			void Views::AddNoiseToImage(CvRNG& rng, IplImage* noise, IplImage* image)
			{
#ifdef ADD_NOSE_ENABLED
				// Initialize gaussian noise image (mean=0, stdev=1)
				cvSetZero(noise);
				const double mean  = 0.0;
				const double stdev = 10.0;  //= 8.0;
				cvRandArr(&rng, noise, CV_RAND_NORMAL, cvScalarAll(mean), cvScalarAll(stdev));

				// Now add to the gray-scale image, and clip b/w 0 and 255
				CvScalar pi, pn;
				for (int y = 0; y < image->height; ++y)
				{
					for (int x = 0; x < image->width; ++x)
					{
						// Get pixel values from image and noise generator
						pi = cvGet2D(image, y, x);
						pn = cvGet2D(noise, y, x);

						// Only add noise to pixels with data in them
						if ((pi.val[0] == 0) && (pi.val[1] == 0) && (pi.val[2] == 0))
						{
							continue;
						}

						// Add noise
						pi.val[0] += pn.val[0];  // b
						pi.val[1] += pn.val[1];  // g
						pi.val[2] += pn.val[2];  // r

						// Clip b/w 0 and 255
						pi.val[0] = std::max<double>(pi.val[0], 0.0);  // b
						pi.val[0] = std::min<double>(pi.val[0], 255.0);
						pi.val[1] = std::max<double>(pi.val[1], 0.0);  // g
						pi.val[1] = std::min<double>(pi.val[1], 255.0);
						pi.val[2] = std::max<double>(pi.val[2], 0.0);  // r
						pi.val[2] = std::min<double>(pi.val[2], 255.0);

						// Update image
						cvSet2D(image, y, x, pi);
					}
				}

				// Finally, blur in the noise
				cvSmooth(image, image);
#endif
			}

			//------------------------------------------------------------
			void Views::SaveImageView(const std::string& out_directory, const int index)
			{
				// Set the filename for the rendered view image, grab the image, and save it to file.
				memset(mImageViewFileName, 0, 1024);
				sprintf(mImageViewFileName, "%sview_%d.png", out_directory.c_str(), index);

				cvSetZero(mImage);
				//				int k = 0;
				for (int y = 0; y < mImage->height; ++y)
				{
					//					for (int x = 0; x < mImage->width; ++x)
					{
						glReadPixels(0, mImage->height - y - 1, mImage->width, 1, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)( (char*)(mImage->imageData) + 3 * y * mImage->width) );
						//						k ++;
					}
				}
				//glReadPixels(0,0, mImage->width, mImage->height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)mImage->imageData );
				//AddNoiseToImage(mRNG_state, mNoise, mImage);

				cvSaveImage(mImageViewFileName, mImage);
				std::cout << "Successfully saved view: " << mImageViewFileName << std::endl;
			}

			//------------------------------------------------------------
			void Views::SaveModelView(const std::string& out_directory, const int index)
			{
				// Write out the modelview matrix to file
				//
				// Here's the format (16-element arrays as follows):
				//
				//	[ 0  4  8 12;
				//    1  5  9 13;
				//    2  6 10 14;
				//    3  7 11 15];
				//
				// To keep this in "normal array format", I will write out as: mat[0] mat[4] mat[8] mat[12] mat[1] mat[5] ...
				//
				memset(mModelViewFileName, 0, 1024);
				sprintf(mModelViewFileName, "%smodelview_%d.txt", out_directory.c_str(), index);

				std::ofstream mv_ofs(mModelViewFileName);
				mv_ofs << 
					mModelview[0] << " " << mModelview[4] << " " << mModelview[8]  << " " << mModelview[12] << " " <<
					mModelview[1] << " " << mModelview[5] << " " << mModelview[9]  << " " << mModelview[13] << " " <<
					mModelview[2] << " " << mModelview[6] << " " << mModelview[10] << " " << mModelview[14] << " " <<
					mModelview[3] << " " << mModelview[7] << " " << mModelview[11] << " " << mModelview[15] << std::endl;

				mv_ofs.close();
				std::cout << "Successfully saved modelview matrix: " << mModelViewFileName << std::endl;
			}

			//------------------------------------------------------------
			void Views::SaveDepth(std::vector<double> & depthVec)
			{
				float depth;
				CvScalar pixel;

				for (int y = 0; y < mImage->height; ++y)
				{
					for (int x = 0; x < mImage->width; ++x)
					{
						// Skip this pixel if there's no data in it
						pixel = cvGet2D(mImage, y, x);
						if ((pixel.val[0] > 0.0) || (pixel.val[1] > 0.0) || (pixel.val[2] > 0.0))
						{
							// Get 3-D window components
							mWindow[0] = static_cast<double>(x);
							mWindow[1] = static_cast<double>(mViewport[3]) - static_cast<double>(y);// - 1.0;
							glReadPixels(static_cast<int>(mWindow[0]), static_cast<int>(mWindow[1]), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
							//throw away nonsense
							if(fabs(depth - 1.000) < 0.000000001)
							{
								depthVec.push_back(-1);
								continue;
							}
							mWindow[2] = depth;
							if (glGetError() == GL_INVALID_OPERATION)
							{
								std::cerr << "\nGOT ERROR!\n" << std::endl;
								continue;
							}

							// Get the 3D coordinate associated with this 2D pixel
							gluUnProject(mWindow[0], mWindow[1], mWindow[2], mModelview, mProjection, mViewport, &mObject[0], &mObject[1], &mObject[2]);

							double d = sqrt( (mObject[0] - mCam_x) * (mObject[0] - mCam_x) +
								(mObject[1] - mCam_y) * (mObject[1] - mCam_y) + 
								(mObject[2] - mCam_z) * (mObject[2] - mCam_z) );
							depthVec.push_back(d);
						}
						else
							depthVec.push_back(-1);
					}
				}
			}

			void Views::SaveMapping(const std::string& out_directory, const int index)
			{

				// Write out 2D-to-3D mappings of this view
				memset(mMappingFileName, 0, 1024);
				sprintf(mMappingFileName, "%s2D_To_3D_Map_%d.txt", out_directory.c_str(), index);
				std::ofstream map_ofs(mMappingFileName);

				// Write out depth of this view
				memset(mDepthFileName, 0, 1024);
				sprintf(mDepthFileName, "%sdepth_%d.txt", out_directory.c_str(), index);
				std::ofstream depth_ofs(mDepthFileName);

				float depth;
				CvScalar pixel;

				for (int y = 0; y < mImage->height; ++y)
				{
					for (int x = 0; x < mImage->width; ++x)
					{
						// Skip this pixel if there's no data in it
						pixel = cvGet2D(mImage, y, x);
						if ((pixel.val[0] > 0.0) || (pixel.val[1] > 0.0) || (pixel.val[2] > 0.0))
						{
							// Get 3-D window components
							mWindow[0] = static_cast<double>(x);
							mWindow[1] = static_cast<double>(mViewport[3]) - static_cast<double>(y);// - 1.0;
							glReadPixels(static_cast<int>(mWindow[0]), static_cast<int>(mWindow[1]), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
							//throw away nonsense
							if(fabs(depth - 1.000) < 0.000000001)
							{
								depth_ofs << "-1\n";
								continue;
							}
							//printf("%f\n", depth);
							mWindow[2] = depth;
							if (glGetError() == GL_INVALID_OPERATION)
							{
								std::cerr << "\nGOT ERROR!\n" << std::endl;
								continue;
							}

							// Get the 3D coordinate associated with this 2D pixel
							gluUnProject(mWindow[0], mWindow[1], mWindow[2], mModelview, mProjection, mViewport, &mObject[0], &mObject[1], &mObject[2]);
							map_ofs << mObject[0] << "," << mObject[1] << "," << mObject[2] << "," << static_cast<int>(mWindow[0]) << 
								"," << static_cast<int>(mWindow[1]) << std::endl;

							double d = sqrt( (mObject[0] - mCam_x) * (mObject[0] - mCam_x) +
								(mObject[1] - mCam_y) * (mObject[1] - mCam_y) + 
								(mObject[2] - mCam_z) * (mObject[2] - mCam_z) );
							depth_ofs << d << "\n";
						}
						else
							depth_ofs << "-1\n";
					}
				}
				map_ofs.close();
				std::cout << "Successfully saved 2D-to-3D map: " << mMappingFileName << std::endl;
			}
		}
	}
}



