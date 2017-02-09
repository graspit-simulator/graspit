#include <gtest/gtest.h>
#include <iostream>
#include <QString>
#include <pthread.h>
#include <QMutex>
#include <Inventor/Qt/SoQt.h>

#include "graspitCore.h"
#include <graspitApp.h>
#include "mainWindow.h"
#include "ivmgr.h"
#include "world.h"

#define SLOP 0.001

TEST(TEST_TRANSFORMS, TEST_TRANSFORM_APPLIED_TO_POSITION_AND_VEC3) {

  vec3 x = vec3(1,0,0);
  transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
  transf translate = transf::TRANSLATION(vec3(1,2,3));
  position point = position(0.1, 0.2, 0.3);
  vec3 norm = vec3(0.1, 0.2, 0.3).normalized();

  norm = (rot % translate).affine() * (norm);
  point = (rot % translate) * (point);

  vec3 xcrossnorm = x.cross(norm);
  double xdotnorm = x.dot(norm);

  ASSERT_NEAR(norm.x(), 0.267261, SLOP);
  ASSERT_NEAR(norm.y(), -0.0123153, SLOP);
  ASSERT_NEAR(norm.z(), -0.963545, SLOP);

  ASSERT_NEAR(point.x(), 1.1, SLOP);
  ASSERT_NEAR(point.y(), -0.0506877, SLOP);
  ASSERT_NEAR(point.z(), -3.96578, SLOP);

  ASSERT_NEAR(xcrossnorm.x(), 0.0, SLOP);
  ASSERT_NEAR(xcrossnorm.y(), 0.963545, SLOP);
  ASSERT_NEAR(xcrossnorm.z(), -0.0123153, SLOP);

  ASSERT_NEAR(xdotnorm, 0.267261, SLOP);

}

TEST(TEST_TRANSFORMS, TEST_TRANSFORM_ORDER_OF_OPERATIONS) {
    /*
     * This appears very simple, but I wanted to demonstrate specifically how
     * Transform operations work in GraspIt! expecially after switching to Eigen
     * which caused the order in which transforms are applied to be reversed.
     *
     * They now works following:
     * peter's notes pg 5
     * http://www.cs.columbia.edu/~allen/F15/NOTES/forwardspong.pdf
     */


  double d01 = 0.1;
  double d12 = 0.2;
  double d23 = 0.3;

  transf A_01 = transf(mat3::Identity(), vec3(0,0,d01));
  mat3 r_12;
  r_12.col(0) = vec3(1,0,0);
  r_12.col(1) = vec3(0,0,-1);
  r_12.col(2) = vec3(0,1,0);

  transf A_12 = transf(r_12, vec3(0,0,d12));
  transf A_23 = transf(mat3::Identity(), vec3(0,0,d23));

  transf A_03 = A_01 % (A_12 % A_23);

  ASSERT_NEAR(A_03.translation().x(), 0.0, SLOP);
  ASSERT_NEAR(A_03.translation().y(), 0.3, SLOP);
  ASSERT_NEAR(A_03.translation().z(), 0.3, SLOP);

  ASSERT_NEAR(A_03.rotation().w(), 0.707107, SLOP);
  ASSERT_NEAR(A_03.rotation().x(), -0.707107, SLOP);
  ASSERT_NEAR(A_03.rotation().y(), 0.0, SLOP);
  ASSERT_NEAR(A_03.rotation().z(), 0.0, SLOP);

  position p_0 = position (0.1, 0.2, 0.3);
  position p_3 = A_03 * (p_0);

  ASSERT_NEAR(p_3.x(), 0.1, SLOP);
  ASSERT_NEAR(p_3.y(), 0.6, SLOP);
  ASSERT_NEAR(p_3.z(), 0.1, SLOP);

}


TEST(TEST_TRANSFORMS, TEST_JACOBIAN) {
/*
 * Ensuring that the Jacobian returned is the same
 * as prior to switching to Eigen, from GraspIt!'s old
 * custom math library.
 */

  vec3 x = vec3(1,0,0);
  transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
  transf translate = transf::TRANSLATION(vec3(1,2,3));

  transf in = rot % translate;

  double M[36];
  in.jacobian(M);

  //UNCOMMENT THIS IF YOU NEED TO UPDATE ASSERT VALUES
//  for(int i =0; i < 36; i++)
//   {
//      std::cout << "ASSERT_NEAR(M[" <<i<<"], " << M[i] << ", SLOP);" << std::endl;
//  }

  ASSERT_NEAR(M[0], 1, SLOP);
  ASSERT_NEAR(M[1], -0, SLOP);
  ASSERT_NEAR(M[2], 0, SLOP);
  ASSERT_NEAR(M[3], 0, SLOP);
  ASSERT_NEAR(M[4], 0, SLOP);
  ASSERT_NEAR(M[5], 0, SLOP);
  ASSERT_NEAR(M[6], 0, SLOP);
  ASSERT_NEAR(M[7], -0.839072, SLOP);
  ASSERT_NEAR(M[8], 0.544021, SLOP);
  ASSERT_NEAR(M[9], 0, SLOP);
  ASSERT_NEAR(M[10], 0, SLOP);
  ASSERT_NEAR(M[11], 0, SLOP);
  ASSERT_NEAR(M[12], -0, SLOP);
  ASSERT_NEAR(M[13], -0.544021, SLOP);
  ASSERT_NEAR(M[14], -0.839072, SLOP);
  ASSERT_NEAR(M[15], 0, SLOP);
  ASSERT_NEAR(M[16], 0, SLOP);
  ASSERT_NEAR(M[17], 0, SLOP);
  ASSERT_NEAR(M[18], 0, SLOP);
  ASSERT_NEAR(M[19], -3, SLOP);
  ASSERT_NEAR(M[20], 2, SLOP);
  ASSERT_NEAR(M[21], 1, SLOP);
  ASSERT_NEAR(M[22], -0, SLOP);
  ASSERT_NEAR(M[23], 0, SLOP);
  ASSERT_NEAR(M[24], -3.60526, SLOP);
  ASSERT_NEAR(M[25], 0.544021, SLOP);
  ASSERT_NEAR(M[26], 0.839072, SLOP);
  ASSERT_NEAR(M[27], 0, SLOP);
  ASSERT_NEAR(M[28], -0.839072, SLOP);
  ASSERT_NEAR(M[29], 0.544021, SLOP);
  ASSERT_NEAR(M[30], 0.0460797, SLOP);
  ASSERT_NEAR(M[31], -0.839072, SLOP);
  ASSERT_NEAR(M[32], 0.544021, SLOP);
  ASSERT_NEAR(M[33], -0, SLOP);
  ASSERT_NEAR(M[34], -0.544021, SLOP);
  ASSERT_NEAR(M[35], -0.839072, SLOP);

}




int main(int argc, char **argv)
{

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
