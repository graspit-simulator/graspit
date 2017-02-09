#include <gtest/gtest.h>

#include "transform.h"

#define SLOP 0.001


TEST(TEST_QUATERNION, TEST_QUATERNION_TO_ROTATION_MATRIX_AND_BACK) {

  Eigen::AngleAxisd aa = Eigen::AngleAxisd(10, vec3(1,1,1).normalized());
  Quaternion q(aa);

  mat3 R = q.toRotationMatrix();

  //THIS WAS A BUG PRIOR TO EIGEN
  //THESE WERE NOT EQUAL, THEY ARE NOW
  vec3 RdotZ = R * vec3(0,0,1);
  vec3 qdotZ = q * vec3(0,0,1);

  ASSERT_NEAR(RdotZ.x(), qdotZ.x(), SLOP);
  ASSERT_NEAR(RdotZ.y(), qdotZ.y(), SLOP);
  ASSERT_NEAR(RdotZ.z(), qdotZ.z(), SLOP);

  Quaternion q2 = Quaternion(R);

  //THIS ALWAYS WORKED, EVEN PRIOR TO EIGEN
  ASSERT_NEAR(q.w(), -q2.w(), SLOP);
  ASSERT_NEAR(q.x(), -q2.x(), SLOP);
  ASSERT_NEAR(q.y(), -q2.y(), SLOP);
  ASSERT_NEAR(q.z(), -q2.z(), SLOP);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
