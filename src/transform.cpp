#include "graspit/transform.h"

const transf transf::IDENTITY(Quaternion::Identity(), vec3::Zero());

/*!
FACTORY CONSTRUCTORS, for creating various transforms
*/

/*!
  creates a transform with an identity rotation and
  a translation equal to vector \a v.
*/
transf
transf::TRANSLATION(const vec3 &v)
{
  return transf(Quaternion::Identity(), v);
}

/*!
  creates a tranform that has a rotation of \a angle
  radians about \a axis, and a zero translation.
*/
transf
transf::AXIS_ANGLE_ROTATION(double angle, const vec3 &axis)
{     
  if(angle*angle < 0.000001)
    {
      return transf::IDENTITY;
    }
  Eigen::AngleAxisd aa = Eigen::AngleAxisd(angle, axis.normalized());
  Quaternion q(aa);
  return transf(q, vec3::Zero());
}

/*!
  creates a tranform from the global frame to the
  new frame with its origin at position \a o, and with the given x and y axes.
*/
transf
transf::COORDINATE(const position &o, const vec3 &xaxis, const vec3 &yaxis)
{
  vec3 newxaxis = xaxis.normalized();
  vec3 newzaxis = (newxaxis.cross(yaxis)).normalized();
  vec3 newyaxis = (newzaxis.cross(newxaxis)).normalized();
  mat3 m;
  m.col(0) = newxaxis;
  m.col(1) = newyaxis;
  m.col(2) = newzaxis;
  return transf(m, o);
}

transf transf::RPY(double rx, double ry, double rz)
{
  transf r;
  vec3 x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);

  r = transf::AXIS_ANGLE_ROTATION(rx, x);
  y =  r.inverse().affine() * y;
  r =  r % transf::AXIS_ANGLE_ROTATION(ry, y);
  z = r.inverse().affine() * z;
  r =  r % transf::AXIS_ANGLE_ROTATION(rz, z);
  return r;
}

/*!
  Returns the inverse transformation.
*/
transf
transf::inverse() const
{
  return transf(rot.inverse(), -(rot.inverse() * t));
}

/*! Assignment operator, copies transform \a tr. */
transf const &
transf::operator=(transf const &tr)
{
  rot = tr.rotation();
  R = tr.affine();
  t = tr.translation();
  return *this;
}

/*! Converts this transform to Inventor format. */
void
transf::toSoTransform(SoTransform *IVt) const
{
  IVt->rotation.setValue(QuaterniontoSbRotation(rot));
  IVt->translation.setValue(toSbVec3f(t));
}
/*! Converts this transform to a column major 4x4 matrix*/
void
transf::toColMajorMatrix(double mat[][4]) const
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat[j][i] = R(j, i);
    }
    mat[i][3] = 0.0;
  }
  mat[3][0] = t[0];
  mat[3][1] = t[1];
  mat[3][2] = t[2];
  mat[3][3] = 1.0;
}

/*! Converts this transform to a row major 4x4 matrix*/
void
transf::toRowMajorMatrix(double mat[][4]) const
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat[i][j] = R(j, i);
    }
    mat[3][i] = 0.0;
  }
  mat[0][3] = t[0];
  mat[1][3] = t[1];
  mat[2][3] = t[2];
  mat[3][3] = 1.0;
}

/*! Converts this transform to the collision system format, a 4x4 matrix. */
void
transf::tocol_Mat4(col_Mat4 colTran) const
{
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      colTran[i][j] = R(j, i);
    }
    colTran[3][j] = 0.0;
  }
  colTran[0][3] = t[0];
  colTran[1][3] = t[1];
  colTran[2][3] = t[2];
  colTran[3][3] = 1.0;
}

/*! Sets the value of this transform by converting an Inventor transform. */
void
transf::set(const SoTransform *IVt)
{
  float qw, qx, qy, qz, x, y, z;

  // Inventor stores the quaternion as qx,qy,qz,qw
  IVt->rotation.getValue().getValue(qx, qy, qz, qw);
  IVt->translation.getValue().getValue(x, y, z);
  rot.w() = (double)qw; rot.x() = (double)qx; rot.y() = (double)qy; rot.z() = (double)qz;
  t[0] = (double)x; t[1] = (double)y; t[2] = (double)z;
  R = rot;
}

/*! Multiplies two transforms.  */
transf
operator%(const transf &tr1, const transf &tr2)
{
  Quaternion newRot = tr1.rotation() * tr2.rotation();
  return transf(newRot, tr1.translation() + tr1.rotation() * tr2.translation());
}

/*!
 * returns the vector under transformation tr
*/
vec3
operator*(const transf &tr1, const vec3 &v)
{
  return tr1.rot * v + tr1.t;
}


/*!
  Compares two transforms.  Returns true if both the rotations and translations
  are equal (according to their comparators).
*/
bool
transf::operator==(transf const &tr) const
{
  if (tr.rot.isApprox(rot) && tr.t == t) {
    return true;
  }
  return false;
}



/*!
  Creates a 6x6 jacobian from a transform.  Code adapted from Peter Corke's
  Robot Toolbox for MATLAB.

 MATLAB representation:
\verbatim
  J = [ t(1:3,1)' cross(t(1:3,4),t(1:3,1))'
      t(1:3,2)' cross(t(1:3,4),t(1:3,2))'
      t(1:3,3)' cross(t(1:3,4),t(1:3,3))'
      zeros(3,3)    t(1:3,1:3)'   ];
\endverbatim

The jacobian is returned is in column-major format.
*/
void
transf::jacobian(double jac[])
{
  mat3 m = affine().transpose();
  vec3 p = translation();

  // first 3 columns of jac
  jac[0]  = m(0, 0); jac[6]  = m(0, 1); jac[12] = m(0, 2);
  jac[1]  = m(1, 0); jac[7]  = m(1, 1); jac[13] = m(1, 2);
  jac[2]  = m(2, 0); jac[8]  = m(2, 1); jac[14] = m(2, 2);
  jac[3]  = 0.0;            jac[9]  = 0.0;            jac[15] = 0.0;
  jac[4]  = 0.0;            jac[10] = 0.0;            jac[16] = 0.0;
  jac[5]  = 0.0;            jac[11] = 0.0;            jac[17] = 0.0;

  // 4th column of jac
  jac[18] = p.y() * m(0, 2) - p.z() * m(0, 1);
  jac[19] = p.y() * m(1, 2) - p.z() * m(1, 1);
  jac[20] = p.y() * m(2, 2) - p.z() * m(2, 1);
  jac[21] = m(0, 0);
  jac[22] = m(1, 0);
  jac[23] = m(2, 0);

  // 5th column of jac
  jac[24] = p.z() * m(0, 0) - p.x() * m(0, 2);
  jac[25] = p.z() * m(1, 0) - p.x() * m(1, 2);
  jac[26] = p.z() * m(2, 0) - p.x() * m(2, 2);
  jac[27] = m(0, 1);
  jac[28] = m(1, 1);
  jac[29] = m(2, 1);

  // 6th column of jac
  jac[30] = p.x() * m(0, 1) - p.y() * m(0, 0);
  jac[31] = p.x() * m(1, 1) - p.y() * m(1, 0);
  jac[32] = p.x() * m(2, 1) - p.y() * m(2, 0);
  jac[33] = m(0, 2);
  jac[34] = m(1, 2);
  jac[35] = m(2, 2);
}
