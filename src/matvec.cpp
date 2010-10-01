//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: matvec.cpp,v 1.12 2009/04/08 15:25:52 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the classes: vec3, position , mat3 , Quaternion , and transf
 */
#include "matvec3D.h"

//#define GRASPITDBG
#include "debug.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GCC22

#define BADCONFIG                                     \
{                                                     \
  pr_error("not a valid configuration file!");        \
  fclose(fp);                                         \
  return FAILURE;                                     \
}

const vec3 vec3::ZERO(0, 0, 0);
const vec3 vec3::X(1, 0, 0);
const vec3 vec3::Y(0, 1, 0);
const vec3 vec3::Z(0, 0, 1);
const position position::ORIGIN(0, 0, 0);
const mat3 mat3::ZERO(vec3::ZERO,vec3::ZERO,vec3::ZERO);
const mat3 mat3::IDENTITY(vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1));
const Quaternion Quaternion::IDENTITY(1,0,0,0);
const transf transf::IDENTITY(Quaternion::IDENTITY,vec3::ZERO);

/*!
  Returns two vectors that are perpendicular to this vector and each other.
*/
void
vec3::perpVectors(vec3 &v1,vec3 &v2)
{
  double k;
  if (fabs(vec[2]) > M_SQRT1_2) {
    // choose v1 in y-z plane
    k = 1.0/sqrt(vec[1]*vec[1] + vec[2]*vec[2]);
    v1[0] = 0;
    v1[1] = -vec[2]*k;
    v1[2] = vec[1]*k;
  }
  else {
    // choose v1 in x-y plane
    k = 1.0/sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
    v1[0] = -vec[1]*k;
    v1[1] = vec[0]*k;
    v1[2] = 0;
  }

  v2 = *this * v1;
}

/*!
  Converts a rotation matrix to the Euler angles roll, pitch, and yaw.
*/
void
mat3::ToEulerAngles(double &roll,double &pitch, double &yaw) const
{
  double cosPitch;
  pitch = atan2(-mat[6],sqrt(mat[0]*mat[0]+mat[3]*mat[3]));
  if (pitch == M_PI/2.0) {
    yaw = 0.0;
    roll = atan2(mat[1],mat[4]);
  }
  else if (pitch == -M_PI/2.0) {
    yaw = 0.0;
    roll = -atan2(mat[1],mat[4]);
  }
  else {
    cosPitch = cos(pitch);
    yaw = atan2(mat[3]/cosPitch,mat[0]/cosPitch);
    roll = atan2(mat[7]/cosPitch,mat[8]/cosPitch);
  }
}

/*! Inverts a 3x3 matrix (does not assume matrix is orthonormal) */
mat3
mat3::inverse() const
{
  double det, detInv;
  double M[9];

  if (fabs(det = determinant()) < 1.0e-12) return mat3::ZERO;
  detInv = 1 / det;

  M[0] = (mat[4] * mat[8] - mat[7] * mat[5]) * detInv;
  M[1] = (mat[7] * mat[2] - mat[1] * mat[8]) * detInv;
  M[2] = (mat[1] * mat[5] - mat[4] * mat[2]) * detInv;
  M[3] = (mat[6] * mat[5] - mat[3] * mat[8]) * detInv;
  M[4] = (mat[0] * mat[8] - mat[6] * mat[2]) * detInv;
  M[5] = (mat[3] * mat[2] - mat[0] * mat[5]) * detInv;
  M[6] = (mat[3] * mat[7] - mat[6] * mat[4]) * detInv;
  M[7] = (mat[6] * mat[1] - mat[0] * mat[7]) * detInv;
  M[8] = (mat[0] * mat[4] - mat[3] * mat[1]) * detInv;

  return mat3(M);
}

/*! Returns this matrix post-multiplied with the matrix \a m. */
mat3 const&
mat3::operator*=(const mat3 &m)
{
  double M[9];
  memcpy(M,mat,9*sizeof(double));

  mat[0] = M[0]*m.mat[0] + M[3]*m.mat[1] + M[6]*m.mat[2]; 
  mat[1] = M[1]*m.mat[0] + M[4]*m.mat[1] + M[7]*m.mat[2]; 
  mat[2] = M[2]*m.mat[0] + M[5]*m.mat[1] + M[8]*m.mat[2]; 

  mat[3] = M[0]*m.mat[3] + M[3]*m.mat[4] + M[6]*m.mat[5]; 
  mat[4] = M[1]*m.mat[3] + M[4]*m.mat[4] + M[7]*m.mat[5]; 
  mat[5] = M[2]*m.mat[3] + M[5]*m.mat[4] + M[8]*m.mat[5]; 
  
  mat[6] = M[0]*m.mat[6] + M[3]*m.mat[7] + M[6]*m.mat[8]; 
  mat[7] = M[1]*m.mat[6] + M[4]*m.mat[7] + M[7]*m.mat[8]; 
  mat[8] = M[2]*m.mat[6] + M[5]*m.mat[7] + M[8]*m.mat[8]; 
  return *this;
}

/*! Returns the product of \a m1 * \a m2. */
mat3
operator*(const mat3 &m1, const mat3 &m2)
{
  double M[9];

  M[0] = m1.mat[0]*m2.mat[0] + m1.mat[3]*m2.mat[1] + m1.mat[6]*m2.mat[2]; 
  M[1] = m1.mat[1]*m2.mat[0] + m1.mat[4]*m2.mat[1] + m1.mat[7]*m2.mat[2]; 
  M[2] = m1.mat[2]*m2.mat[0] + m1.mat[5]*m2.mat[1] + m1.mat[8]*m2.mat[2]; 

  M[3] = m1.mat[0]*m2.mat[3] + m1.mat[3]*m2.mat[4] + m1.mat[6]*m2.mat[5]; 
  M[4] = m1.mat[1]*m2.mat[3] + m1.mat[4]*m2.mat[4] + m1.mat[7]*m2.mat[5]; 
  M[5] = m1.mat[2]*m2.mat[3] + m1.mat[5]*m2.mat[4] + m1.mat[8]*m2.mat[5]; 

  M[6] = m1.mat[0]*m2.mat[6] + m1.mat[3]*m2.mat[7] + m1.mat[6]*m2.mat[8]; 
  M[7] = m1.mat[1]*m2.mat[6] + m1.mat[4]*m2.mat[7] + m1.mat[7]*m2.mat[8]; 
  M[8] = m1.mat[2]*m2.mat[6] + m1.mat[5]*m2.mat[7] + m1.mat[8]*m2.mat[8]; 

  return mat3(M);
}

/*!
  Creates a 6x6 jacobian from a transform.  Code adapted from Peter Corke's
  Robot Toolbox for MATLAB.

 MATLAB representation:
\verbatim
	J = [	t(1:3,1)'	cross(t(1:3,4),t(1:3,1))'
			t(1:3,2)'	cross(t(1:3,4),t(1:3,2))'
			t(1:3,3)'	cross(t(1:3,4),t(1:3,3))'
			zeros(3,3)		t(1:3,1:3)'		];
\endverbatim

The jacobian is returned is in column-major format.
*/
void
transf::jacobian(double jac[])
{
  mat3 m = affine();
  vec3 p = translation();

  // first 3 columns of jac
  jac[0]  = m.element(0,0); jac[6]  = m.element(0,1); jac[12] = m.element(0,2);
  jac[1]  = m.element(1,0); jac[7]  = m.element(1,1); jac[13] = m.element(1,2);
  jac[2]  = m.element(2,0); jac[8]  = m.element(2,1); jac[14] = m.element(2,2);
  jac[3]  = 0.0;            jac[9]  = 0.0;            jac[15] = 0.0;
  jac[4]  = 0.0;            jac[10] = 0.0;            jac[16] = 0.0;
  jac[5]  = 0.0;            jac[11] = 0.0;            jac[17] = 0.0;

  // 4th column of jac
  jac[18] = p.y()*m.element(0,2) - p.z()*m.element(0,1);
  jac[19] = p.y()*m.element(1,2) - p.z()*m.element(1,1);
  jac[20] = p.y()*m.element(2,2) - p.z()*m.element(2,1);
  jac[21] = m.element(0,0);
  jac[22] = m.element(1,0);
  jac[23] = m.element(2,0);

  // 5th column of jac
  jac[24] = p.z()*m.element(0,0) - p.x()*m.element(0,2);
  jac[25] = p.z()*m.element(1,0) - p.x()*m.element(1,2);
  jac[26] = p.z()*m.element(2,0) - p.x()*m.element(2,2);
  jac[27] = m.element(0,1);
  jac[28] = m.element(1,1);
  jac[29] = m.element(2,1);

  // 6th column of jac
  jac[30] = p.x()*m.element(0,1) - p.y()*m.element(0,0);
  jac[31] = p.x()*m.element(1,1) - p.y()*m.element(1,0);
  jac[32] = p.x()*m.element(2,1) - p.y()*m.element(2,0);
  jac[33] = m.element(0,2);
  jac[34] = m.element(1,2);
  jac[35] = m.element(2,2);
}

//---------------------------------------------------------------------------
/*!
  Sets this quaternion by converting from rotation matrix \a R.
*/
void
Quaternion::set(const mat3 &R)
{
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".

    double trace = R.element(0,0)+R.element(1,1)+R.element(2,2);
    double root;

    if ( trace > 0.0 )
    {
        // |w| > 1/2, may as well choose w > 1/2
        root = sqrt(trace+1.0);  // 2w
        w = 0.5*root;
        root = 0.5/root;  // 1/(4w)
        x = (R.element(1,2)-R.element(2,1))*root;
        y = (R.element(2,0)-R.element(0,2))*root;
        z = (R.element(0,1)-R.element(1,0))*root;
    }
    else
    {
        // |w| <= 1/2
        static int next[3] = { 1, 2, 0 };
        int i = 0;
        if ( R.element(1,1) > R.element(0,0) )
            i = 1;
        if ( R.element(2,2) > R.element(i,i) )
            i = 2;
        int j = next[i];
        int k = next[j];

        root = sqrt(R.element(i,i)-R.element(j,j)-R.element(k,k)+1.0);
        double* quat[3] = { &x, &y, &z };
        *quat[i] = 0.5*root;
        root = 0.5/root;
        w = (R.element(j,k)-R.element(k,j))*root;
        *quat[j] = (R.element(i,j)+R.element(j,i))*root;
        *quat[k] = (R.element(i,k)+R.element(k,i))*root;
    }
    normalise();
}

/*!
  Sets this quaternion by converting the rotation from angle-axis formate.
  The angle is specified in radians.
*/
void
Quaternion::set(const double& angle, const vec3& axis)
{
    // assert:  axis[] is unit length
    //
	// The quaternion representing the rotation is
	//   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

    double halfAngle = 0.5*angle;
    double sn = sin(halfAngle);
    w = cos(halfAngle);
    x = sn*axis.x();
    y = sn*axis.y();
    z = sn*axis.z();
    normalise();
}

/*!
  Sets this quaternion by converting from Inventor format.
*/
void
Quaternion::set(const SbRotation &SbRot)
{
  w = (double)SbRot.getValue()[3];
  x = (double)SbRot.getValue()[0];
  y = (double)SbRot.getValue()[1];
  z = (double)SbRot.getValue()[2];
  normalise();
}

/*!
  Converts this quaternion to a 3x3 rotation matrix.
*/
void
Quaternion::ToRotationMatrix(mat3 &R) const
{
    double tx  = 2.0*x;
    double ty  = 2.0*y;
    double tz  = 2.0*z;
    double twx = tx*w;
    double twy = ty*w;
    double twz = tz*w;
    double txx = tx*x;
    double txy = ty*x;
    double txz = tz*x;
    double tyy = ty*y;
    double tyz = tz*y;
    double tzz = tz*z;

    R.element(0,0) = 1.0-(tyy+tzz);
    R.element(1,0) = txy-twz;
    R.element(2,0) = txz+twy;
    R.element(0,1) = txy+twz;
    R.element(1,1) = 1.0-(txx+tzz);
    R.element(2,1) = tyz-twx;
    R.element(0,2) = txz-twy;
    R.element(1,2) = tyz+twx;
    R.element(2,2) = 1.0-(txx+tyy);
}

/*!
  Converts this quaternion to angle-axis format.
*/
void
Quaternion::ToAngleAxis(double& angle, vec3& axis) const
{
	// The quaternion representing the rotation is
	//   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

    double length2 = x*x+y*y+z*z;
    if ( length2 > 1.0e-15 )
    {
        angle = 2.0*acos(w);
        double invlen = 1.0/sqrt(length2);
        axis[0] = x*invlen;
        axis[1] = y*invlen;
        axis[2] = z*invlen;
    }
    else
    {
        // angle is 0 (mod 2*pi), so any axis will do
        angle = 0.0;
        axis[0] = 1.0;
        axis[1] = 0.0;
        axis[2] = 0.0;
    }
}

/*!
  Computes a rotation between \a p and \a q using spherical linear
  interpolation and the scalar \a t [0..1].  If \a t is 0, \a p will be
  returned.  If \a t is 1, \a q will be returned.
*/  
Quaternion
Quaternion::Slerp (double t, const Quaternion& p,
		   const Quaternion& q)
{
  Quaternion q1;
  double sn,c0,c1,angle;
  double cs;
  
  if (t == 0.0) return p;
  if (t == 1.0) return q;

  cs = p % q;

  if (cs < 0.0) {
    cs = -cs;
    q1 = -q;
  }
  else q1 = q;

  // calculate interpolating coeffs
  if ((1.0 - cs) > 0.00001) {
    // standard case
    angle = acos(cs);
    sn = sin(angle);
    c0 = sin((1.0 - t) * angle) / sn;
    c1 = sin(t * angle) / sn;
  } else {        
    // q0 and q1 very close - just do linear interp.
    c0 = 1.0 - t;
    c1 = t;
  }

  return Quaternion(c0*p.w + c1*q1.w,c0*p.x + c1*q1.x,c0*p.y + c1*q1.y,
		    c0*p.z + c1*q1.z);
}


/*! Reads the values of a vector \a v from stream \a is formatted as
  "[#, #, #]"
 */
std::istream&
operator>>(std::istream &is, vec3 &v)
{
  char ch;
  double x,y,z;
  if (is >> ch && ch == '[' && is >> x && is >> y && is >> z &&
      is >> ch && ch == ']') {
    v.vec[0] = x; v.vec[1] = y; v.vec[2] = z;
  }
//  else
//    is.setstate(ios::failbit);
  
  return is;
}

/*! Writes the values of vector \a v to a stream \a os in the format
  "[#, #, #]".
 */
std::ostream &
operator<<(std::ostream &os, const vec3 &v)
{
#ifdef WIN32
  int oldFlags = os.setf(std::ios_base::showpos);
#else
#ifdef GCC22
 int oldFlags = os.setf(ios::showpos);
#else
  std::_Ios_Fmtflags oldFlags = os.setf(std::ios_base::showpos);
#endif 
#endif
  os << '[' << v.vec[0] << ' ' << v.vec[1] << ' ' << v.vec[2] << ']';
  os.flags(oldFlags);
  return os;
}

/*! Reads the values of a position \a p from stream \a is formatted as
  "[#, #, #]".
*/
std::istream&
operator>>(std::istream &is, position &p)
{
  char ch;
  double x,y,z;
  if (is >> ch && ch == '[' && is >> x && is >> y && is >> z &&
      is >> ch && ch == ']') {
    p.pos[0] = x; p.pos[1] = y; p.pos[2] = z;
  } 
//  else
//    is.setstate(ios::failbit);

  return is;
}

/*! Writes the values of position \a p to a stream \a os in the format
  "[#, #, #]". */
std::ostream &
operator<<(std::ostream &os, const position &p)
{
#ifdef WIN32
  int oldFlags = os.setf(std::ios_base::showpos);
#else
#ifdef GCC22
 int oldFlags = os.setf(ios::showpos);
#else
  std::_Ios_Fmtflags oldFlags = os.setf(std::ios_base::showpos);
#endif 
#endif
  os << '[' << p.pos[0] << ' ' << p.pos[1] << ' ' << p.pos[2] << ']';
  os.flags(oldFlags);
  return os;
}

/*! Reads the 9 consecutive values of a 3x3 matrix in column major format. */
std::istream &
operator>>(std::istream &is, mat3 &m)
{
  is >> m.mat[0] >> m.mat[3] >> m.mat[6];
  is >> m.mat[1] >> m.mat[4] >> m.mat[7];
  is >> m.mat[2] >> m.mat[5] >> m.mat[8];
  return is;
}

/*!
  Writes the matrix out on 3 lines in the following format:
\verbatim
[ # # #]
[ # # #]
[ # # #]
\endverbatim
*/
std::ostream &
operator<<(std::ostream &os, const mat3 &m)
{
#ifdef WIN32
  int oldFlags = os.setf(std::ios_base::showpos);
#else
#ifdef GCC22
 int oldFlags = os.setf(ios::showpos);
#else
  std::_Ios_Fmtflags oldFlags = os.setf(std::ios_base::showpos);
#endif
#endif
  os << '[' << m.mat[0] << ' ' << m.mat[3] << ' ' << m.mat[6]<< ']'<<std::endl;
  os << '[' << m.mat[1] << ' ' << m.mat[4] << ' ' << m.mat[7]<< ']'<<std::endl;
  os << '[' << m.mat[2] << ' ' << m.mat[5] << ' ' << m.mat[8]<< ']'<<std::endl;
  os.flags(oldFlags);
  return os;
}

/*!
  Reads the 4 values of a quaternion \a q formatted as "(w, x, y, z)" from a 
  stream \a is.
*/
std::istream &
operator>>(std::istream &is, Quaternion &q)
{
  char ch;
  double w,x,y,z;
  if (is >> ch && ch == '(' && is >> w && is >> x && is >> y && is >> z &&
      is >> ch && ch == ')') {
    q.w = w; q.x = x; q.y = y; q.z = z;
  } 
//  else
//    is.setstate(ios::failbit);
  
  return is;
}

/*!
  Writes the 4 values of a quaternion \a q in the format "(w, x, y, z)" to as
  stream \a os.
*/
std::ostream&
operator<<(std::ostream &os, const Quaternion &q)
{
#ifdef WIN32
  int oldFlags = os.setf(std::ios_base::showpos);
#else
#ifdef GCC22
 int oldFlags = os.setf(ios::showpos);
#else
  std::_Ios_Fmtflags oldFlags = os.setf(std::ios_base::showpos);
#endif
#endif 
  os <<'('<< q.w <<' '<< q.x <<' '<< q.y <<' '<< q.z <<')';
  os.flags(oldFlags);
  return os;
}

/*!
  Reads the 7 values of a transform \a tr formated as "(w, x, y, z) [v1,v2,v3]"
  from stream \a is.
*/
std::istream&
operator>>(std::istream &is, transf &tr)
{
  Quaternion q;
  vec3 v;

  if (is >> q && is >> v)
    tr.set(q,v);
//  else
//    is.setstate(ios::failbit);

  return is;
}

/*!
  Writes the 7 values of a transform \a tr formated as
  "(w, x, y, z) [v1,v2,v3]" to stream \a os.
*/
std::ostream &
operator<<(std::ostream &os, const transf &tr)
{
  return os << tr.rot << tr.t;
}

void FlockTransf::identity()
{
	mount = transf::IDENTITY;
	mountInv = transf::IDENTITY;
	flockBaseInv = transf::IDENTITY;
	objBase = transf::IDENTITY;
}

transf FlockTransf::get(transf t) const
{
	return mountInv * t * flockBaseInv * mount * objBase;
}

transf FlockTransf::getAbsolute(transf t) const
{
	return mountInv * t;
}

/*!	Does not multiply with the mount transform. It is meant to use with 
	camera, and we assume we already have the camera mount built into the
	objectBase.
*/
transf FlockTransf::get2(transf t) const
{
	return mountInv * t * flockBaseInv * objBase;
}
