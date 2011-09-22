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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: matvec3D.h,v 1.17 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the classes: vec3, position, mat3, Quaternion, and transf.
 */
#ifndef _MATVEC_H_
#include <math.h>
#include <string.h>
#include <iostream>
#include <stack>

#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoTransform.h>

class vec3;
class position;
class mat3;
class Quaternion;
class transf;

#define MACHINE_ZERO 1.0e-9
#define resabs 1.0e-5

typedef double col_Mat4[4][4];

//! A 3-dimensional double vector and methods to operate on them.
/*!
  vec3 is stored as an array of 3 doubles.
 */
class vec3 {

  //! Storarge for the 3 vector coordinates.
  double vec[3];

public:
  /*!  Default constructor, vector is uninitialized. */
  vec3() {}

  /*! Copy constructor. */
  vec3(const vec3& v) {set(v);}

  /*! Initializes the vector with the three values in the array \a v. */
  vec3(double v[3])   {set(v);}

  /*! Initializes the vector with the values from the Inventor vector \a v. */
  vec3(const SbVec3f& v) {set(v);}

  /*! Initializes the vector with the values \a x,\a y,\a z. */
  vec3(double x,double y,double z) {set(x,y,z);}

  /*! Returns a constant reference to the value at index \a i. */
  const double& operator[](int i) const {return vec[i];}

  /*! Returns a reference to the value at index \a i. */
  double& operator[](int i) {return vec[i];}

  /*! Copies vector \a v. */
  void set(const vec3& v) {vec[0] = v[0]; vec[1] = v[1]; vec[2] = v[2];}
  
  /*! Sets the 3 values in this vector to the 3 values in the array \a v. */
  void set(double v[3])   {vec[0] = v[0]; vec[1] = v[1]; vec[2] = v[2];}

  /*! Sets the 3 values in this vector to \a x, \a y, \a z. */
  void set(double x,double y,double z) {vec[0] = x; vec[1] = y; vec[2] = z;}

  inline double len() const;
  inline double len_sq() const;

  inline void set(const SbVec3f &v);  
  inline void toSbVec3f(SbVec3f &v) const;
  inline SbVec3f toSbVec3f() const;

  // input output

  inline vec3 const& operator=(vec3 const& v);
  inline vec3 const& operator*=(double s);
  //  inline vec3 const& operator*=(mat3 const& m);
  //inline vec3 const& operator*=(transf const& t);
  inline vec3 const& operator+=(vec3 const& v);
  inline vec3 const& operator-=(vec3 const& v);
  inline vec3 const& operator/=(double s);
  inline bool operator==(vec3 const& v) const;
  inline bool operator<(vec3 const& v) const;

  /*! Returns the x-coordinate of this vector. */
  double x() const {return vec[0];}

  /*! Returns the y-coordinate of this vector. */
  double y() const {return vec[1];}

  /*! Returns the z-coordinate of this vector. */
  double z() const {return vec[2];}

  /*! Returns a reference to the x-coordinate of this vector. */
  double& x() {return vec[0];}

  /*! Returns a reference to the y-coordinate of this vector. */
  double& y() {return vec[1];}

  /*! Returns a reference to the z-coordinate of this vector. */
  double& z() {return vec[2];}  


  void perpVectors(vec3 &v1,vec3 &v2); 

  /*  friend double operator%(position const& p, vec3 const& v);
      friend double operator%(vec3 const& v, position const& p);*/

  friend inline double operator%(vec3 const& v1, vec3 const& v2);  // dot prod
  friend inline double operator%(position const& p, vec3 const& v);
  friend inline double operator%(vec3 const& v, position const& p);
  friend inline vec3   operator*(double s, vec3 const& v);
  friend inline vec3   operator*(vec3 const& v, double s);
  friend inline vec3   operator*(mat3 const& m, vec3 const& v);

  friend inline vec3   operator*(vec3 const& v, transf const& tr);
  friend inline vec3   operator>(vec3 const& v, transf const& tr);
  friend inline vec3   operator*(vec3 const& v1, vec3 const& v2);  // cross
  friend inline vec3   operator+(vec3 const& v1, vec3 const& v2);
  friend inline vec3   operator-(vec3 const& v1, vec3 const& v2);
  friend inline vec3   operator-(vec3 const& v);  
  friend inline vec3   operator/(vec3 const& v, double s);
  friend inline vec3   normalise(vec3 const& v);

  friend std::istream &operator>>(std::istream &is, vec3 &v);
  friend std::ostream &operator<<(std::ostream &os, const vec3 &v);

  //! The vector [0,0,0]
  static const vec3 ZERO;

  //! The vector [1,0,0]
  static const vec3 X;

  //! The vector [0,1,0]
  static const vec3 Y;

  //! The vector [0,0,1]
  static const vec3 Z;
};


//! A 3-dimensional position and methods to operate on them.
/*!
  position is stored as an array of 3 doubles.
 */
class position {

  //! Storarge for the 3 position coordinates.
  double pos[3];

public:

  /*!  Default constructor, position is uninitialized. */
  position() {}

  /*! Copy constructor. */
  position(const position& p) {set(p);}

  /*! Initializes the position with the three values in the array \a v. */
  position(double p[3])       {set(p);}

  /*! Initializes the vector with the values from the Inventor vector \a v. */
  position(const SbVec3f& p)  {set(p);}

  /*! Initializes the vector with the values \a x, \a y, \a z. */
  position(double x,double y,double z) {set(x,y,z);}
    
  /*! Returns a constant reference to the value at index \a i. */
  const double& operator[](int i) const {return pos[i];}

  /*! Returns a reference to the value at index \a i. */
  double& operator[](int i) {return pos[i];}

  /*! Copies vector \a v. */
  void set(const position& p) {pos[0] = p[0]; pos[1] = p[1]; pos[2] = p[2];}

  /*! Sets the 3 values in this vector to the 3 values in the array \a v. */
  void set(double p[3])       {pos[0] = p[0]; pos[1] = p[1]; pos[2] = p[2];}

  /*! Sets the 3 values in this vector to \a x, \a y, \a z. */
  void set(double x,double y,double z) {pos[0] = x; pos[1] = y; pos[2] = z;}

  void set(const vec3 &v){pos[0] = v.x(); pos[1] = v.y(); pos[2] = v.z();}

  /*! Gets the 3 values that make up this position */
  void get(double p[3]) const {p[0] = pos[0]; p[1] = pos[1]; p[2] = pos[2];}

  inline void set(const SbVec3f &v);
  inline void toSbVec3f(SbVec3f &v) const;
  inline SbVec3f toSbVec3f() const;

  // input output

  inline position const& operator=(position const& p);
  //  inline position const& operator*=(mat3 const& m);
  //  inline position const& operator*=(transf const& tr);
  inline position const& operator+=(vec3 const& v);
  inline position const& operator-=(vec3 const& v);
  inline bool operator==(position const& p) const;

  /*! Returns the x-coordinate of this vector. */  
  double x() const {return pos[0];}

  /*! Returns the y-coordinate of this vector. */
  double y() const {return pos[1];}

  /*! Returns the z-coordinate of this vector. */
  double z() const {return pos[2];}

  /*! Returns a reference to the x-coordinate of this vector. */
  double& x() {return pos[0];}

  /*! Returns a reference to the y-coordinate of this vector. */
  double& y() {return pos[1];}

  /*! Returns a reference to the z-coordinate of this vector. */
  double& z() {return pos[2];}  

  friend inline double     operator%(position const& p, vec3 const& v);  // dot
  friend inline double     operator%(vec3 const& v, position const& p);  // dot
  //  friend inline position   operator*(mat3 const& m, position const& p);
  inline friend position   operator*(double d, position const& p);
  inline friend position   operator*(position const& p, transf const& tr);
  friend inline position   operator+(position const& p, vec3 const& v);
  friend inline position   operator-(position const& p, vec3 const& v);
  friend inline position   operator+(vec3 const& v, position const& p);
  friend inline vec3       operator-(position const& p1, position const& p2);
  friend inline position   operator+(position const& p1, position const& p2);

  friend std::istream&       operator>>(std::istream &is, position &p);
  friend std::ostream&       operator<<(std::ostream &os, const position &p); 

  //! The position [0,0,0].
  static const position ORIGIN;
};


//! A 3x3 matrix most often used for storing rotations.
/*!
  The matrix is stored as a column-major array.
 */
class mat3 {
  
  //! Storage for the matrix
  double mat[9];

public:

  /*! Default constructor, matrix is uninitialized. */
  mat3() {}

  /*! Copy constructor. */  
  mat3(const double M[9]) {set(M);}

  /*! Initializes the rotation matrix using the quaternion \a q. */  
  mat3(const Quaternion &q) {set(q);}

  /*! Initializes the rotation matrix using the three column vectors \a v1,
    \a v2, \a v3.*/
  mat3(const vec3 &v1, const vec3 &v2, const vec3 &v3) {set(v1,v2,v3);}

  /*! Copies the matrix \a M. */
  void set(const double M[9]) {memcpy(mat,M,sizeof(double)*9);}

  inline void set(const Quaternion &q);
  inline void set(const vec3 &v1, const vec3 &v2, const vec3 &v3);

  inline void setCrossProductMatrix(const vec3 &v);

  void ToEulerAngles(double &roll,double &pitch, double &yaw) const;

  /*! Return a constant reference to matrix element at row \a i, col \a j. */
  const double& element(int i,int j) const {return mat[j*3+i];}

  /*! Return a reference to matrix element at row \a i, col \a j. */
        double& element(int i,int j)       {return mat[j*3+i];}

  /*! Return a constant reference to the i-th element (in column major
    order)  of the matrix. */
  const double& operator[](int i) const {return mat[i];}
  /*! Return a reference to the i-th element (in column major order) of the
      matrix. */
        double& operator[](int i)       {return mat[i];}

  /*! Return a copy of the i-th row of the matrix as a vector. */
        vec3    row(int i) const {return vec3(mat[i],mat[i+3],mat[i+6]);}
  
  inline        double        determinant() const;

  inline        mat3          transpose() const;
                mat3          inverse() const;

                mat3 const&   operator*=(mat3 const& m);
		 friend vec3          operator*(mat3 const& m,  vec3 const& v);
  //     friend position      operator*(mat3 const& m,  position const& p);
	     friend vec3          operator*(vec3 const& v,  mat3 const& m);
         friend mat3          operator*(mat3 const& m1, mat3 const& m2);
  inline friend mat3          operator*(double const& s,  mat3 const& m);
  inline mat3 const& operator+=(mat3 const& v);

  friend std::istream&     operator>>(std::istream &is, mat3 &m);
  friend std::ostream&     operator<<(std::ostream &os, const mat3 &m);  


  //! A 3x3 zero matrix.
  static const mat3 ZERO;
  
  //! A 3x3 identity matrix.
  static const mat3 IDENTITY;  
};

//! A 4-dimensional unit vector used to store a rotation
/*!
  Quaternions are an efficient way to store and manipulate rotations.
 */
class Quaternion {
public:
  //! The four scalar components of a quaternion
  double w, x, y, z;
  
  /*! Default constructor, Quaternion is initialized to an identity rotation */
  Quaternion () : w(1.0),x(0.0),y(0.0),z(0.0) {}

  /*! Initializes quaternion with 4 scalar values (automatically normalized).*/
  Quaternion (double W, double X, double Y, double Z) {set(W,X,Y,Z);}

  /*! Initializes quaternion using a rotation matrix \a R. */
  Quaternion (const mat3& R) {set(R);}

  /*! Initializes quaternion using a rotation exressed as in angle and axis
      format. */
  Quaternion (const double& angle, const vec3 &axis) {set(angle,axis);}

  /*! Copies the values of quaterion Q. */
  void set(const Quaternion& Q)
    {w = Q.w; x = Q.x; y = Q.y; z = Q.z;}

  /*! Sets the value of the quaternion using 4 scalars and normalizes it. */
  void set(double W, double X, double Y, double Z)
    {w = W; x = X; y = Y; z = Z; normalise();}

  void set(const mat3& R);
  void set(const double& angle, const vec3 &axis);
  void set(const SbRotation &SbRot);

  void ToRotationMatrix (mat3& m) const;  
  void ToAngleAxis (double& angle, vec3& axis) const;
  inline SbRotation toSbRotation() const;

  // arithmetic operations
  inline Quaternion& operator= (const Quaternion& q);
  inline Quaternion operator+ (const Quaternion& q) const;
  inline Quaternion operator- (const Quaternion& q) const;
  inline Quaternion operator* (const Quaternion& q) const;
  //inline Quaternion operator* (double c) const;
  inline Quaternion operator- () const;
  inline double     operator% (const Quaternion& q) const;  // dot product 
  //  inline friend Quaternion operator* (double c, const Quaternion& q);
  inline bool operator==(Quaternion const& q) const;

  // functions of a quaternion
  inline double norm () const;  // squared-length of quaternion
  inline Quaternion inverse () const;  // apply to unit quaternion
  
  // rotation of a vector by a quaternion
  inline vec3 operator* (const vec3& v) const;
  
  // rotation of a position by a quaternion
  inline position operator* (const position& pt) const;
  
  // spherical linear interpolation
  static Quaternion Slerp (double t, const Quaternion& p,
			   const Quaternion& q);
  
  // setup for spherical quadratic interpolation
  static void Intermediate (const Quaternion& q0, const Quaternion& q1,
			    const Quaternion& q2, Quaternion& a, Quaternion& b);
  
  // spherical quadratic interpolation
  static Quaternion Squad (double t, const Quaternion& p,
			   const Quaternion& a, const Quaternion& b, const Quaternion& q);
  
  inline void normalise();
  
  friend std::istream& operator>>(std::istream &is, Quaternion &q);
  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
  
  //! An identity rotation [1,0,0,0]
  static const Quaternion IDENTITY;

};

//! A rigid body transformation
/*!
  The transform is stored as a rotation quaternion and a translation vector.
  A rotation matrix version is also stored so that it does not need to be
  recomputed each time it is requested.  (This maybe eliminated at some point).
 */
class transf {

  //! A 3x3 matrix storing the current rotation
  mat3 R;

  //! A 3x1 vector storing the current translation
  vec3 t;

  //! A 4x1 Quaternion storing the current rotation
  Quaternion rot;

public:

  /*! Default constructor, initializes the transform to the identity */
  transf() {R = mat3::IDENTITY; t = vec3::ZERO; rot = Quaternion::IDENTITY;}

  /*! Initializes transform with a rotation quaternion and translation vector*/
  transf(const Quaternion& r, const vec3& d) {set(r,d);}

  /*! Initializes transform with a rotation matrix and translation vector */
  transf(const mat3& r, const vec3& d) {set(r,d);}

  /*! Initializes transform using an Inventor transform. */
  transf(const SoTransform *IVt) {set(IVt);}

  /*! Sets value of the transform with the rotation \a r and the translation
    \a d */
  void set(const Quaternion& r, const vec3& d) {
    rot = r; t = d; rot.ToRotationMatrix(R);
  }
  
  /*! Sets value of the transform with the rotation \a r and the translation
    \a d */
  void set(const mat3& r, const vec3& d) {rot.set(r); t = d; R = r;}

  inline void set(const SoTransform *IVt);

  /*! Returns the rotation portion of the transform. */
  const Quaternion& rotation() const {return rot;}

  /*! Returns the rotation portion of the transform as a 3x3 matrix. */
  const mat3& affine() const {return R;}

  /*! Returns the translation portion of the transform. */
  const vec3& translation() const {return t;}

  inline void toSoTransform(SoTransform *IVt) const;
  inline void tocol_Mat4(col_Mat4 colTran) const;
  inline void toColMajorMatrix(double t[][4]) const;
  inline void toRowMajorMatrix(double t[][4]) const;
  //! Creates a 6x6 jacobian from this transform
  void jacobian(double jac[]);
  
  inline transf          inverse() const;
  
  inline transf const&   operator=(transf const& tr);
  inline bool operator==(transf const& tr) const;
  
  /*! Returns the negation of the == operator. */
  inline bool operator!=(transf const& tr) const {return !operator==(tr);}


  //  inline transf const&   operator*=(transf const& tr);

  inline friend transf   operator*(transf const& tr1,  transf const& tr2);
  inline friend vec3     operator*(vec3 const& v, transf const& tr);
  inline friend vec3     operator>(vec3 const& v, transf const& tr);
  inline friend position operator*(position const& p, transf const& tr);


  inline friend transf translate_transf(const vec3& v);
  inline friend transf rotate_transf(double angle, const vec3& axis);
  inline friend transf coordinate_transf(const position& origin,
					 const vec3& xaxis, const vec3& yaxis);
  inline friend transf rotXYZ(double, double, double);

  friend std::istream& operator>>(std::istream &is, transf &tr);
  friend std::ostream& operator<<(std::ostream &os, const transf &tr);

  //! The Identity transform (1,0,0,0)[0,0,0]
  static const transf IDENTITY;
};

//! A class for easy storage and manipulation of Flock of Birds transform
/*! A Flock of Birds transform, when applied to a robot or object
	in GraspIt! has to take into account where the sensor is mounted
	on the robot. It must also be able to use the flock transform in 
	relative terms, so we get the movement of the GraspIt body, but not
	in absolute terms. This class stores all of that conveniently.

	Assume that, at the moment where we start measuring, the object's
	transform is Ob and the Flock transform ir Fb. The transform from 
	object origin to sensor mount location is M.

	At a given time, we read from the Flock sensor the transform F. What
	is the correct GraspIt transform for the body?

	- in relative operation: MInv * F * FbInv * M * Ob

	- in absolute operation: MInv * F
*/
class FlockTransf{
private:
	transf mount, mountInv;
	transf flockBaseInv, objBase;

public:
	void identity();
	//! Set the tranform from object origin to sensor mounting location
	void setMount(transf t){mount = t; mountInv = t.inverse();}
	//! Set the base flock transform, that all subsequent flock transforms are relative to
	void setFlockBase(transf t){flockBaseInv = t.inverse();}
	//! Set the base object transform, the one that corresponds to the base flock trasnform
	void setObjectBase(transf t){objBase = t;}
	//! Get the object's GraspIt transform for a give flock transform read from the sensor
	transf get(transf t) const;
	//! Get the object's GraspIt transform if we are operating in absolute terms
	transf getAbsolute(transf t) const;
	//! Get the object transform in relative terms, but without factoring in a mount transform
	transf get2(transf t) const;
	transf getMount() const {return mount;}
};

///////////////////////////////////////////////////////////////////////////////
//                       vec3  functions
///////////////////////////////////////////////////////////////////////////////

/*!
  Sets the values in this vector with the values from the Inventor vector \a v.
 */
void
vec3::set(const SbVec3f &v)
{
  vec[0] = (double) v[0];
  vec[1] = (double) v[1];
  vec[2] = (double) v[2];
}

/*!
  Converts the values of this vector into floats and stores them in the
  provided Inventor vector.
*/
void
vec3::toSbVec3f(SbVec3f &v) const
{
  v[0] = (float) vec[0];
  v[1] = (float) vec[1];
  v[2] = (float) vec[2];
}

/*!
  Converts this vector into an Inventor vector and returns it.
*/
SbVec3f
vec3::toSbVec3f() const
{
  return SbVec3f((float) vec[0],(float) vec[1],(float) vec[2]);
}

/*!
  Returns the dot product of vectors \a v1 and \a v2.
*/
double
operator%(vec3 const& v1, vec3 const& v2)
{
  return v1.vec[0]*v2.vec[0] + v1.vec[1]*v2.vec[1] + v1.vec[2]*v2.vec[2];
}

/*!
  Returns a new vector that is the result of scaling this vector by \a s.
*/
vec3
operator*(double s, vec3 const& v)
{
  return vec3(s*v.vec[0],s*v.vec[1],s*v.vec[2]);
}

/*!
  Returns a new vector that is the result of scaling this vector by \a s.
*/ 
vec3
operator*(vec3 const& v, double s)
{
  return vec3(s*v.vec[0],s*v.vec[1],s*v.vec[2]);
}

/*!
  Returns the length of this vector.
*/
double
vec3::len() const
{
  return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

/*!
  Returns the squared length of this vector.
*/
double
vec3::len_sq() const
{
  return vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
}

/*! Copies the vector \a v. */
vec3 const&
vec3::operator=(vec3 const& v)
{
  vec[0] = v[0]; vec[1] = v[1]; vec[2] = v[2];
  return *this;
}

/*! Scales this vector by \a s. */
vec3 const&
vec3::operator*=(double s)
{
  vec[0] *= s;   vec[1] *= s;   vec[2] *= s;
  return *this;
}

/* 
// Not used right now.
vec3 const&
vec3::operator*=(mat3 const& m)
{
  double v[3];
  memcpy(v,vec,3*sizeof(double));

  vec[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
  vec[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
  vec[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
  return *this;
}
*/

/*! Adds vector \a v to this vector. */
vec3 const&
vec3::operator+=(vec3 const& v)
{
  vec[0] += v[0]; vec[1] += v[1]; vec[2] += v[2];
  return *this;
}

/*! Subtract vector \a v from this vector. */
vec3 const&
vec3::operator-=(vec3 const& v)
{
  vec[0] -= v[0]; vec[1] -= v[1]; vec[2] -= v[2];
  return *this;
}

/*! Scales this vector by 1/s. */
vec3 const&
vec3::operator/=(double s)
{
  vec[0] /= s;   vec[1] /= s;   vec[2] /= s;
  return *this;
}

/*! Compares if 2 vectors are within \a resabs distance from each other. */
bool
vec3::operator==(vec3 const& v) const
{
  if ((*this - v).len_sq() < resabs * resabs)
    return true;
  return false;
}

/*! 
  Compares 2 vectors lexigraphically.  Returns true if this vector comes
  before vector \a v.  Vectors are ordered on z axis then y and then x. 
 */
bool
vec3::operator<(vec3 const& v) const
{
  double v0diff,v1diff,v2diff;
  v2diff = vec[2]-v[2];  
  if (v2diff < -resabs) return true;
  else if (v2diff < resabs) {
    v1diff = vec[1]-v[1];
    if (v1diff < -resabs) return true;
    else if (v1diff < resabs) {
      v0diff = vec[0]-v[0];
      if (v0diff < -resabs) return true;      
    }
  }
  return false;
}

/*! Returns the dot product of vector \a v and position \a p. */
double
operator%(position const& p, vec3 const& v)
{
  return p.pos[0]*v.vec[0] + p.pos[1]*v.vec[1] + p.pos[2]*v.vec[2];
}

/*! Returns the dot product of vector \a v and position \a p. */
double
operator%(vec3 const& v, position const& p)
{
  return p.pos[0]*v.vec[0] + p.pos[1]*v.vec[1] + p.pos[2]*v.vec[2];
}


vec3
operator*(const mat3 &m, const vec3 &v)
{
  double result[3];
  result[0] = m[0] * v.vec[0] + m[3] * v.vec[1] + m[6] * v.vec[2];
  result[1] = m[1] * v.vec[0] + m[4] * v.vec[1] + m[7] * v.vec[2];
  result[2] = m[2] * v.vec[0] + m[5] * v.vec[1] + m[8] * v.vec[2];
  return vec3(result);
}


/*! Returns the cross product of vectors \a v1 and \a v2 ( \a v1 x \a v2 ). */
vec3
operator*(vec3 const& v1, vec3 const& v2)  // cross pr
{
  return vec3(v1.vec[1]*v2.vec[2]-v1.vec[2]*v2.vec[1],
	      v1.vec[2]*v2.vec[0]-v1.vec[0]*v2.vec[2],
	      v1.vec[0]*v2.vec[1]-v1.vec[1]*v2.vec[0]);
}

/*! Returns the sum of vectors \a v1 and \a v2. */
vec3
operator+(vec3 const& v1, vec3 const& v2)
{
  return vec3(v1.vec[0]+v2.vec[0],v1.vec[1]+v2.vec[1],v1.vec[2]+v2.vec[2]);
}


/*! Returns the difference between vectors \a v1 and \a v2. */
vec3
operator-(vec3 const& v1, vec3 const& v2)
{
  return vec3(v1.vec[0]-v2.vec[0],v1.vec[1]-v2.vec[1],v1.vec[2]-v2.vec[2]);
}
 

/*! Returns a vector that is the negative of vector \a v. (unary minus) */
vec3
operator-(vec3 const& v)
{
  return vec3(-v.vec[0],-v.vec[1],-v.vec[2]);
}
  

/*! Returns a new vector that is the result of scaling this vector by 1/s. */
vec3
operator/(vec3 const& v, double s)
{
  return vec3(v.vec[0]/s,v.vec[1]/s,v.vec[2]/s);
}

/*! Returns a new unit vector that is the normalized version of vector \a v. */
vec3
normalise(vec3 const& v)
{
  double length = v.len();
#ifdef GRASPITDBG
  if (length == 0.0)
    printf("0 length in vec3 normalise\n");
#endif
  return v/length;
}

 


///////////////////////////////////////////////////////////////////////////////
//                       position inline functions
///////////////////////////////////////////////////////////////////////////////

/*!
  Sets the values in this vector with the values from the Inventor vector \a v.
 */
void
position::set(const SbVec3f &v)
{
  pos[0] = (double) v[0];
  pos[1] = (double) v[1];
  pos[2] = (double) v[2];
}

/*!
  Converts the values of this vector into floats and stores them in the
  provided Inventor vector.
*/
void
position::toSbVec3f(SbVec3f &v) const
{
  v[0] = (float) pos[0];
  v[1] = (float) pos[1];
  v[2] = (float) pos[2];
}

/*!
  Converts this vector into an Inventor vector and returns it.
*/
SbVec3f
position::toSbVec3f() const
{
  return SbVec3f((float) pos[0],(float) pos[1],(float) pos[2]);
}

/*! Copies the position \a p. */
position const&
position::operator=(position const& p)
{
  pos[0] = p[0]; pos[1] = p[1]; pos[2] = p[2];
  return *this;
}

/*
//not used
position const&
position::operator*=(mat3 const& m)
{
  double p[3];
  memcpy(p,pos,3*sizeof(double));

  pos[0] = m[0] * p[0] + m[3] * p[1] + m[6] * p[2];
  pos[1] = m[1] * p[0] + m[4] * p[1] + m[7] * p[2];
  pos[2] = m[2] * p[0] + m[5] * p[1] + m[8] * p[2];
  return *this;
}
*/

/*! Adds vector \a v to this position. */
position const&
position::operator+=(vec3 const& v)
{
  pos[0] += v[0]; pos[1] += v[1]; pos[2] += v[2];
  return *this;
}

/*! Subtracts vector \a v from this position. */
position const&
position::operator-=(vec3 const& v)
{
  pos[0] -= v[0]; pos[1] -= v[1]; pos[2] -= v[2];
  return *this;
}

/*
position
operator*(const mat3 &m, const position &p)
{
  double result[3];
  result[0] = m.mat[0] * p.pos[0] + m.mat[3] * p.pos[1] + m.mat[6] * p.pos[2];
  result[1] = m.mat[1] * p.pos[0] + m.mat[4] * p.pos[1] + m.mat[7] * p.pos[2];
  result[2] = m.mat[2] * p.pos[0] + m.mat[5] * p.pos[1] + m.mat[8] * p.pos[2];
  return position(result);
}
*/

/*! Returns the result of adding vector \a v to position \a p. */
position
operator+(position const& p, vec3 const& v)
{
  return position(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
}

/*! Returns the result of subtracting vector \a v from position \a p. */
position
operator-(position const& p, vec3 const& v)
{
  return position(p[0]-v[0], p[1]-v[1], p[2]-v[2]);
}

/*! Returns the result of adding vector \a v to position \a p. */
position
operator+(vec3 const& v, position const& p)
{
  return position(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
}

/*! Returns the result of subtracting position \a p2 from position \a p1. */
vec3
operator-(position const& p1, position const& p2)
{
  return vec3(p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]);
}

position
operator+(position const& p1, position const& p2)
{
  return position(p1[0]+p2[0], p1[1]+p2[1], p1[2]+p2[2]);
}

position
operator*(double d, position const& p)
{
	return position(d*p[0], d*p[1], d*p[2]);
}

/*! Returns true if two positions are separated by a distance less the
  \a resabs.
 */
bool
position::operator==(position const& p) const
{
  if ((*this - p).len_sq() < resabs * resabs)
    return true;
  return false;
}

///////////////////////////////////////////////////////////////////////////////
//                       mat3 inline functions
///////////////////////////////////////////////////////////////////////////////


/*! Converts a quaternion to a 3x3 rotation matrix and sets the mat3 to this
    value.
*/
void
mat3::set(const Quaternion &q)
{
  double tx  = 2.0*q.x;
  double ty  = 2.0*q.y;
  double tz  = 2.0*q.z;
  double twx = tx*q.w;
  double twy = ty*q.w;
  double twz = tz*q.w;
  double txx = tx*q.x;
  double txy = ty*q.x;
  double txz = tz*q.x;
  double tyy = ty*q.y;
  double tyz = tz*q.y;
  double tzz = tz*q.z;
  
  mat[0] = 1.0-(tyy+tzz);    mat[1] = txy-twz;          mat[2] = txz+twy;
  mat[3] = txy+twz;          mat[4] = 1.0-(txx+tzz);    mat[5] = tyz-twx;
  mat[6] = txz-twy;          mat[7] = tyz+twx;          mat[8] = 1.0-(txx+tyy);
}

/*! Value of the mat3 is set using the 3 column vectors \a v1, \a v2, \a v3. */
void
mat3::set(const vec3 &v1, const vec3 &v2, const vec3 &v3)
{
  mat[0] = v1[0]; mat[3] = v1[1]; mat[6] = v1[2];
  mat[1] = v2[0]; mat[4] = v2[1]; mat[7] = v2[2];
  mat[2] = v3[0]; mat[5] = v3[1]; mat[8] = v3[2];
}

/*! This matrix becomes the cross product matrix for the given vector. 
 In other words, multiplying a second vector v2 with this matrix is 
 equivalent with perrforming the cross product v2 * v */
void 
mat3::setCrossProductMatrix(const vec3 &v)
{
	mat[0]=0.0;
	mat[1]= v.z();
	mat[2]=-v.y();
  
	mat[3]=-v.z();
	mat[4]=0.0;
	mat[5]= v.x();
  
	mat[6]= v.y();
	mat[7]=-v.x();
	mat[8]=0.0;

}

/*! Returns the determinant of the matrix. */
double
mat3::determinant() const
{
  return  mat[0] * (mat[4] * mat[8] - mat[7] * mat[5])
        + mat[3] * (mat[7] * mat[2] - mat[1] * mat[8])
        + mat[6] * (mat[1] * mat[5] - mat[4] * mat[2]);
}

/*! Returns a mat3 that is the transpose of this one. */
mat3
mat3::transpose() const
{
  double M[9];

  M[0] = mat[0];
  M[1] = mat[3];
  M[2] = mat[6];

  M[3] = mat[1];
  M[4] = mat[4];
  M[5] = mat[7];
  
  M[6] = mat[2];
  M[7] = mat[5];
  M[8] = mat[8];

  return mat3(M);
}

/*! Scales matrix \a m by the scalar \a s. */
mat3
operator*(double const& s,  mat3 const& m)
{
  double M[9];

  M[0] = s*m.mat[0];
  M[1] = s*m.mat[1];
  M[2] = s*m.mat[2];
  M[3] = s*m.mat[3];
  M[4] = s*m.mat[4];
  M[5] = s*m.mat[5];
  M[6] = s*m.mat[6];
  M[7] = s*m.mat[7];
  M[8] = s*m.mat[8];
  return mat3(M);
}

/*! Adds matrix \a m to this matrix. */
mat3 const&
mat3::operator+=(mat3 const& m)
{
  mat[0] += m[0]; mat[1] += m[1]; mat[2] += m[2];
  mat[3] += m[3]; mat[4] += m[4]; mat[5] += m[5];
  mat[6] += m[6]; mat[7] += m[7]; mat[8] += m[8];
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
//                       Quaternion inline functions
///////////////////////////////////////////////////////////////////////////////

/*! Converts a quaternion into an Inventor style SbRotation.  Inventor
    stores their quaternions in a slightly different order, and uses floats.
*/
SbRotation
Quaternion::toSbRotation() const
{
  return SbRotation((float) x,(float) y, (float) z, (float) w);
}

/*! Assignment operator, copies quaternion \a q. */
Quaternion&
Quaternion::operator=(const Quaternion& q)
{
  set(q);
  return *this;
}

/*! Returns the normalized result of adding \a q to this quaternion. */
Quaternion
Quaternion::operator+(const Quaternion& q) const
{
    return Quaternion(w+q.w,x+q.x,y+q.y,z+q.z);
}

/*! Returns the normalized result of subtracting \a q from this quaternion. */
Quaternion
Quaternion::operator-(const Quaternion& q) const
{
    return Quaternion(w-q.w,x-q.x,y-q.y,z-q.z);
}

/*! Returns the normalized result of multiplying this quaternion by \a q. */
Quaternion
Quaternion::operator*(const Quaternion& q) const
{
    return Quaternion(
		w*q.w-x*q.x-y*q.y-z*q.z,
		w*q.x+x*q.w+y*q.z-z*q.y,
		w*q.y+y*q.w+z*q.x-x*q.z,
		w*q.z+z*q.w+x*q.y-y*q.x
    );
}


/*Quaternion
Quaternion::operator*(double c) const
{
    return Quaternion(c*w,c*x,c*y,c*z);
}


Quaternion
operator*(double c, const Quaternion& q)
{
    return Quaternion(c*q.w,c*q.x,c*q.y,c*q.z);
}
*/

/*! Unary negation operator.  Negates each component of the quaternion. */
Quaternion
Quaternion::operator-() const
{
    return Quaternion(-w,-x,-y,-z);
}

/*! Returns the dot product of this quaternion and \a q. */
double
Quaternion::operator%(const Quaternion& q) const
{
    return w*q.w+x*q.x+y*q.y+z*q.z;
}

/*! Returns the norm of the quaternion. */
double
Quaternion::norm() const
{
    return w*w+x*x+y*y+z*z;
}

/*! Returns the inverse quaternion. */
Quaternion
Quaternion::inverse() const
{
    // assert:  'this' is unit length
    return Quaternion(w,-x,-y,-z);
}

/*! Returns the vector \a v after it has been rotated by this quaternion. */
vec3
Quaternion::operator*(const vec3& v) const
{
    // Given a vector u = (x0,y0,z0) and a unit length quaternion
    // q = <w,x,y,z>, the vector v = (x1,y1,z1) which represents the
    // rotation of u by q is v = q*u*q^{-1} where * indicates quaternion
    // multiplication and where u is treated as the quaternion <0,x0,y0,z0>.
    // Note that q^{-1} = <w,-x,-y,-z>, so no real work is required to
    // invert q.  Now
    //
    //   q*u*q^{-1} = q*<0,x0,y0,z0>*q^{-1}
    //     = q*(x0*i+y0*j+z0*k)*q^{-1}
    //     = x0*(q*i*q^{-1})+y0*(q*j*q^{-1})+z0*(q*k*q^{-1})
    //
    // As 3-vectors, q*i*q^{-1}, q*j*q^{-1}, and 2*k*q^{-1} are the columns
    // of the rotation matrix computed in Quaternion::ToRotationMatrix.  The
    // vector v is obtained as the product of that rotation matrix with
    // vector u.  As such, the quaternion representation of a rotation
    // matrix requires less space than the matrix and more time to compute
    // the rotated vector.  Typical space-time tradeoff...

    mat3 R;
    ToRotationMatrix(R);

    vec3 result;
    result[0] = v.x()*R.element(0,0)+v.y()*R.element(1,0)+v.z()*R.element(2,0);
    result[1] = v.x()*R.element(0,1)+v.y()*R.element(1,1)+v.z()*R.element(2,1);
    result[2] = v.x()*R.element(0,2)+v.y()*R.element(1,2)+v.z()*R.element(2,2);

    return result;
}

/*! Returns the position \a p after it has been rotated by this quaternion. */
position
Quaternion::operator*(const position& p) const
{
  mat3 R;
  ToRotationMatrix(R);
  
  position result;
  result[0] = p.x()*R.element(0,0)+p.y()*R.element(1,0)+p.z()*R.element(2,0);
  result[1] = p.x()*R.element(0,1)+p.y()*R.element(1,1)+p.z()*R.element(2,1);
  result[2] = p.x()*R.element(0,2)+p.y()*R.element(1,2)+p.z()*R.element(2,2);
  
  return result;
}

/*! Comparison operator, returns true if the L2 distance between this
    quaternion and \a q is less than resabs.
*/
bool
Quaternion::operator==(Quaternion const& q) const
{
  if ((w-q.w)*(w-q.w)+(x-q.x)*(x-q.x)+(y-q.y)*(y-q.y)+(z-q.z)*(z-q.z) < 
      resabs * resabs)
    return true;
  return false;
}

/*! Normalizes this quaternion. */
void
Quaternion::normalise()
{
  double nm=norm();
  double invnorm;
  if (nm==0.0) {
#ifdef GRASPITDBG
    printf("0 norm in Quaternion normalise\n");
#endif	
	return;
  }
  invnorm =1.0/sqrt(nm);
  w*=invnorm; x*=invnorm; y*=invnorm; z*=invnorm;
}

///////////////////////////////////////////////////////////////////////////////
//                       transf  functions
///////////////////////////////////////////////////////////////////////////////

/*!
  Returns the inverse transformation.
*/
transf
transf::inverse() const
{ 
  return transf(rot.inverse(),-(rot.inverse() * t));
}

/*! Assignment operator, copies transform \a tr. */
transf const&
transf::operator=(transf const& tr)
{
  rot = tr.rotation();
  R = tr.affine();
  t = tr.translation();
  return *this;
}

/*transf const&
transf::operator*=(transf const& tr)
{
  rot = tr.rotation() * rot;
  rot.ToRotationMatrix(R);
  t  += tr.translation + rot * t;
  return *this;
}
*/


/*! Converts this transform to Inventor format. */
void
transf::toSoTransform(SoTransform *IVt) const
{  
  IVt->rotation.setValue(rot.toSbRotation());
  IVt->translation.setValue(t.toSbVec3f());
}
/*! Converts this transform to a column major 4x4 matrix*/
void
transf::toColMajorMatrix(double mat[][4]) const
{
  for (int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      mat[j][i] = R.element(j,i);
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
  for (int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      mat[i][j] = R.element(j,i);
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
  for (int j=0;j<3;j++) {
    for(int i=0;i<3;i++)
      colTran[i][j] = R.element(j,i);
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
  float q1,q2,q3,q4,x,y,z;

  // Inventor stores the quaternion as qx,qy,qz,qw
  IVt->rotation.getValue().getValue(q2,q3,q4,q1);
  IVt->translation.getValue().getValue(x,y,z);  
  rot.w=(double)q1; rot.x=(double)q2; rot.y=(double)q3; rot.z=(double)q4;
  t[0]=(double)x; t[1]=(double)y; t[2]=(double)z;
  rot.ToRotationMatrix(R);
}

/*! Multiplies two transforms.  */
transf
operator*(const transf& tr2,  const transf& tr1)
{
  Quaternion newRot = tr1.rotation() * tr2.rotation();
  return transf(newRot, tr1.translation() + tr1.rotation() * tr2.translation());
}

/*!
  Returns a vector that is the result of rotating vector \a v using transform
  \a tr.
 */
vec3
operator*(const vec3& v, const transf& tr)
{
  return tr.rot * v;
}
/*returns the vector under transformation tr*/
vec3
operator>(const vec3& v, const transf& tr)
{
	return tr.rot*v + tr.t;
}


/*!
  Returns the result of transforming position \a p with transform \a tr.
 */
position
operator*(const position& p, const transf& tr)
{
  return tr.rot * p + tr.t;
}

/*!
  Compares two transforms.  Returns true if both the rotations and translations
  are equal (according to their comparators).
*/
bool
transf::operator==(transf const& tr) const
{
  if (tr.rot == rot && tr.t == t)
    return true;
  return false;
}

/*! 
  Helper function that creates a transform with an identity rotation and
  a translation equal to vector \a v.
*/
transf
translate_transf(const vec3& v)
{
  return transf(Quaternion::IDENTITY,v);
}

/*!
  Helper function that creates a tranform that has a rotation of \a angle
  radians about \a axis, and a zero translation.
*/
transf
rotate_transf(double angle, const vec3& axis)
{
  return transf(Quaternion(angle,axis),vec3::ZERO);
}

/*!
  Helper function that creates a tranform from the global frame to the
  new frame with its origin at position \a o, and with the given x and y axes.
*/
transf
coordinate_transf(const position& o, const vec3& xaxis, const vec3& yaxis)
{ 
  vec3 newxaxis = normalise(xaxis);
  vec3 newzaxis = normalise(newxaxis * yaxis);
  vec3 newyaxis = normalise(newzaxis * newxaxis);
  return transf(mat3(newxaxis,newyaxis,newzaxis),o - position::ORIGIN);
}

transf
rotXYZ(double rx, double ry, double rz)
{
	transf r;
	vec3 x(1,0,0), y(0,1,0), z(0,0,1);
	
	r = rotate_transf(rx, x);
	y = y * r.inverse();
	r = rotate_transf(ry, y) * r;
	z = z * r.inverse();
	r = rotate_transf(rz, z) * r;

	return r;
}

/*! Given two line segments, P1-P2 and P3-P4, returns the line segment 
	Pa-Pb that is the shortest route between them. Calculates also the 
	values of \a mua and \a mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Returns FALSE if no solution exists.
   adapted from http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
*/
inline int LineLineIntersect(vec3 p1,vec3 p2,vec3 p3,vec3 p4,vec3 *pa,vec3 *pb,double *mua, double *mub)
{
   vec3 p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;
   double EPS = 1.0e-5;

   p13.x() = p1.x() - p3.x();
   p13.y() = p1.y() - p3.y();
   p13.z() = p1.z() - p3.z();
   p43.x() = p4.x() - p3.x();
   p43.y() = p4.y() - p3.y();
   p43.z() = p4.z() - p3.z();
   if ( fabs(p43.x())  < EPS && fabs(p43.y())  < EPS && fabs(p43.z())  < EPS )
      return false;

   p21.x() = p2.x() - p1.x();
   p21.y() = p2.y() - p1.y();
   p21.z() = p2.z() - p1.z();
   if ( fabs(p21.x())  < EPS && fabs(p21.y())  < EPS && fabs(p21.z())  < EPS )
      return false;

   d1343 = p13.x() * p43.x() + p13.y() * p43.y() + p13.z() * p43.z();
   d4321 = p43.x() * p21.x() + p43.y() * p21.y() + p43.z() * p21.z();
   d1321 = p13.x() * p21.x() + p13.y() * p21.y() + p13.z() * p21.z();
   d4343 = p43.x() * p43.x() + p43.y() * p43.y() + p43.z() * p43.z();
   d2121 = p21.x() * p21.x() + p21.y() * p21.y() + p21.z() * p21.z();

   denom = d2121 * d4343 - d4321 * d4321;
   if ( fabs(denom) < EPS )
      return false;
   numer = d1343 * d4321 - d1321 * d4343;

   *mua = numer / denom;
   *mub = (d1343 + d4321 * (*mua)) / d4343;

   pa->x() = p1.x() + (*mua) * p21.x();
   pa->y() = p1.y() + (*mua) * p21.y();
   pa->z() = p1.z() + (*mua) * p21.z();
   pb->x() = p3.x() + (*mub) * p43.x();
   pb->y() = p3.y() + (*mub) * p43.y();
   pb->z() = p3.z() + (*mub) * p43.z();

   return true;
}

#define _MATVEC_H_
#endif


