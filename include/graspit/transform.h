#ifndef _TRANSFORM_H_

#define _TRANSFORM_H_
#include "graspit/matvec3D.h"

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
    transf() {R = mat3::Identity(); t = vec3::Zero(); rot = Quaternion::Identity();}

    /*! Initializes transform with a rotation quaternion and translation vector*/
    transf(const Quaternion &r, const vec3 &d) {set(r, d);}

    /*! Initializes transform with a rotation matrix and translation vector */
    transf(const mat3 &r, const vec3 &d) {set(r, d);}

    /*! Initializes transform using an Inventor transform. */
    transf(const SoTransform *IVt) {set(IVt);}

    /*!
     * Factory Methods for creating transf instances
    */
    static transf TRANSLATION(const vec3 &v);
    static transf AXIS_ANGLE_ROTATION(double angle, const vec3 &axis);
    static transf COORDINATE(const position &o, const vec3 &xaxis, const vec3 &yaxis);
    static transf RPY(double rx, double ry, double rz);

    /*! Sets value of the transform with the rotation \a r and the translation
      \a d */
    void set(const Quaternion &r, const vec3 &d) {
      rot = r.normalized(); t = d; R = rot.toRotationMatrix();
    }

    /*! Sets value of the transform with the rotation \a r and the translation
      \a d */
    void set(const mat3 &r, const vec3 &d)
    {
      rot = r;
      rot.normalize();
      t = d; R = r;
    }

    void set(const SoTransform *IVt);

    /*! Returns the rotation portion of the transform. */
    const Quaternion &rotation() const {return rot;}

    /*! Returns the rotation portion of the transform as a 3x3 matrix. */
    const mat3 &affine() const {return R;}

    /*! Returns the translation portion of the transform. */
    const vec3 &translation() const {return t;}

    void toSoTransform(SoTransform *IVt) const;
    void tocol_Mat4(col_Mat4 colTran) const;
    void toColMajorMatrix(double t[][4]) const;
    void toRowMajorMatrix(double t[][4]) const;
    //! Creates a 6x6 jacobian from this transform
    void jacobian(double jac[]);

    transf          inverse() const;

    transf const   &operator=(transf const &tr);
    bool operator==(transf const &tr) const;

    /*! Returns the negation of the == operator. */
    bool operator!=(transf const &tr) const {return !operator==(tr);}

    friend std::istream &operator>>(std::istream &is, transf &tr);
    friend std::ostream &operator<<(std::ostream &os, const transf &tr);
    friend transf operator%(const transf &tr1, const transf &tr2);
    friend vec3 operator*(const transf &tr1, const vec3 &v);

    static const transf IDENTITY;

};

#endif
