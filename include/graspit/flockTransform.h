#ifndef _FLOCKTRANSFORM_H_

#include "graspit/matvec3D.h"

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
class FlockTransf {
  private:
    transf mount, mountInv;
    transf flockBaseInv, objBase;

  public:
    void identity();
    //! Set the tranform from object origin to sensor mounting location
    void setMount(transf t) {mount = t; mountInv = t.inverse();}
    //! Set the base flock transform, that all subsequent flock transforms are relative to
    void setFlockBase(transf t) {flockBaseInv = t.inverse();}
    //! Set the base object transform, the one that corresponds to the base flock trasnform
    void setObjectBase(transf t) {objBase = t;}
    //! Get the object's GraspIt transform for a give flock transform read from the sensor
    transf get(transf t) const;
    //! Get the object's GraspIt transform if we are operating in absolute terms
    transf getAbsolute(transf t) const;
    //! Get the object transform in relative terms, but without factoring in a mount transform
    transf get2(transf t) const;
    transf getMount() const {return mount;}
};

#define _FLOCKTRANSFORM_H_
#endif
