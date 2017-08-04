#include "graspit/flockTransform.h"


void FlockTransf::identity()
{
  mount = transf::IDENTITY;
  mountInv = transf::IDENTITY;
  flockBaseInv = transf::IDENTITY;
  objBase = transf::IDENTITY;
}

transf FlockTransf::get(transf t) const
{
  return objBase % mount % flockBaseInv % t % mountInv;
}

transf FlockTransf::getAbsolute(transf t) const
{
  return t % mountInv;
}

/*! Does not multiply with the mount transform. It is meant to use with
  camera, and we assume we already have the camera mount built into the
  objectBase.
*/
transf FlockTransf::get2(transf t) const
{
  return objBase % flockBaseInv % t % mountInv;
}
