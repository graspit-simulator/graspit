
#ifndef POINT_CONTACT_H
#define POINT_CONTACT_H

#include "graspit/matvec3D.h"
#include "graspit/contact/contact.h"

class Body;
class SoSeparator;

//! A Point Contact With Friction (PCWF) implementing a Coulomb friction model
/*! The Point Contact simulates rigid bodies in contact. As such, its wrench
    space is a 3D cone and its friction space is a 2D circle with just
    tangential friction. Its visual indicator is the cone itself. The cone's
    angle is equal to the tangent of the friction coefficient.
*/
class PointContact : public Contact
{
  public:
    //! Also sets up friction edges according to Coulomb model
    PointContact(Body *b1, Body *b2, position pos, vec3 norm);
    //! Stub destructor
    ~PointContact();
    //! Defines a 2D friction circle with tangential friction only.
    int setUpFrictionEdges(bool dynamicsOn = false);
    //! Returns the visual indicator which is the wrench space cone itself
    SoSeparator *getVisualIndicator();
};

#endif
