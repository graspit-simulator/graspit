
#ifndef SOFT_CONTACT_H
#define SOFT_CONTACT_H

#include "graspit/matvec3D.h"
#include "graspit/contact/contact.h"

class Body;
class SoSeparator;

//! Soft Contact implements an SFC model for contacts between soft bodies
/*! The SoftContact attempts to capture some of the frictional effects between
    soft bodies without explicitly computing the deformation at the contacts.
    It locally fits analytical surfaces to both bodies involved by computing
    their local radii of curvature. After this, it uses analytical models
    to compute the expected contact area and pressure distribution.

    Based on the contact area, it then tries to approximate the relationship
    between tangential friction and frictional torque using the limit surface,
    which becomes a 3D friction ellipsoid. The friction space is thus 3D, and
    the wrench space becomes 4D.

    Its visual indicator shows the analytical surface that has been fit to both
    bodies involved, as a small patch around the contact location.
*/
class SoftContact : public Contact
{
  protected:
    //! A list of points from body1 that surround the contact in the frame of body1
    vec3 *bodyNghbd;

    //! The number of points in the current fit for analytical surfaces
    int numPts;

    //! Second order fit of points from body1 in the form r1x^2 + r2y^2
    double r1, r2;
    //! Parameters of unrotated fit in form z= ax^2+by^2+cxy
    double a, b, c;

    //! The relative angle between r1 and the mate's r1
    double relPhi;

    //! The rotation from the contact frame to the frame in which the fit is given by the radii of curvature, r1 and r2
    mat3 fitRot;

    //! The angle of the fitRot rotation
    double fitRotAngle;

    //! The major axis of the ellipse of contact
    double majorAxis;
    //! The minor axis of the ellipse of contact
    double minorAxis;

    //! The relative radii of curvature of the two bodies
    double r1prime;
    //! The relative radii of curvature of the two bodies
    double r2prime;

    //calculates the relative angle between the frames with 2 radii of curvature between
    //the contact and its mate
    int CalcRelPhi();

    //! Calculate the relative curvatures of the two bodies for contact dynamics
    int CalcRprimes();

    //! Fits an analytical surface to a local patch on body1 around the contact
    void FitPoints();

    //! Calculates friction characteristics using a Mattress model
    double CalcContact_Mattress(double nForce);

  public:
    //! Also takes a local neighborhood of points around body 1
    SoftContact(Body *b1, Body *b2, position pos, vec3 norm, Neighborhood *bn);

    //! Deletes its local record of the body neighborhood
    ~SoftContact();

    //! Friction model is a 3D friction ellipsoid also containing frictional torque
    int setUpFrictionEdges(bool dynamicsOn = false);

    //! Visual indicator is a small patch of the fit analytical surface on the body
    SoSeparator *getVisualIndicator();

    //! Also attempt to apply some torques in the contact plane; currently disabled.
    virtual void computeWrenches();
};

#endif
