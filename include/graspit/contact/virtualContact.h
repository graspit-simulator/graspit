#ifndef VIRTUAL_CONTACT_H
#define VIRTUAL_CONTACT_H

#include "graspit/matvec3D.h"
#include "graspit/contact/contact.h"

class Body;
class SoSeparator;
class SoMaterial;
class TiXmlElement;
class QString;

//! A contact that exists even when a hand is not perfectly touching another object
/*! This class is meant for studing grasp quality when there is really no grasp, as
    the hand is not exactly touching an object. The high-level purpose is to convert
    the grasp quality function to a continous function that exists in all hand
    configurations, rather then a highly discrete function that only is non-zero
    when the hand is touching an object in the right way. However, this line of
    work is not completed and this class is in bad need of an overhaul.

    A virtual contact does NOT come in a pair - there is no mate. As such, any instance
    of this will have the mate set to NULL, a distinct sign of a virtual contact.
    Also, virtual contacts have the normals pointing outward, instead of the usual
    contacts whose normals point inward. This is confusing and should probably be
    changed at some point.

    The VirtualContact does not always need a body2. In fact, most of its design can
    work if the body2 is NULL. However, it can have a body2, in which case some
    distance functions are computed based on the body2. This is not very well
    implemented right now and subject to change.

    A virtual contact can be instantiated from any kind of contact as it pretty much
    only copies the friction edges and location. It also needs a separate
    computeWrenches to eliminate the need for the object.

    All computations for a virtual contact (like wrenches, etc) are performed in the
    world coordinate system, unlike the usual contact which is in the object's
    coordinate system. The reason is that we can have a grasp defined my many virtual
    contacts on the hand (rather then meny traditional contacts on an object). In this
    case, each link of the hand has its own coordinate frame, so in order to have a
    coherent frame for the grasp we use world coordinates.
*/
class VirtualContact : public Contact
{
  private:
    //! Meant to replace the object CoG when a virtual contact has no object
    position mCenter;
    //! Meant to replace the object max radius when a virtual contact has no object
    float mMaxRadius;

    //! We can hook this up directly to the world root to see the contact; for debug purposes
    SoSeparator *mWorldInd;
    //! We keep a pointer so we can change this contact's appearance on the fly; for debug
    SoMaterial *mZaxisMat;

    //! Which finger (kinematic chain) of the robot this virtual contact is on. -1 means the palm.
    int mFingerNum;
    //! Which link of the chain this virtual contact is on.
    int mLinkNum;

    //! Empty contact initialization shared by all constructors; sets the contact as its own mate
    void init();
  public:

    //! The constructor initializes an empty virtual contact, setting it as its own mate
    VirtualContact();
    //! Also flips the normal of the original contact, to make it point outward
    VirtualContact(int f, int l, Contact *original);
    //! Copy constructor copies friction edges and location
    VirtualContact(const VirtualContact *original);
    //! Destructor
    ~VirtualContact();

    //! Sets the body that this contact is on, presumably to a robot link
    void setBody(Body *b) {body1 = b;}
    //! Sets an object that this contact is not exactly on, but we use in calculations.
    void setObject(Body *b) {body2 = b;}

    //! Writes this contact, including friction edges, to a file
    // Returns false if file cannot be opened for writing
    void writeToFile(std::ofstream &outfile);
    //! Loads this contact, including friction edges, from a file
    // Returns false if file cannot be opened for reading
    bool readFromFile(std::ifstream &infile);
    int loadFromXml(const TiXmlElement *root);

    //! Wrench computation is done in world coordinates considers the fact that we have no object
    void computeWrenches(bool useObjectData = false, bool simply = false);

    //! Scales the wrench space of this contact by a given factor
    void scaleWrenches(double factor);

    //! The virtual contact can not set up its own friction edges.
    int setUpFrictionEdges(bool dynamicsOn = false);

    //! Updates mObjectDistance and mObjectNormal based on current world locations.
    void updateContact(Body *object);

    //! Sets the max radius that is to be used in wrench computations
    void setRadius(float r) {mMaxRadius = r;}
    //! Sets the c.o.g that is to be used in wrench computations
    void setCenter(position c) {mCenter = c;}
    //! Gets the max radius that is used in wrench computations
    double getMaxRadius() {return mMaxRadius;}
    //! Gets the center that is used in wrench computations
    position getCenter() {return mCenter;}

    //! The visual indicator is just a thin red cylinder
    SoSeparator *getVisualIndicator();
    //! Another indicator that can be atatched directly to the world root, for debug
    void getWorldIndicator(bool useObjectData = false);

    //! The location of this contact in world coordinates
    position getWorldLocation();
    //! The normal of this contact (pointing outwards) in world coordinates
    vec3 getWorldNormal();
    //! Computes the distance to the object along the contact normal and the object surface normal at the closest point
    void getObjectDistanceAndNormal(Body *body, vec3 *objDistance, vec3 *objNormal);

    //! Changes the visual marker to show that this contact is marked; for debug purposes.
    void mark(bool m);

    //! Returns the number of the finger that this contact is on
    int getFingerNum() {return mFingerNum;}
    //! Returns the number of the link that this contact is on
    int getLinkNum() {return mLinkNum;}

    //! Changes the frame (and thus also the location and normal) of this virtual contact
    void changeFrame(transf tr);
};

#endif
