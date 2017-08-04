#ifndef VIRTUAL_CONTACT_ON_OBJECT_H
#define VIRTUAL_CONTACT_ON_OBJECT_H

#include "graspit/matvec3D.h"
#include "graspit/contact/virtualContact.h"

/*! Similar to the VirtualContact class
it changes the virtual contact's location from the finger to an object imported before with these virtual contact
*/
class VirtualContactOnObject : public VirtualContact
{
  public:
    VirtualContactOnObject();
    ~VirtualContactOnObject();
    bool readFromFile(std::ifstream &infile);
    void writeToFile(std::ofstream &outfile);
};

#endif
