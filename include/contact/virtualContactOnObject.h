#ifndef VIRTUAL_CONTACT_ON_OBJECT_H
#define VIRTUAL_CONTACT_ON_OBJECT_H

#include "matvec3D.h"
#include "virtualContact.h"

/*! Similar to the VirtualContact class
it changes the virtual contact's location from the finger to an object imported before with these virtual contact
*/
class VirtualContactOnObject : public VirtualContact
{
public:
    VirtualContactOnObject ();
    ~VirtualContactOnObject ();
    bool readFromFile(std::ifstream& infile);
#ifdef ARIZONA_PROJECT_ENABLED
    void readFromRawData(ArizonaRawExp* are, QString file, int index, bool flipNormal = false);
#endif
    void writeToFile(std::ofstream& outfile);
};

#endif
