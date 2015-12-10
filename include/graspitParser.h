
#include "OptionParser.h"

using namespace std;
using namespace optparse;

class GraspitParser

{

public:
    GraspitParser();
    Values& parseArgs(int argc, char *argv[]);

    static const std::string usage;
    static const std::string version;
    static const std::string description;
    static const std::string epilog;
    static const std::string plugin_help;
    static const std::string world_help;
    static const std::string object_help;
    static const std::string obstacle_help;
    static const std::string robot_help;

private:
     optparse::OptionParser parser;
};
