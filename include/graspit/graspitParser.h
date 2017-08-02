
#include "graspit/cmdline/cmdline.h"

class GraspitParser

{

  public:
    GraspitParser();
    ~GraspitParser();
    cmdline::parser *parseArgs(int argc, char *argv[]);

    static const std::string usage;
    static const std::string version;
    static const std::string footer;
    static const std::string help_help;
    static const std::string plugin_help;
    static const std::string world_help;
    static const std::string object_help;
    static const std::string obstacle_help;
    static const std::string robot_help;
    static const std::string version_help;
    static const std::string headless_help;

  private:
    cmdline::parser *parser;
};
