#include "graspitParser.h"

const std::string GraspitParser::usage = "Usage: graspit [-w worldname] [-r robotname] [-o objectname] [-b obstaclename] [-p pluginname]";

const std::string GraspitParser::version =
        "GraspIt!\n"
        "Copyright (C) 2002-2009  Columbia University in the City of New York.\n"
        "All rights reserved.\n"
        "\n"
        "GraspIt! is free software: you can redistribute it and/or modify\n"
        "it under the terms of the GNU General Public License as published by\n"
        "the Free Software Foundation, either version 3 of the License, or\n"
        "(at your option) any later version.\n"
        "\n"
        "GraspIt! is distributed in the hope that it will be useful,\n"
        "but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
        "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
        "GNU General Public License for more details.\n"
        "\n"
        "You should have received a copy of the GNU General Public License\n"
        "along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.\n";

const std::string GraspitParser::description =
         "Possible valid arguments\n"
         "graspit -w plannerMug\n"
         "graspit -b table -r Barrett\n"
         "graspit -b table -r Barrett -p helloworld\n";

const std::string GraspitParser::epilog = "Please remember to set the GRASPIT environment variable to a valid directory "
         "normally the source directory where graspit was installed.  On Linux this can be done by navigating to the correct "
         "directory and running \n\"export GRASPIT=$PWD\"\nIf your are using plugins, then remember to set GRASPIT_PLUGIN_DIR as well.\n\n";

const std::string GraspitParser::plugin_help = "Name of plugin to load, multiple plugins may be loaded by multiple -p args.\n"
         "Ex: graspit -p helloworld\n"
         "would load the helloworld plugin";

const std::string GraspitParser::world_help =
         "World file to load from $GRASPIT/worlds.\n"
         "Ex: --world myworld\n"
         "would load $GRASPIT/worlds/myworld.xml";

const std::string GraspitParser::object_help = "Name of object to load from $GRASPIT/models/objects/ imported as a GraspableBody\n"
         "Ex: graspit -o mug\n"
         "would load $GRASPIT/models/objects/mug.xml";

const std::string GraspitParser::obstacle_help = "Name of obstacle to load from $GRASPIT/models/obstacles/, imported as a Body.\n"
        "Ex: graspit -b table\n"
        "would load $GRASPIT/models/obstacles/table.xml";

const std::string GraspitParser::robot_help = "Name of robot to load, from $GRASPIT/models/robots.  The robot directory must have an xml file of the same name"
         "as the folder.  For example $GRASPIT/models/robots/Barrett has a matching Barrett.xml file inside of it.\n"
         "Ex: graspit -r Barrett\n"
         "would load $GRASPIT/models/robots/Barrett/Barrett.xml";

GraspitParser::GraspitParser()
{
     parser = optparse::OptionParser()
             .usage(usage)
             .version(version)
             .description(description)
             .epilog(epilog);

    parser.add_option("-p", "--plugin").action("append").help(plugin_help);
    parser.add_option("-w", "--world").help(world_help);
    parser.add_option("-o", "--object").help(object_help);
    parser.add_option("-b", "--obstacle").help(obstacle_help);
    parser.add_option("-r", "--robot").help(robot_help);
}

Values& GraspitParser::parseArgs(int argc, char *argv[])
{
    return  parser.parse_args(argc, argv);
}


