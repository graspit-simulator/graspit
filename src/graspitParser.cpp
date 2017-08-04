#include "graspit/graspitParser.h"

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

const std::string GraspitParser::footer =
  "\n\n"
  "Environment Variables:\n"
  "GRASPIT\n"
  "\t models, and worlds are loaded relative to this path,\n"
  "\t normally should be set to directory containing the \n"
  "\t subfolders worlds and models\n"
  "GRASPIT_PLUGIN_DIR\n"
  "\t point to director containined plugin libs, if using ros plugins\n"
  "\t then normally should be set to my_ros_wkspace/devel/lib\n";

const std::string GraspitParser::help_help =
  "Print this message\n";

const std::string GraspitParser::plugin_help =
  "Name of plugin to load, multiple plugins may be loaded"
  "\n\t\t\t seperated by comma."
  "\n\t\t\t Ex: graspit -p plugin1,plugin2"
  "\n\t\t\t would load plugin1 and plugin2"
  "\n\t\t\t";

const std::string GraspitParser::world_help =
  "World file to load from $GRASPIT/worlds."
  "\n\t\t\t Ex: --world myworld"
  "\n\t\t\t would load $GRASPIT/worlds/myworld.xml"
  "\n\t\t\t";

const std::string GraspitParser::object_help =
  "Name of object to load from $GRASPIT/models/objects/"
  "\n\t\t\t imported as a GraspableBody"
  "\n\t\t\t Ex: graspit -o mug"
  "\n\t\t\t would load $GRASPIT/models/objects/mug.xml"
  "\n\t\t\t";

const std::string GraspitParser::obstacle_help =
  "Name of obstacle to load from $GRASPIT/models/obstacles/,"
  "\n\t\t\t imported as a Body."
  "\n\t\t\t Ex: graspit -o table"
  "\n\t\t\t would load $GRASPIT/models/obstacles/table.xml"
  "\n\t\t\t";

const std::string GraspitParser::robot_help =
  "Name of robot to load, from $GRASPIT/models/robots."
  "\n\t\t\t The robot directory must have an xml file of the same name"
  "\n\t\t\t as the folder.  For example $GRASPIT/models/robots/Barrett "
  "\n\t\t\t has a matching Barrett.xml file inside of it."
  "\n\t\t\t Ex: graspit -r Barrett"
  "\n\t\t\t would load $GRASPIT/models/robots/Barrett/Barrett.xml"
  "\n\t\t\t";

const std::string GraspitParser::version_help =
  "print GraspIt! version\n";

const std::string GraspitParser::headless_help =
  "Run GraspIt! in headless mode if you do not want"
  "\n\t\t\t the user interface to display.  IVMgr will be NULL";

GraspitParser::GraspitParser()
{

  bool is_required_arg = false;

  parser = new cmdline::parser();

  parser->add("help", 'h', help_help);
  parser->add<std::string>("plugin", 'p', plugin_help,  is_required_arg);
  parser->add<std::string>("world", 'w', world_help,  is_required_arg);
  parser->add<std::string>("object", 'o', object_help,  is_required_arg);
  parser->add<std::string>("obstacle", 'b', obstacle_help,  is_required_arg);
  parser->add<std::string>("robot", 'r', robot_help,  is_required_arg);
  parser->add("headless", '\0', headless_help);
  parser->add("version", 'v', version_help);

  parser->footer(footer);

}

GraspitParser::~GraspitParser()
{
  delete parser;
}

cmdline::parser *GraspitParser::parseArgs(int argc, char *argv[])
{
  parser->parse(argc, argv);

  if (parser->exist("help"))
  {
    std::cerr << parser->usage();
    exit(0);
  }
  if (parser->exist("version"))
  {
    std::cerr << version << std::endl;
    exit(0);
  }

  return  parser;
}


