//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: worldElement.h,v 1.17 2009/06/18 20:41:02 saint Exp $
//
//######################################################################


#ifndef WORLD_ELEMENT_FACTORY_H
#define WORLD_ELEMENT_FACTORY_H

#include <string>
#include <map>

class World;
class WorldElement;

//! Functor interface for creating world elements
class WorldElementCreator
{
public:
  virtual WorldElement* operator() (World* parent, const char* name) = 0;
};

//! Templated implementation
template <class E>
class SimpleWorldElementCreator : public WorldElementCreator
{
public:
  virtual WorldElement* operator() (World* parent, const char* name)
  {
    return new E(parent, name);
  }
};

//! Factory class for creating WorldElements
/*! Used to instantiate the right WorldElement based on a name, presumably read
  from an .xml configuration file, such as a saved world or a robot definition.
*/
class WorldElementFactory 
{
private:
  //! Maps world element types to their creators
  std::map<std::string, WorldElementCreator*> mCreators;

public:  
  //! Stub constructor
  WorldElementFactory(){}

  //! Cleans up, deletes all creators
  ~WorldElementFactory();

  //! Instantiates an element based on the given type
  WorldElement* createElement(std::string elementType, World *parent,const char *name);

  //! Registers a new world element creator for the given type
  void registerCreator(std::string elementType, WorldElementCreator *creator);

  //! Registers creators for the built in element types. Called from the world constructor
  static void registerBuiltinCreators();
};

//! Returns the world element factory singleton
inline WorldElementFactory& getWorldElementFactory()
{
  static WorldElementFactory wef;
  return wef;
}

//! Convenience macro for registering a new class
#define REGISTER_CREATOR(STRINGNAME,CLASSNAME) getWorldElementFactory().registerCreator( STRINGNAME, new SimpleWorldElementCreator<CLASSNAME>() );

#endif
