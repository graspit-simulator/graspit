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
// Author(s): Iretiayo Akinola, Jake Varley
//
//######################################################################

#ifndef _searchenergyfactory_h_
#define _searchenergyfactory_h_

#include <string>
#include <map>
#include <vector>

class SearchEnergy;

//! Functor interface for creating world elements
class SearchEnergyCreator
{
  public:
    virtual SearchEnergy *operator()() = 0;
    virtual ~SearchEnergyCreator() {};
};

//! Templated implementation
template <class E>
class SimpleSearchEnergyCreator : public SearchEnergyCreator
{
  public:
    virtual SearchEnergy *operator()()
    {
      return new E();
    }
};


class SearchEnergyFactory
{
  public:

    static SearchEnergyFactory* getInstance(){
        if(!searchEnergyFactory) {
            searchEnergyFactory = new SearchEnergyFactory();
        }
        return searchEnergyFactory;
    }

    SearchEnergy* createEnergy(std::string energyType) {
        std::map<std::string, SearchEnergyCreator *>::iterator it = mCreators.find(energyType);
        if (it == mCreators.end()) { return NULL; }
        return (*(it->second))();
        }

    void registerCreator(std::string energyType, SearchEnergyCreator *creator) {
        mCreators[energyType] = creator;
        }

    //! Registers creators for the built in element types. Called from the world constructor
    static void registerBuiltinCreators();

    std::vector<std::string> getAllRegisteredEnergy() {
      std::vector<std::string> registeredEnergies;
      for(std::map<std::string, SearchEnergyCreator *> ::const_iterator it = mCreators.begin(); it != mCreators.end(); it++)
      {
          registeredEnergies.push_back(it->first);
      }
      return registeredEnergies;
    }

private:
    std::map<std::string, SearchEnergyCreator *> mCreators;
    static SearchEnergyFactory * searchEnergyFactory;
    SearchEnergyFactory() {}

};


//! Convenience macro for registering a new class
#define REGISTER_SEARCH_ENERGY_CREATOR(STRINGNAME,CLASSNAME) SearchEnergyFactory::getInstance()->registerCreator( STRINGNAME, new SimpleSearchEnergyCreator<CLASSNAME>() );


#endif
