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
// Author(s):  Jennifer Buehler 
//
//######################################################################

/*! \file 
  \brief Defines the IVmgrHeadless class which acts like the original IVmgrHeadless (handling 3D user interaction), but bypasses all GUI related actions.
 */
#ifndef IVMGR_HEADLESS_HXX
#include <Inventor/SbBasic.h>
#include <Inventor/SbLinear.h>
#include <QWidget>
#include <list>
#include <vector>
#include <qstring.h>
#include "material.h"
#include "ivmgrBase.h"

class World;
class QWidget;
class SoSeparator;

//! Handles the inventor files and QT dependencies without requiring a GUI
class IVmgrHeadless: public IVmgrBase {

private:

  //! Points to the main world associated with this iteraction manager
  World *world;

  //! Pointer to the root of the entire Inventor scene graph
  SoSeparator *sceneRoot;

  // QT object which is required at the very minimum to support all Qt operations.
  // This does not have to be displayed however.
  QWidget * qapp;
  
  // Initializes the Qt environment. However, the main loop still has to be started manually with beginMainLoop().
  void initSoQt();

public:
  /**
   * Initializes Inventor/Qt environment (for use without Gui) and creates an empty world with given name, which
   * is going to be used as the main world.
   */
  IVmgrHeadless(const char * name);
  ~IVmgrHeadless();

  /*! 
    Returns a pointer to the main World that the user interacts with through
    this manager.
   */
  World *getWorld() const {return world;}

  SoSeparator * getSceneRoot() { return sceneRoot; } 

  /**!
    Creates a new world object as the main world. 
   */
  void createNewWorld(const char* name);

  void emptyWorld();

  void beginMainLoop();
  void exitMainLoop();

};
#define IVMGR_HEADLESS_HXX
#endif
