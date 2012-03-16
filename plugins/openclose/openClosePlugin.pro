
win32{
TEMPLATE	= vclib
}else{
TEMPLATE = lib
}

LANGUAGE	= C++
CONFIG += qt dylib
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

!exists($(GRASPIT)) {
   error("GRASPIT environment variable not set")
}

DEFINES += COIN_DLL


INCLUDEPATH += $(QTDIR)/include $(COINDIR)/include qhull $$ADDITIONAL_INCLUDE_DIR

INCLUDEPATH += $(GRASPIT)/src $(GRASPIT)/src/Collision $(GRASPIT)/include $(GRASPIT)/include/math $(GRASPIT)/include/Planner $(GRASPIT)/include/EGPlanner $(GRASPIT)/ui $(GRASPIT)/ui/Planner $(GRASPIT)/ui/EGPlanner


DEPENDPATH += $(GRASPIT)/src $(GRASPIT)/src/Collision include $(GRASPIT)/include/math $(GRASPIT)/include/Planner $(GRASPIT)/include/EGPlanner $(GRASPIT)/ui $(GRASPIT)/ui/Planner $(GRASPIT)/ui/EGPlanner


HEADERS += openClosePlugin.h $(GRASPIT)/include/plugin.h

SOURCES += openClosePlugin.cpp


#in order to get symbols defined in the main executable 
#(most notably GraspitGUI * graspItGUI), you need to link against the executable
win32{
  LIBS += $(GRASPIT)/bin/graspit.lib
}
