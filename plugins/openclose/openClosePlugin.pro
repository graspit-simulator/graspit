TEMPLATE	= lib
LANGUAGE	= C++
CONFIG += qt plugin 
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

!exists($(GRASPIT)) {
   error("GRASPIT environment variable not set")
}

INCLUDEPATH += $(GRASPIT)/include $(GRASPIT)/src $(GRASPIT)/src/Collision $(GRASPIT)/ui

HEADERS += openClosePlugin.h 

SOURCES += openClosePlugin.cpp

