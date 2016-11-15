
#to build as a make file in windows, change vclib to lib
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


INCLUDEPATH += $(GRASPIT)/include graspit/plugins/helloworld

DEPENDPATH += $(GRASPIT)/src 

HEADERS += helloWorldPlugin.h $(GRASPIT)/include/plugin.h

SOURCES += helloWorldPlugin.cpp

