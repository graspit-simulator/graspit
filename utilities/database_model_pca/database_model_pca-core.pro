
QT += qt3support sql
CONFIG += qt warn_on exceptions 

DESTDIR = bin

INCLUDEPATH += $(QTDIR)/include $(GRASPIT)/src/DBase/  $(GRASPIT)/src/DBase/DBPlanner/ include $(GRASPIT)/include


openmesh{
INCLUDEPATH += $(OPENMESH) 
HEADERS += include/openmesh.h include/openmesh_meshHandler.h
SOURCES += src/openmesh.cpp src/openmesh_meshHandler.cpp
DEFINES += USE_OPENMESH
}  
  
opencv{
SOURCES +=  src/opencv_pcaFunctor.cpp
HEADERS +=   include/opencv_pcaFunctor.h
 OPENCVVERSIONNUM = 210
INCLUDEPATH += $(OPENCV2)/include/
DEFINES += USE_OPENCV

}

!exists($(GRASPIT)){
                      error("GRASPIT path not set")
}

HEADERS += $(GRASPIT)/src/DBase/DBPlanner/database.h\
           $(GRASPIT)/src/DBase/DBPlanner/db_manager.h\
           $(GRASPIT)/include/matvec3D.h\
           $(GRASPIT)/src/DBase/DBPlanner/sql_database_manager.h\
           include/meshHandler.h\
include/pcaFunctor.h\
include/iv_to_openmesh.h

SOURCES += $(GRASPIT)/src/DBase/DBPlanner/database.cpp\
           $(GRASPIT)/src/DBase/DBPlanner/sql_database_manager.cpp\
           src/main_pipeline.cpp src/iv_to_openmesh.cpp\

win32{
include (database_model_pca-lib-WINDOWS.pro)
}else{
include (database_model_pca-lib-LINUX.pro)
}
