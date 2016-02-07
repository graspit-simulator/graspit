    # Linux-specific libraries for GraspIt!. Included from graspit.pro - not for standalone use.

LIBS += $$ADDITIONAL_LINK_FLAGS

# ---------------------- Blas and Lapack ----------------------------------

LIBS += -lblas -llapack 

HEADERS += include/lapack_wrappers.h


# ---------------------- General libraries and utilities ----------------------------------

#dynamic linking library
LIBS += -ldl

#add qhull include dir
INCLUDEPATH += /usr/include/qhull

#add qhull libraries
LIBS	+= -lqhull 

#add openinventor libraries
LIBS	+= -lSoQt -lCoin

#add utility libraries
LIBS += -lGL -lpthread

MOC_DIR = .moc
OBJECTS_DIR = .obj

#needed for plugins to find symbols in the main program
QMAKE_LFLAGS += -rdynamic

#------------------------------------ add-ons --------------------------------------------

graspit_test{
    INCLUDEPATH += test/
    SOURCES += test/simple_test.cpp
    LIBS += -L/usr/lib/ -lgtest
    TARGET = graspit-test
}

cgdb {
        graspit_ros {        
                SOURCES += src/DBase/DBPlanner/ros_database_manager.cpp
                HEADERS += src/DBase/DBPlanner/ros_database_manager.h
                
                DEFINES += GRASPIT_ROS

                DEFINES += ROS_DATABASE_MANAGER

                MODEL_DATABASE_CFLAGS = $$system(rospack cflags-only-I household_objects_database)
                INCLUDEPATH += $$MODEL_DATABASE_CFLAGS

                MODEL_DATABASE_LIBS_L = $$system(rospack libs-only-L household_objects_database)
                MODEL_DATABASE_LIBS_l = $$system(rospack libs-only-l household_objects_database)
                QMAKE_LIBDIR += $$MODEL_DATABASE_LIBS_L
                SPLIT_LIBS = $$split(MODEL_DATABASE_LIBS_l,' ')
                for(onelib,SPLIT_LIBS):QMAKE_LIBS += -l$${onelib}
        }
}

mosek {
	!exists($(MOSEK6_0_INSTALLDIR)) {
		error("Mosek not installed or MOSEK6_0_INSTALLDIR environment variable not set")
	}
	INCLUDEPATH += $(MOSEK6_0_INSTALLDIR)/tools/platform/linux32x86/h
	LIBS += -L$(MOSEK6_0_INSTALLDIR)/tools/platform/linux32x86/bin/ -lmoseknoomp -lc -ldl -lm
}

qpOASES {
        !exists($(QPOASES_DIR)) {
                 error("qpOASES not installed or QPOASES_DIR environment variable not set")
        }
        INCLUDEPATH += $(QPOASES_DIR)/INCLUDE
        LIBS += -L$(QPOASES_DIR)/SRC -lqpOASES
}

cgal_qp {
	error("CGAL linking only tested under Windows")
}

boost {

    LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system
}

hardwarelib {
	error("Hardware library only available under Windows")
}
