# Linux-specific libraries for GraspIt!. Included from graspit.pro - not for standalone use.

# ---------------------- Blas and Lapack ----------------------------------

LIBS += -lblas -llapack 

HEADERS += include/lapack_wrappers.h


# ---------------------- General libraries and utilities ----------------------------------

#specifying the qhull library by exact file, so that the system does not use
#the version that comes from the package manager instead 
LIBS	+= qhull/libqhull.a 
#LIBS	+= -Lqhull -lqhull 

LIBS	+= -L$(COINDIR)/lib -lSoQt -lCoin -lGL -lpthread

MOC_DIR = .moc
OBJECTS_DIR = .obj

#------------------------------------ add-ons --------------------------------------------

cgdb {
        graspit_ros {        
                SOURCES += src/DBase/DBPlanner/ros_database_manager.cpp
                HEADERS += src/DBase/DBPlanner/ros_database_manager.h
                
                DEFINES += GRASPIT_ROS

                DEFINES += ROS_DATABASE_MANAGER

                MODEL_DATABASE_CFLAGS = $$system(rospack export --lang=cpp --attrib=cflags household_objects_database)
                QMAKE_CXXFLAGS += $$MODEL_DATABASE_CFLAGS

                MODEL_DATABASE_LFLAGS = $$system(rospack export --lang=cpp --attrib=lflags household_objects_database)
                QMAKE_LFLAGS += $$MODEL_DATABASE_LFLAGS
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
	error("Boost linking only tested under Windows")
}

hardwarelib {
	error("Hardware library only available under Windows")
}
