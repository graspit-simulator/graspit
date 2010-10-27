TEMPLATE	= app
LANGUAGE	= C++

#-------------------------options--------------------------

#comment out this line for compiling in Release mode
#usually, compiling in Release mode delivers a significant gain in performance
#in MS Visual Studio *also* set the project mode to Release
CONFIG += graspitdbg

#select collision detection engine
#possible values: graspit_collision pqp_collision
COLLISION = graspit_collision

#select blas and lapack libraries, Windows-only
#possible values: mkl clapack
LAPACK = mkl

#build and use interface with Columbia Grasp Database
CONFIG += cgdb
#and use the ros database manager
CONFIG += graspit_ros

#link against Mosek QP solver (must be installed separately)
#CONFIG += mosek

#link against the qpOASES solver (must be installed separately)
#CONFIG += qpOASES

#enable the CGAL QP solver (must be installed separately)
#also needs boost
#CONFIG += cgal_qp

#enable linking against boost (must be installed separately)
#CONFIG += boost

#------------------system-specific libraries---------------
CONFIG +=  $$COLLISION $$LAPACK

graspitdbg {
    CONFIG += debug
} else {
    CONFIG += release
    DEFINES += GRASPIT_RELEASE
}

win32 {
	include(graspit-lib-WINDOWS.pro)
} else {
	include(graspit-lib-LINUX.pro)
}

#------------------GraspIt! core files---------------------

include(graspit-core.pro)
