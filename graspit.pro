TEMPLATE	= app
LANGUAGE	= C++

#-------------------------options--------------------------

#comment out this line for compiling in Release mode
#usually, compiling in Release mode delivers a significant gain in performance
#in MS Visual Studio *also* set the project mode to Release
CONFIG += graspitdbg
CONFIG += debug
DEFINES +=JARED_AUTO_RUNNING

#select collision detection engine
#possible values: graspit_collision pqp_collision
COLLISION = graspit_collision

#select blas and lapack libraries, Windows-only
#possible values: mkl clapack
LAPACK = clapack

#build and use interface with Columbia Grasp Database
CONFIG += cgdb

#link against the Hardware library (included with this distribution) which 
#provides access to various hardware. Hardware library must be compiled
#separately and is Windows-only.
#CONFIG += hardwarelib

#link against Mosek QP solver (must be installed separately)
#CONFIG += mosek

#enable the CGAL QP solver (must be installed separately)
#also needs boost
#CONFIG += cgal_qp

#enable linking against boost (must be installed separately)
#CONFIG += boost

#a project currently under construction
#CONFIG += eigengrids

#enable arizona project here
#CONFIG += arizona

#enable staubli control
#CONFIG += staubli

#enable semantic planner
#CONFIG += semantic_planner

#enable blind planner
CONFIG += blind_planner

#CONFIG += uncertainty_planner

#CONFIG += strategy_planner

#need to be enabled with levmar and PCL
#CONFIG += local_explore
#CONFIG += levmar
#CONFIG += PCL

#------------------system-specific libraries---------------
CONFIG +=  $$COLLISION $$LAPACK

win32 {
	include(graspit-lib-WINDOWS.pro)
} else {
	include(graspit-lib-LINUX.pro)
}

#------------------GraspIt! core files---------------------

include(graspit-core.pro)
