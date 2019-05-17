cmake_minimum_required(VERSION 2.8.3)
project(graspit)


set(GRASPIT_MAJOR_VERSION 4)
set(GRASPIT_MINOR_VERSION 0)
set(GRASPIT_PATCH_VERSION 0)
set(GRASPIT_VERSION
  ${GRASPIT_MAJOR_VERSION}.${GRASPIT_MINOR_VERSION}.${GRASPIT_PATCH_VERSION})

####################################
## Define build type ###############
####################################
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release"
     FORCE)
endif(NOT CMAKE_BUILD_TYPE)

####################################
## Define graspit variables ####### 
####################################

# main source directory
set (GSRC ${PROJECT_SOURCE_DIR})

set(COLLISION_ENGINE GRASPIT_COLLISION CACHE PATH "back end for GraspIt! Collision detection, current options are GRASPIT_COLLISION, PQP_COLLISION.")
set(CGDB true CACHE PATH "compile CGDB sources as well")
set(PLY true CACHE PATH "compile PLY sources as well")
set(DYNAMICS_ENGINE GRASPIT_DYNAMICS CACHE PATH "back end for GraspIt! Dynamics, current options are GRASPIT_DYNAMICS, BULLET_DYNAMICS")
set(OPTIMIZATION_SOLVER NONE CACHE PATH "back end for GraspIt! optimization solver, current options are NONE, MOSEK, OASES, GUROBI")
set(BUILD_TESTS false CACHE PATH "build tests as well")
set(GRASPIT_DATA_DIR /home/$ENV{USER}/.graspit CACHE PATH "location to save graspit models, worlds and images")

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/graspit)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()



# Configuration file for graspit
set (GRASPIT_CONFIG_CMAKE graspitConfig.cmake)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set (GRASPIT_DEFINES 
        -DGRASPIT_RELEASE  
        -DGRASPIT_EXPORTS)
endif(CMAKE_BUILD_TYPE STREQUAL "Release")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set (GRASPIT_DEFINES  
        -DGRASPIT_DEBUG
        -DGRASPIT_EXPORTS)
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")


set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")

find_package(Qhull REQUIRED)
find_package(SoQt4 REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3)

if (DYNAMICS_ENGINE STREQUAL BULLET_DYNAMICS)

    set(BULLET_ROOT $ENV{BULLET_ROOT})
    message(USING BULLET_ROOT=${BULLET_ROOT} from env variable BULLET_ROOT.)
    find_package(BULLET)

endif(DYNAMICS_ENGINE STREQUAL BULLET_DYNAMICS)

if (OPTIMIZATION_SOLVER STREQUAL MOSEK)

    set(MOSEK_ROOT $ENV{MOSEK6_0_INSTALLDIR})
    message(USING MOSEK_ROOT=${MOSEK_ROOT} from env variable MOSEK6_0_INSTALLDIR.)

endif (OPTIMIZATION_SOLVER STREQUAL MOSEK)

if (OPTIMIZATION_SOLVER STREQUAL OASES)

    set(OASES_ROOT $ENV{QPOASES_DIR})
    message(USING OASES_ROOT=${OASES_ROOT} from env variable QPOASES_DIR.)

endif (OPTIMIZATION_SOLVER STREQUAL OASES)

if (OPTIMIZATION_SOLVER STREQUAL GUROBI)

    set(GUROBI_ROOT $ENV{GUROBI_HOME})
    message(USING GUROBI_ROOT=${GUROBI_ROOT} from env variable GUROBI_HOME.)

endif (OPTIMIZATION_SOLVER STREQUAL GUROBI)

SET( QT_USE_QT3SUPPORT TRUE )
find_package(Qt4 COMPONENTS QtCore QtSql REQUIRED)

include (${QT_USE_FILE})

#--------------------------------------- Core sources ---------------------------------

set (GRASPIT_INCLUDES
    ${GSRC}
    ${GSRC}/include/
    ${GSRC}/cmdline
    ${GSRC}/ui
    # following needs to be included because basic
    # files wouldn't compile without main window
    ${GSRC}/ui/Planner
    ${GSRC}/ui/EGPlanner
)


set (GRASPIT_INCLUDEDIR
    ${GSRC}/include/graspit/
)
# all headers which are directly under the 
# include directory and should be included
# in the installation should be listed in the
# variable GRASPIT_INCLUDEDIR_HEADERS
set (GRASPIT_INCLUDEDIR_HEADERS
    ${GSRC}/include/graspit/robots/barrett.h
    ${GSRC}/include/graspit/robots/pr2Gripper.h
    ${GSRC}/include/graspit/robots/m7.h
    ${GSRC}/include/graspit/robots/m7tool.h
    ${GSRC}/include/graspit/robots/robonaut.h
    ${GSRC}/include/graspit/robots/robotiq.h
    ${GSRC}/include/graspit/robots/humanHand.h
    ${GSRC}/include/graspit/robots/shadow.h
    ${GSRC}/include/graspit/robots/mcGrip.h
    ${GSRC}/include/graspit/robots/puma560.h
    ${GSRC}/include/graspit/robot.h
    ${GSRC}/include/graspit/graspitParser.h
    ${GSRC}/include/graspit/body.h
    ${GSRC}/include/graspit/bBox.h
    ${GSRC}/include/graspit/bbox_inl.h
    ${GSRC}/include/graspit/contact/contact.h
    ${GSRC}/include/graspit/contact/softContact.h
    ${GSRC}/include/graspit/contact/pointContact.h
    ${GSRC}/include/graspit/contact/virtualContact.h
    ${GSRC}/include/graspit/contact/virtualContactOnObject.h
    ${GSRC}/include/graspit/contactSetting.h
    ${GSRC}/include/graspit/quality/quality.h
    ${GSRC}/include/graspit/quality/qualEpsilon.h
    ${GSRC}/include/graspit/quality/qualVolume.h
    ${GSRC}/include/graspit/quality/qualPCR.h
    ${GSRC}/include/graspit/quality/qualPGR.h
    ${GSRC}/include/graspit/debug.h
    ${GSRC}/include/graspit/dof.h
    ${GSRC}/include/graspit/dynamics/dynamics.h
    ${GSRC}/include/graspit/dynamics/dynJoint.h
    ${GSRC}/include/graspit/dynamics/dynamicsEngine.h
    ${GSRC}/include/graspit/eigenGrasp.h
    ${GSRC}/include/graspit/gloveInterface.h
    ${GSRC}/include/graspit/grasp.h
    ${GSRC}/include/graspit/graspRecord.h
    ${GSRC}/include/graspit/graspSolver.h
    ${GSRC}/include/graspit/gws.h
    ${GSRC}/include/graspit/gwsprojection.h
    ${GSRC}/include/graspit/ivmgr.h
    ${GSRC}/include/graspit/jacobian.h
    ${GSRC}/include/graspit/joint.h
    ${GSRC}/include/graspit/kinematicChain.h
    ${GSRC}/include/graspit/lmiOptimizer.h
    ${GSRC}/include/graspit/material.h
    ${GSRC}/include/graspit/matvec3D.h
    ${GSRC}/include/graspit/matvecIO.h
    ${GSRC}/include/graspit/maxdet.h
    ${GSRC}/include/graspit/mytools.h
    ${GSRC}/include/graspit/profiling.h
    ${GSRC}/include/graspit/qhull_mutex.h
    ${GSRC}/include/graspit/plugin.h
    ${GSRC}/include/graspit/SoArrow.h
    ${GSRC}/include/graspit/SoComplexShape.h
    ${GSRC}/include/graspit/SoTorquePointer.h
    ${GSRC}/include/graspit/scanSimulator.h
    ${GSRC}/include/graspit/timer_calls.h
    ${GSRC}/include/graspit/triangle_inl.h
    ${GSRC}/include/graspit/triangle.h
    ${GSRC}/include/graspit/transform.h
    ${GSRC}/include/graspit/flockTransform.h
    ${GSRC}/include/graspit/worldElement.h
    ${GSRC}/include/graspit/worldElementFactory.h
    ${GSRC}/include/graspit/world.h
    ${GSRC}/include/graspit/graspitCore.h
    ${GSRC}/include/graspit/graspitServer.h
    ${GSRC}/include/graspit/graspitApp.h
    ${GSRC}/include/graspit/arch.h
    ${GSRC}/include/graspit/FitParabola.h
    ${GSRC}/include/graspit/bodySensor.h
)
set (GRASPIT_PLANNER_HEADERS
    ${GSRC}/include/graspit/Planner/grasp_visualization.h
    ${GSRC}/include/graspit/Planner/grasp_tester.h
    ${GSRC}/include/graspit/Planner/grasp_preshape.h
    ${GSRC}/include/graspit/Planner/grasp_presenter.h
    ${GSRC}/include/graspit/Planner/grasp_planner.h
    ${GSRC}/include/graspit/Planner/grasp_manager.h
    ${GSRC}/include/graspit/Planner/grasp_grasps.h
    ${GSRC}/include/graspit/Planner/grasp_directions.h
    ${GSRC}/include/graspit/Planner/grasp_coordinates.h
)
set (GRASPIT_EGPLANNER_HEADERS
    ${GSRC}/include/graspit/EGPlanner/search.h
    ${GSRC}/include/graspit/EGPlanner/simAnn.h
    ${GSRC}/include/graspit/EGPlanner/searchState.h
    ${GSRC}/include/graspit/EGPlanner/searchStateImpl.h
    ${GSRC}/include/graspit/EGPlanner/onLinePlanner.h
    ${GSRC}/include/graspit/EGPlanner/egPlanner.h
    ${GSRC}/include/graspit/EGPlanner/simAnnPlanner.h
    ${GSRC}/include/graspit/EGPlanner/simAnnParams.h
    ${GSRC}/include/graspit/EGPlanner/guidedPlanner.h
    ${GSRC}/include/graspit/EGPlanner/loopPlanner.h
    ${GSRC}/include/graspit/EGPlanner/timeTest.h
    ${GSRC}/include/graspit/EGPlanner/graspTesterThread.h
    ${GSRC}/include/graspit/EGPlanner/onLineGraspInterface.h
    ${GSRC}/include/graspit/EGPlanner/listPlanner.h
)

set (GRASPIT_EGPLANNER_ENERGY_HEADERS
    ${GSRC}/include/graspit/EGPlanner/energy/searchEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/searchEnergyFactory.h
    ${GSRC}/include/graspit/EGPlanner/energy/autoGraspQualityEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/compliantEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/contactEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/dynamicAutoGraspEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/guidedAutoGraspEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/guidedPotentialQualityEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/potentialQualityEnergy.h
    ${GSRC}/include/graspit/EGPlanner/energy/strictAutoGraspEnergy.h
)

set (GRASPIT_CONTACT_HEADERS
    ${GSRC}/include/graspit/contact/contact.h
    ${GSRC}/include/graspit/contact/pointContact.h
    ${GSRC}/include/graspit/contact/softContact.h
    ${GSRC}/include/graspit/contact/virtualContact.h
    ${GSRC}/include/graspit/contact/virtualContactOnObject.h
)


set (GRASPIT_QUALITY_HEADERS
    ${GSRC}/include/graspit/quality/quality.h
    ${GSRC}/include/graspit/quality/qualEpsilon.h
    ${GSRC}/include/graspit/quality/qualPCR.h
    ${GSRC}/include/graspit/quality/qualPGR.h
    ${GSRC}/include/graspit/quality/qualVolume.h
)

# all headers which are in the  cmdline
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_CMDLINE_HEADERS
set (GRASPIT_CMDLINE_HEADERS
    ${GSRC}/include/graspit/cmdline/cmdline.h 
)

# all headers which are in the src/Collision 
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_SRC_COLLISION_HEADERS
set (GRASPIT_SRC_COLLISION_HEADERS
    ${GSRC}/include/graspit/Collision/collisionInterface.h
    ${GSRC}/include/graspit/Collision/collisionStructures.h
)


# all headers which are in the include/math 
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_INCLUDE_MATH_HEADERS
set (GRASPIT_INCLUDE_MATH_HEADERS
    ${GSRC}/include/graspit/math/matrix.h
)


if (CGDB)  
    set(CGDB_HEADERS
        include/graspit/DBase/graspit_db_model.h
        include/graspit/DBase/graspit_db_grasp.h
        include/graspit/DBase/graspit_db_planner.h
        include/graspit/DBase/taskDispatcher.h
        include/graspit/DBase/preGraspCheckTask.h
        include/graspit/DBase/graspClusteringTask.h
        include/graspit/DBase/graspTransferCheckTask.h
        include/graspit/DBase/tableCheckTask.h
        include/graspit/DBase/compliantGraspCopyTask.h
    )
    set(CGDB_MOC_HEADERS
        ui/DBase/dbaseDlg.h
        ui/DBase/dbasePlannerDlg.h
        include/graspit/DBase/dbase_grasp.h
        include/graspit/DBase/graspit_db_planner.h
        include/graspit/DBase/graspPlanningTask.h
    )

    set(CGDB_PLANNER_HEADERS
        include/graspit/DBase/DBPlanner/grasp.h
        include/graspit/DBase/DBPlanner/model.h
        include/graspit/DBase/DBPlanner/db_manager.h
        include/graspit/DBase/DBPlanner/database.h
        include/graspit/DBase/DBPlanner/sql_database_manager.h
        include/graspit/DBase/DBPlanner/task.h
    )

    set(CGDB_SOURCES
        ui/DBase/dbaseDlg.cpp
        ui/DBase/dbasePlannerDlg.cpp
        src/DBase/dbase_grasp.cpp
        src/DBase/graspit_db_model.cpp
        src/DBase/graspit_db_grasp.cpp
        src/DBase/DBPlanner/database.cpp
        src/DBase/DBPlanner/sql_database_manager.cpp
        src/DBase/graspit_db_planner.cpp
        src/DBase/taskDispatcher.cpp
        src/DBase/graspPlanningTask.cpp
        src/DBase/preGraspCheckTask.cpp
        src/DBase/graspClusteringTask.cpp
        src/DBase/graspTransferCheckTask.cpp
        src/DBase/tableCheckTask.cpp
        src/DBase/compliantGraspCopyTask.cpp
    )
    set(CGDB_FORMS 
        ui/DBase/dbaseDlg.ui
        ui/DBase/dbasePlannerDlg.ui
    )

    # you can also define BATCH_PROCESSING in order to supress error output which requires user attention
    set (GRASPIT_DEFINES 
        ${GRASPIT_DEFINES}
        -DCGDB_ENABLED 
        -DBATCH_PROCESSING
    )

    set (GRASPIT_CGDB_INCLUDES
        ${GSRC}/include/graspit/
        ${GSRC}/include/graspit/DBase/
        ${GSRC}/include/graspit/DBase/DBPlanner/
    )
endif (CGDB)  

if (PLY)
    set (PLY_HEADERS
        ${GSRC}/include/graspit/ply/ply.h
        ${GSRC}/include/graspit/ply/mesh_loader.h
    )
    set (PLY_SOURCES
        ${GSRC}/src/ply/ply.c
        ${GSRC}/src/ply/mesh_loader.cpp
    )
    set (GRASPIT_DEFINES 
        ${GRASPIT_DEFINES}
        -DPLY_READER
    )
    set (GRASPIT_PLY_INCLUDES
        ${GSRC}/ply
    )
endif (PLY)


set (MOC_HEADERS
    ${GSRC}/include/graspit/body.h
    ${GSRC}/include/graspit/robot.h
    ${GSRC}/include/graspit/world.h
    ${GSRC}/include/graspit/grasp.h
    ${GSRC}/include/graspit/worldElement.h
    ${GSRC}/include/graspit/graspitServer.h
    ${GSRC}/include/graspit/ivmgr.h
    ${GSRC}/include/graspit/EGPlanner/egPlanner.h
    ${GSRC}/include/graspit/EGPlanner/simAnn.h
    ${GSRC}/include/graspit/EGPlanner/loopPlanner.h
    ${GSRC}/include/graspit/EGPlanner/energy/searchEnergy.h
    ${GSRC}/include/graspit/EGPlanner/guidedPlanner.h
    ${GSRC}/include/graspit/Planner/grasp_tester.h
    ${GSRC}/include/graspit/robots/humanHand.h
    ${GSRC}/include/graspit/robots/m7tool.h
    ${GSRC}/include/graspit/robots/m7.h
    ${GSRC}/include/graspit/robots/barrett.h
    ${GSRC}/include/graspit/robots/robonaut.h
    ${GSRC}/include/graspit/robots/pr2Gripper.h
    ${GSRC}/include/graspit/robots/robotiq.h
    ${GSRC}/include/graspit/robots/mcGrip.h
    ${CGDB_MOC_HEADERS}
) 


set (GRASPIT_CORE_SOURCES
    ${GSRC}/src/graspitParser.cpp 
    ${GSRC}/src/arch.cpp 
    ${GSRC}/src/robots/barrett.cpp 
    ${GSRC}/src/robots/humanHand.cpp 
    ${GSRC}/src/robots/m7.cpp 
    ${GSRC}/src/robots/m7tool.cpp 
    ${GSRC}/src/robots/mcGrip.cpp 
    ${GSRC}/src/robots/pr2Gripper.cpp 
    ${GSRC}/src/robots/puma560.cpp 
    ${GSRC}/src/robots/robonaut.cpp 
    ${GSRC}/src/robots/robotiq.cpp 
    ${GSRC}/src/robots/shadow.cpp 
    ${GSRC}/src/bBox.cpp 
    ${GSRC}/src/body.cpp 
    ${GSRC}/src/contact/contact.cpp
    ${GSRC}/src/contact/softContact.cpp
    ${GSRC}/src/contact/pointContact.cpp
    ${GSRC}/src/contact/virtualContact.cpp 
    ${GSRC}/src/contact/virtualContactOnObject.cpp
    ${GSRC}/src/contactSetting.cpp 
    ${GSRC}/src/quality/quality.cpp
    ${GSRC}/src/quality/qualEpsilon.cpp
    ${GSRC}/src/quality/qualVolume.cpp
    ${GSRC}/src/quality/qualPCR.cpp
    ${GSRC}/src/quality/qualPGR.cpp
    ${GSRC}/src/dof.cpp 
    ${GSRC}/src/dynamics/dynamics.cpp 
    ${GSRC}/src/dynamics/dynJoint.cpp 
    ${GSRC}/src/eigenGrasp.cpp 
    ${GSRC}/src/gloveInterface.cpp 
    ${GSRC}/src/grasp.cpp 
    ${GSRC}/src/graspitCore.cpp
    ${GSRC}/src/graspitServer.cpp 
    ${GSRC}/src/graspitApp.cpp 
    ${GSRC}/src/graspRecord.cpp 
    ${GSRC}/src/graspSolver.cpp 
    ${GSRC}/src/gws.cpp 
    ${GSRC}/src/gwsprojection.cpp 
    ${GSRC}/src/ivmgr.cpp 
    ${GSRC}/src/jacobian.cpp 
    ${GSRC}/src/joint.cpp 
    ${GSRC}/src/kinematicChain.cpp 
    ${GSRC}/src/lmiOptimizer.cpp 
    ${GSRC}/src/material.cpp 
    ${GSRC}/src/matvec.cpp 
    ${GSRC}/src/matvecIO.cpp 
    ${GSRC}/src/maxdet_src.cpp 
    ${GSRC}/src/mytools.cpp 
    ${GSRC}/src/profiling.cpp 
    ${GSRC}/src/plugin.cpp 
    ${GSRC}/src/robot.cpp 
    ${GSRC}/src/scanSimulator.cpp 
    ${GSRC}/src/SoArrow.cpp 
    ${GSRC}/src/SoComplexShape.cpp 
    ${GSRC}/src/SoTorquePointer.cpp 
    ${GSRC}/src/triangle.cpp 
    ${GSRC}/src/world.cpp 
    ${GSRC}/src/worldElement.cpp 
    ${GSRC}/src/worldElementFactory.cpp 
    ${GSRC}/src/math/matrix.cpp 
    ${GSRC}/src/transform.cpp
    ${GSRC}/src/flockTransform.cpp
    ${GSRC}/src/Collision/collisionInterface.cpp 
    ${GSRC}/src/Planner/grasp_visualization.cpp 
    ${GSRC}/src/Planner/grasp_tester.cpp 
    ${GSRC}/src/Planner/grasp_preshape.cpp 
    ${GSRC}/src/Planner/grasp_presenter.cpp 
    ${GSRC}/src/Planner/grasp_planner.cpp 
    ${GSRC}/src/Planner/grasp_manager.cpp 
    ${GSRC}/src/Planner/grasp_grasps.cpp 
    ${GSRC}/src/Planner/grasp_directions.cpp 
    ${GSRC}/src/Planner/grasp_coordinates.cpp 
    ${GSRC}/src/EGPlanner/simAnn.cpp 
    ${GSRC}/src/EGPlanner/searchState.cpp 
    ${GSRC}/src/EGPlanner/searchStateImpl.cpp 
    ${GSRC}/src/EGPlanner/energy/searchEnergy.cpp
    ${GSRC}/src/EGPlanner/energy/searchEnergyFactory.cpp
    ${GSRC}/src/EGPlanner/onLinePlanner.cpp 
    ${GSRC}/src/EGPlanner/egPlanner.cpp 
    ${GSRC}/src/EGPlanner/simAnnPlanner.cpp 
    ${GSRC}/src/EGPlanner/simAnnParams.cpp
    ${GSRC}/src/EGPlanner/guidedPlanner.cpp 
    ${GSRC}/src/EGPlanner/loopPlanner.cpp 
    ${GSRC}/src/EGPlanner/timeTest.cpp 
    ${GSRC}/src/EGPlanner/graspTesterThread.cpp 
    ${GSRC}/src/EGPlanner/onLineGraspInterface.cpp 
    ${GSRC}/src/EGPlanner/listPlanner.cpp 
    ${GSRC}/src/EGPlanner/energy/autoGraspQualityEnergy.cpp
    ${GSRC}/src/EGPlanner/energy/compliantEnergy.cpp 
    ${GSRC}/src/EGPlanner/energy/contactEnergy.cpp 
    ${GSRC}/src/EGPlanner/energy/dynamicAutoGraspEnergy.cpp
    ${GSRC}/src/EGPlanner/energy/guidedAutoGraspEnergy.cpp 
    ${GSRC}/src/EGPlanner/energy/guidedPotentialQualityEnergy.cpp 
    ${GSRC}/src/EGPlanner/energy/potentialQualityEnergy.cpp 
    ${GSRC}/src/EGPlanner/energy/strictAutoGraspEnergy.cpp 
    ${GSRC}/src/bodySensor.cpp
)

#--------------------------------------- Collision library ---------------------------------


if (COLLISION_ENGINE STREQUAL PQP_COLLISION)
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
         -DPQP_COLLISION)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
        ${GSRC}/PQP-VCOLLIDE/include 
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src 
        ${GSRC}/PQP-VCOLLIDE/ivcollide)
    set (COLLISION_PQP_HEADERS
        ${GSRC}/src/Collision/PQP/PQPCollision.h)
    set (GRASPIT_COLLISION_SOURCES
        ${GSRC}/src/Collision/PQP/PQPCollision.cpp 
        ${GSRC}/PQP-VCOLLIDE/src/VCollide.cpp
        ${GSRC}/PQP-VCOLLIDE/src/VInternal.cpp
        ${GSRC}/PQP-VCOLLIDE/src/NBody.cpp
        ${GSRC}/PQP-VCOLLIDE/src/PairData.cpp
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src/Build.cpp
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src/BV.cpp
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src/PQP.cpp
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src/TriDist.cpp
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src/Tri.cpp)

else (COLLISION_ENGINE STREQUAL PQP_COLLISION)
 
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DGRASPIT_COLLISION)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
        ${GSRC}/include/graspit/Collision/Graspit)
    set (COLLISION_GRASPIT_HEADERS
        ${GSRC}/include/graspit/Collision/Graspit/collisionModel.h
        ${GSRC}/include/graspit/Collision/Graspit/collisionAlgorithms.h
        ${GSRC}/include/graspit/Collision/Graspit/collisionAlgorithms_inl.h
        ${GSRC}/include/graspit/Collision/Graspit/graspitCollision.h)
    set (GRASPIT_COLLISION_SOURCES
        ${GSRC}/src/Collision/Graspit/collisionModel.cpp 
        ${GSRC}/src/Collision/Graspit/collisionAlgorithms.cpp 
        ${GSRC}/src/Collision/Graspit/graspitCollision.cpp)

endif (COLLISION_ENGINE STREQUAL PQP_COLLISION)


#--------------------------------------- Dynamics Engine ---------------------------------
if(DYNAMICS_ENGINE STREQUAL GRASPIT_DYNAMICS)
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DGRASPIT_DYNAMICS)
    set (GRASPIT_DYNAMICS_INCLUDES
        ${GSRC}/include/graspit/dynamics/graspitDynamics.h
        )
    set (GRASPIT_DYNAMICS_SOURCES
        ${GSRC}/src/dynamics/graspitDynamics.cpp
        )
endif(DYNAMICS_ENGINE STREQUAL GRASPIT_DYNAMICS)

#------------------------------------ Optimization Solver --------------------------------
if(OPTIMIZATION_SOLVER STREQUAL MOSEK)
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DMOSEK_QP)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
        ${MOSEK_ROOT}/tools/platform/linux64x86/h)
    set (GRASPIT_OPTIMIZATION_HEADERS
        ${GSRC}/src/math/mosek_qp.h 
        )
    set (GRASPIT_OPTIMIZATION_SOURCES
        ${GSRC}/src/math/mosek_qp.cpp
        )
    find_library(QP_LIBRARIES
        NAMES moseknoomp64
        PATHS ${MOSEK_ROOT}/tools/platform/linux64x86/bin/
    )
endif(OPTIMIZATION_SOLVER STREQUAL MOSEK)

if(OPTIMIZATION_SOLVER STREQUAL OASES)
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DOASES_QP)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
        ${OASES_ROOT}/INCLUDE)
    set (GRASPIT_OPTIMIZATION_HEADERS
        ${GSRC}/src/math/qpoases.h  
        )
    set (GRASPIT_OPTIMIZATION_SOURCES
        ${GSRC}/src/math/qpoases.cpp  
        )
    find_library(QP_LIBRARIES
        NAMES qpOASES
        PATHS ${OASES_ROOT}/SRC/
    )
endif(OPTIMIZATION_SOLVER STREQUAL OASES)

if(OPTIMIZATION_SOLVER STREQUAL GUROBI)
    set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DGUROBI_SOLVER)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
        ${GUROBI_ROOT}/include)
    set (GRASPIT_OPTIMIZATION_HEADERS
        ${GSRC}/src/math/gurobi.h 
        )
    set (GRASPIT_OPTIMIZATION_SOURCES
        ${GSRC}/src/math/gurobi.cpp
        )
    find_library(GUROBI_CPP_LIBRARIES
        NAMES gurobi_g++5.2
        PATHS ${GUROBI_ROOT}/lib/
    )
    find_library(GUROBI_LIBRARIES
        NAMES gurobi81
        PATHS ${GUROBI_ROOT}/lib/
    )
endif(OPTIMIZATION_SOLVER STREQUAL GUROBI)

#--------------------------------------- User interface: main window and dialogs ---------------------------------

set (GRASPIT_UI_INCLUDES
    ${GSRC}/ui 
    ${GSRC}/ui/Planner 
    ${GSRC}/ui/EGPlanner
)

set (GRASPIT_INCLUDES
    ${GRASPIT_INCLUDES}
    ${GRASPIT_UI_INCLUDES}
    ${GRASPIT_CGDB_INCLUDES}
    ${GRASPIT_PLY_INCLUDES}
    ${BULLET_INCLUDE_DIR}
)

set (GRASPIT_FORMS
    ${GSRC}/ui/mainWindow.ui 
    ${GSRC}/ui/about.ui 
    ${GSRC}/ui/archBuilderDlg.ui 
    ${GSRC}/ui/barrettHandDlg.ui 
    ${GSRC}/ui/bodyPropDlg.ui 
    ${GSRC}/ui/contactExaminerDlg.ui 
    ${GSRC}/ui/eigenGraspDlg.ui 
    ${GSRC}/ui/gfoDlg.ui 
    ${GSRC}/ui/gsaDlg.ui 
    ${GSRC}/ui/gloveCalibrationDlg.ui 
    ${GSRC}/ui/graspCaptureDlg.ui 
    ${GSRC}/ui/gwsProjDlgBase.ui 
    ${GSRC}/ui/qmDlg.ui 
    ${GSRC}/ui/qualityIndicator.ui 
    ${GSRC}/ui/settingsDlg.ui 
    ${GSRC}/ui/Planner/plannerdlg.ui 
    ${GSRC}/ui/EGPlanner/egPlannerDlg.ui 
    ${GSRC}/ui/EGPlanner/compliantPlannerDlg.ui
)

set (GRASPIT_FORM_HEADERS 
    ${GSRC}/ui/mainWindow.h 
    ${GSRC}/ui/archBuilderDlg.h 
    ${GSRC}/ui/barrettHandDlg.h 
    ${GSRC}/ui/bodyPropDlg.h 
    ${GSRC}/ui/contactExaminerDlg.h 
    ${GSRC}/ui/eigenGraspDlg.h 
    ${GSRC}/ui/gfoDlg.h 
    ${GSRC}/ui/gsaDlg.h 
    ${GSRC}/ui/gloveCalibrationDlg.h 
    ${GSRC}/ui/graspCaptureDlg.h 
    ${GSRC}/ui/gwsProjDlg.h 
    ${GSRC}/ui/settingsDlg.h 
    ${GSRC}/ui/qmDlg.h 
    ${GSRC}/ui/Planner/plannerdlg.h 
    ${GSRC}/ui/EGPlanner/egPlannerDlg.h 
    ${GSRC}/ui/EGPlanner/compliantPlannerDlg.h
)

set (GRASPIT_FORM_SOURCES
    ${GSRC}/ui/mainWindow.cpp 
    ${GSRC}/ui/archBuilderDlg.cpp 
    ${GSRC}/ui/barrettHandDlg.cpp 
    ${GSRC}/ui/bodyPropDlg.cpp 
    ${GSRC}/ui/contactExaminerDlg.cpp 
    ${GSRC}/ui/eigenGraspDlg.cpp 
    ${GSRC}/ui/gfoDlg.cpp 
    ${GSRC}/ui/gsaDlg.cpp 
    ${GSRC}/ui/gloveCalibrationDlg.cpp 
    ${GSRC}/ui/graspCaptureDlg.cpp 
    ${GSRC}/ui/gwsProjDlg.cpp 
    ${GSRC}/ui/qmDlg.cpp 
    ${GSRC}/ui/settingsDlg.cpp 
    ${GSRC}/ui/Planner/plannerdlg.cpp 
    ${GSRC}/ui/EGPlanner/egPlannerDlg.cpp 
    ${GSRC}/ui/EGPlanner/compliantPlannerDlg.cpp
)

set (MOC_HEADERS
    ${MOC_HEADERS}
    ${GSRC}/ui/mainWindow.h
    ${GSRC}/ui/mainWindow.h
    ${GSRC}/ui/bodyPropDlg.h
    ${GSRC}/ui/eigenGraspDlg.h
    ${GSRC}/ui/EGPlanner/egPlannerDlg.h
    ${GSRC}/ui/EGPlanner/compliantPlannerDlg.h
    ${GSRC}/ui/Planner/plannerdlg.h
    ${GSRC}/ui/graspCaptureDlg.h
    ${GSRC}/ui/contactExaminerDlg.h
    ${GSRC}/ui/settingsDlg.h
    ${GSRC}/ui/gfoDlg.h
    ${GSRC}/ui/gsaDlg.h
    ${GSRC}/ui/qmDlg.h
    ${GSRC}/ui/gwsProjDlg.h
)




#-------------------------------------- images and resources -------------------------------------------------------

set (GRASPIT_IMAGES
    ${GSRC}/src/images/play.xpm 
    ${GSRC}/src/images/pause.xpm 
    ${GSRC}/src/images/splash.jpg 
    ${GSRC}/src/images/logo.png 
    ${GSRC}/src/images/nocollide.xpm 
    ${GSRC}/src/images/collide.xpm 
    ${GSRC}/src/images/translateTool.xpm 
    ${GSRC}/src/images/selectTool.xpm 
    ${GSRC}/src/images/rotateTool.xpm 
    ${GSRC}/src/images/mark.xpm 
    ${GSRC}/src/images/prevMark.xpm 
    ${GSRC}/src/images/filenew.xpm 
    ${GSRC}/src/images/fileopen.xpm 
    ${GSRC}/src/images/filesave.xpm 
    ${GSRC}/src/images/filenew 
    ${GSRC}/src/images/fileopen 
    ${GSRC}/src/images/filesave 
    ${GSRC}/src/images/print 
    ${GSRC}/src/images/undo 
    ${GSRC}/src/images/redo 
    ${GSRC}/src/images/editcut 
    ${GSRC}/src/images/editcopy 
    ${GSRC}/src/images/editpaste 
    ${GSRC}/src/images/searchfind
)


#-------------------------------------- The TinyXML XML parser ---------------------------------------------------

set (TINYXML_INCLUDES
    ${GSRC}/tinyxml
)
set (TINYXML_HEADERS 
    ${GSRC}/tinyxml/tinyxml.h 
    ${GSRC}/tinyxml/tinystr.h
)

set (GRASPIT_CORE_SOURCES
    ${GRASPIT_CORE_SOURCES}
    ${GSRC}/tinyxml/tinyxmlparser.cpp
)

##########################################
## Generate Qt files for the icon sets  ## 
##########################################

# Custom command to create an image collection with uic3 (it has to be uic3, not uic) like this: 
# uic3 -embed graspit -o qt_image_collection.cpp path_to_my_images/*
# The qt_image_collection.cpp must then be added to the sources to compile.
# I also tried adding images as resource (QT4_ADD_RESOURCES) but this didn't work with the 
# current GraspIt source.
ADD_CUSTOM_COMMAND(OUTPUT 
    ${CMAKE_CURRENT_BINARY_DIR}/qt_image_collection.cpp 
    COMMAND ${QT_UIC3_EXECUTABLE} 
    ARGS -o ${CMAKE_CURRENT_BINARY_DIR}/qt_image_collection.cpp 
        -embed graspit ${GRASPIT_IMAGES} 
    DEPENDS ${GRASPIT_IMAGES})

SET(GRASPIT_IMAGE_SOURCES 
    ${CMAKE_CURRENT_BINARY_DIR}/qt_image_collection.cpp)

################################
## Make Qt Moc files of GraspIT
################################

set (ALL_GRASPIT_FORMS
    ${GRASPIT_FORMS}
    ${CGDB_FORMS}
)

set (GRASPIT_UI_INCLUDE_DIRNAME ui)

# we will need to create a special folder named GRASPIT_UI_INCLUDE_DIRNAME
# into which to generate the UI files, so that they can be included from
# the graspit header files with the right prefix.

set (GRASPIT_UI_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${GRASPIT_UI_INCLUDE_DIRNAME})

    
file(MAKE_DIRECTORY ${GRASPIT_UI_INCLUDE_PATH})

set (OLD_CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}) # backup current binary dir
set (CMAKE_CURRENT_BINARY_DIR ${GRASPIT_UI_INCLUDE_PATH}) #artificially changing output location for UI files

QT4_WRAP_UI(UI_OUTFILES ${ALL_GRASPIT_FORMS}) # generate UI outfiles (will use CMAKE_CURRENT_BINARY_DIR)

set (CMAKE_CURRENT_BINARY_DIR ${OLD_CMAKE_CURRENT_BINARY_DIR}) # restore current binary dir

QT4_WRAP_CPP(MOC_OUTFILES ${MOC_HEADERS})


# add Qt stuff to the includes
set (GRASPIT_INCLUDES 
     ${GRASPIT_INCLUDES}
     ${GRASPIT_UI_INCLUDE_PATH}
     ${SOQT_INCLUDE_DIRS}
     ${QT_INCLUDES}
     ${QT_INCLUDE_DIR}
     ${QT_QT3SUPPORT_INCLUDE_DIR}
     ${QHULL_INCLUDE_DIRS}
)


## Specify additional locations of header files
include_directories(
    ${GRASPIT_INCLUDES}
    ${TINYXML_INCLUDES}
    EIGEN3_INCLUDE_DIR
)


############
## Build  ## 
############

add_definitions(${GRASPIT_DEFINES} ${SOQT_CXXFLAGS} ${LAPACK_LINKER_FLAGS} -g)
link_directories(${SOQT_LINK_DIRS} ${QT_LIBRARY_DIR})

set (GRASPIT_LIBRARY_SOURCES
    ${GRASPIT_CORE_SOURCES}
    ${CGDB_SOURCES}
    ${PLY_SOURCES}
    ${GRASPIT_COLLISION_SOURCES}
    ${GRASPIT_DYNAMICS_SOURCES}
    ${GRASPIT_OPTIMIZATION_SOURCES}
    ${GRASPIT_FORM_SOURCES}
    ${GRASPIT_IMAGE_SOURCES}
    ${UI_OUTFILES}
    ${MOC_OUTFILES}
)

add_library(graspit SHARED ${GRASPIT_LIBRARY_SOURCES})

## Make simulator executable

# sources for executable graspit_simulator
set(SIMULATOR_SRC ${GSRC}/src/main.cpp)

add_executable(graspit_simulator ${SIMULATOR_SRC})

## System libraries which are needed for graspit
set(GRASPIT_LINK_LIBRARIES
   ${QT_LIBRARIES}
   ${QT_QT3SUPPORT_LIBRARY}
   ${QHULL_LIBRARIES}
   ${SOQT_LIBRARY}
   ${LAPACK_LIBRARIES}
   ${BULLET_LIBRARIES}
   ${QP_LIBRARIES}
   ${GUROBI_LIBRARIES}
   ${GUROBI_CPP_LIBRARIES}
)

## Link the shared library and the executable
target_link_libraries(graspit ${GRASPIT_LINK_LIBRARIES})
target_link_libraries(graspit_simulator graspit ${GRASPIT_LINK_LIBRARIES})

## Set install destination paths
set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

set(INSTALL_BINS graspit_simulator graspit)

## Mark executables and/or libraries for installation
install(TARGETS ${INSTALL_BINS} 
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${BIN_DESTINATION}
)


install(DIRECTORY
    ${GRASPIT_INCLUDEDIR}
    DESTINATION ${INCLUDE_DESTINATION}/${PROJECT_NAME}
)


if (PQP_COLLISION)
    install(FILES
        ${GRASPIT_INCLUDEDIR_HEADERS}
        ${COLLISION_PQP_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/${PROJECT_NAME}/Collision/PQP
    )
endif (PQP_COLLISION)


if (NOT OPTIMIZATION_SOLVER STREQUAL NONE)
    install(FILES
        ${GRASPIT_OPTIMIZATION_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/${PROJECT_NAME}/math
    )
endif (NOT OPTIMIZATION_SOLVER STREQUAL NONE)

install(FILES
    ${UI_OUTFILES}
    DESTINATION ${INCLUDE_DESTINATION}/${PROJECT_NAME}/${GRASPIT_UI_INCLUDE_DIRNAME}
)

#############
## Install ##
#############

install(TARGETS
    graspit 
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
    EXPORT graspitTargets
    DESTINATION ${LIB_DESTINATION})

install(EXPORT graspitTargets
   DESTINATION ${INSTALL_CMAKE_DIR})

# Install the graspitConfig.cmake and graspitConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/graspitConfig.cmake"
  "${PROJECT_BINARY_DIR}/graspitConfigVersion.cmake"
    CMakeMacros/FindBULLET.cmake
    CMakeMacros/FindSoQt4.cmake
    CMakeMacros/FindQhull.cmake
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(DIRECTORY
    worlds
    models
    images
    DESTINATION ${GRASPIT_DATA_DIR}
)

# # Add all targets to the build-tree export set
# export(TARGETS graspit_simulator graspit
#   FILE "${PROJECT_BINARY_DIR}/graspitTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE GRASPIT)

# Create the graspitConfig.cmake and GraspitConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INCLUDE_DESTINATION}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(graspitConfig.cmake.in
  "${PROJECT_BINARY_DIR}/graspitConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${GRASPIT_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(graspitConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/graspitConfig.cmake" @ONLY)
# ... for both
configure_file(graspitConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/graspitConfigVersion.cmake" @ONLY)




#############
## Testing ##
#############

if(BUILD_TESTS)

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Make test executables
add_executable(primative_planner_tests
    ${GSRC}/test/primative_planner_tests.cpp
)

add_executable(eigen_tests
    ${GSRC}/test/eigen_tests.cpp
)

add_executable(transform_tests
    ${GSRC}/test/transform_tests.cpp
)

add_executable(graspit_collision_tests
    ${GSRC}/test/graspit_collision_tests.cpp
)

set_target_properties( primative_planner_tests eigen_tests transform_tests graspit_collision_tests
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test"
)

add_dependencies(primative_planner_tests googletest)
add_dependencies(eigen_tests googletest)
add_dependencies(transform_tests googletest)
add_dependencies(graspit_collision_tests googletest)

target_link_libraries(primative_planner_tests
   graspit
   ${GTEST_BOTH_LIBRARIES}
)

target_link_libraries(eigen_tests
   graspit
   ${GTEST_BOTH_LIBRARIES}
)

target_link_libraries(transform_tests
   graspit
   ${GTEST_BOTH_LIBRARIES}
)

target_link_libraries(graspit_collision_tests
   graspit
   ${GTEST_BOTH_LIBRARIES}
)






endif(BUILD_TESTS)
