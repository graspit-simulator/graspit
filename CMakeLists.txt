cmake_minimum_required(VERSION 2.8.3)
project(graspit)

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
set(BUILD_TESTS false CACHE PATH "build tests as well")

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


SET( QT_USE_QT3SUPPORT TRUE )
find_package(Qt4 COMPONENTS QtCore QtSql REQUIRED)

include (${QT_USE_FILE})

#--------------------------------------- Core sources ---------------------------------

set (GRASPIT_INCLUDES
    ${GSRC}
    ${GSRC}/include/
    ${GSRC}/include/robots
    ${GSRC}/include/math
    ${GSRC}/include/dynamics
    ${GSRC}/include/contact
    ${GSRC}/include/EGPlanner
    ${GSRC}/include/EGPlanner/energy
    ${GSRC}/include/Planner
    ${GSRC}/include/Collision
    ${GSRC}/cmdline
    ${GSRC}/ui
    # following needs to be included because basic
    # files wouldn't compile without main window
    ${GSRC}/ui/Planner
    ${GSRC}/ui/EGPlanner
)


# all headers which are directly under the 
# include directory and should be included
# in the installation should be listed in the
# variable GRASPIT_INCLUDEDIR_HEADERS
set (GRASPIT_INCLUDEDIR_HEADERS
    ${GSRC}/include/robots/barrett.h 
    ${GSRC}/include/robots/pr2Gripper.h 
    ${GSRC}/include/robots/m7.h 
    ${GSRC}/include/robots/m7tool.h 
    ${GSRC}/include/robots/robonaut.h 
    ${GSRC}/include/robots/robotiq.h 
    ${GSRC}/include/robots/humanHand.h 
    ${GSRC}/include/robots/shadow.h
    ${GSRC}/include/robots/mcGrip.h 
    ${GSRC}/include/robots/puma560.h 
    ${GSRC}/include/robot.h 
    ${GSRC}/include/graspitParser.h 
    ${GSRC}/include/body.h 
    ${GSRC}/include/bBox.h 
    ${GSRC}/include/bbox_inl.h 
    ${GSRC}/include/contact/contact.h 
    ${GSRC}/include/contact/softContact.h 
    ${GSRC}/include/contact/pointContact.h 
    ${GSRC}/include/contact/virtualContact.h 
    ${GSRC}/include/contact/virtualContactOnObject.h
    ${GSRC}/include/contactSetting.h 
    ${GSRC}/include/debug.h 
    ${GSRC}/include/dof.h 
    ${GSRC}/include/dynamics/dynamics.h 
    ${GSRC}/include/dynamics/dynJoint.h 
    ${GSRC}/include/dynamics/dynamicsEngine.h 
    ${GSRC}/include/eigenGrasp.h 
    ${GSRC}/include/gloveInterface.h 
    ${GSRC}/include/grasp.h 
    ${GSRC}/include/graspRecord.h 
    ${GSRC}/include/gws.h 
    ${GSRC}/include/gwsprojection.h 
    ${GSRC}/include/ivmgr.h 
    ${GSRC}/include/jacobian.h 
    ${GSRC}/include/joint.h 
    ${GSRC}/include/kinematicChain.h 
    ${GSRC}/include/lmiOptimizer.h 
    ${GSRC}/include/material.h 
    ${GSRC}/include/matvec3D.h 
    ${GSRC}/include/matvecIO.h 
    ${GSRC}/include/maxdet.h 
    ${GSRC}/include/mytools.h 
    ${GSRC}/include/profiling.h 
    ${GSRC}/include/qhull_mutex.h 
    ${GSRC}/include/quality.h 
    ${GSRC}/include/plugin.h 
    ${GSRC}/include/SoArrow.h 
    ${GSRC}/include/SoComplexShape.h 
    ${GSRC}/include/SoTorquePointer.h 
    ${GSRC}/include/scanSimulator.h 
    ${GSRC}/include/timer_calls.h 
    ${GSRC}/include/triangle_inl.h 
    ${GSRC}/include/triangle.h 
    ${GSRC}/include/transform.h
    ${GSRC}/include/flockTransform.h
    ${GSRC}/include/worldElement.h 
    ${GSRC}/include/worldElementFactory.h 
    ${GSRC}/include/world.h 
    ${GSRC}/include/graspitCore.h
    ${GSRC}/include/graspitServer.h 
    ${GSRC}/include/graspitApp.h 
    ${GSRC}/include/arch.h
    ${GSRC}/include/FitParabola.h
    ${GSRC}/include/bodySensor.h
)
set (GRASPIT_PLANNER_HEADERS
    ${GSRC}/include/Planner/grasp_visualization.h 
    ${GSRC}/include/Planner/grasp_tester.h 
    ${GSRC}/include/Planner/grasp_preshape.h 
    ${GSRC}/include/Planner/grasp_presenter.h 
    ${GSRC}/include/Planner/grasp_planner.h 
    ${GSRC}/include/Planner/grasp_manager.h 
    ${GSRC}/include/Planner/grasp_grasps.h 
    ${GSRC}/include/Planner/grasp_directions.h 
    ${GSRC}/include/Planner/grasp_coordinates.h 
)
set (GRASPIT_EGPLANNER_HEADERS
    ${GSRC}/include/EGPlanner/search.h 
    ${GSRC}/include/EGPlanner/simAnn.h 
    ${GSRC}/include/EGPlanner/searchState.h 
    ${GSRC}/include/EGPlanner/searchStateImpl.h 
    ${GSRC}/include/EGPlanner/onLinePlanner.h 
    ${GSRC}/include/EGPlanner/egPlanner.h 
    ${GSRC}/include/EGPlanner/simAnnPlanner.h 
    ${GSRC}/include/EGPlanner/guidedPlanner.h 
    ${GSRC}/include/EGPlanner/loopPlanner.h 
    ${GSRC}/include/EGPlanner/timeTest.h 
    ${GSRC}/include/EGPlanner/graspTesterThread.h 
    ${GSRC}/include/EGPlanner/onLineGraspInterface.h 
    ${GSRC}/include/EGPlanner/listPlanner.h 
 
)

set (GRASPIT_EGPLANNER_ENERGY_HEADERS
    ${GSRC}/include/EGPlanner/energy/searchEnergy.h 
    ${GSRC}/include/EGPlanner/energy/searchEnergyFactory.h
    ${GSRC}/include/EGPlanner/energy/autoGraspQualityEnergy.h
    ${GSRC}/include/EGPlanner/energy/compliantEnergy.h
    ${GSRC}/include/EGPlanner/energy/contactEnergy.h 
    ${GSRC}/include/EGPlanner/energy/dynamicAutoGraspEnergy.h
    ${GSRC}/include/EGPlanner/energy/guidedAutoGraspEnergy.h 
    ${GSRC}/include/EGPlanner/energy/guidedPotentialQualityEnergy.h 
    ${GSRC}/include/EGPlanner/energy/potentialQualityEnergy.h
    ${GSRC}/include/EGPlanner/energy/strictAutoGraspEnergy.h
)

# all headers which are in the  cmdline
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_CMDLINE_HEADERS
set (GRASPIT_CMDLINE_HEADERS
    ${GSRC}/cmdline/cmdline.h 
)
	
# all headers which are in the src/Collision 
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_SRC_COLLISION_HEADERS
set (GRASPIT_SRC_COLLISION_HEADERS
    ${GSRC}/include/Collision/collisionInterface.h
    ${GSRC}/include/Collision/collisionStructures.h
)


# all headers which are in the include/math 
# directory and should be included
# in the installation should be listed in the
# variable GRASPIT_INCLUDE_MATH_HEADERS
set (GRASPIT_INCLUDE_MATH_HEADERS
    ${GSRC}/include/math/matrix.h 
)


if (CGDB)  
    set(CGDB_HEADERS
        include/DBase/graspit_db_model.h
        include/DBase/graspit_db_grasp.h
        include/DBase/graspit_db_planner.h
        include/DBase/taskDispatcher.h
        include/DBase/preGraspCheckTask.h
        include/DBase/graspClusteringTask.h
        include/DBase/graspTransferCheckTask.h
        include/DBase/tableCheckTask.h
        include/DBase/compliantGraspCopyTask.h
    )
    set(CGDB_MOC_HEADERS
        ui/DBase/dbaseDlg.h
        ui/DBase/dbasePlannerDlg.h
        include/DBase/dbase_grasp.h
        include/DBase/graspit_db_planner.h
        include/DBase/graspPlanningTask.h
    )

    set(CGDB_PLANNER_HEADERS
        include/DBase/DBPlanner/grasp.h
        include/DBase/DBPlanner/model.h
        include/DBase/DBPlanner/db_manager.h
        include/DBase/DBPlanner/database.h
        include/DBase/DBPlanner/sql_database_manager.h
        include/DBase/DBPlanner/task.h
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
        ${GSRC}/include/
        ${GSRC}/include/DBase/
        ${GSRC}/include/DBase/DBPlanner/
    )
endif (CGDB)  

if (PLY)
    set (PLY_HEADERS
        ply/ply.h
        ply/mesh_loader.h
    )
    set (PLY_SOURCES
        ply/ply.c
        ply/mesh_loader.cpp
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
    ${GSRC}/include/body.h
    ${GSRC}/include/robot.h
    ${GSRC}/include/world.h
    ${GSRC}/include/grasp.h
    ${GSRC}/include/worldElement.h
    ${GSRC}/include/graspitServer.h
    ${GSRC}/include/ivmgr.h
    ${GSRC}/include/EGPlanner/egPlanner.h
    ${GSRC}/include/EGPlanner/simAnn.h
    ${GSRC}/include/EGPlanner/loopPlanner.h
    ${GSRC}/include/EGPlanner/energy/searchEnergy.h
    ${GSRC}/include/EGPlanner/guidedPlanner.h
    ${GSRC}/include/Planner/grasp_tester.h
    ${GSRC}/include/robots/humanHand.h
    ${GSRC}/include/robots/m7tool.h
    ${GSRC}/include/robots/m7.h
    ${GSRC}/include/robots/barrett.h
    ${GSRC}/include/robots/robonaut.h
    ${GSRC}/include/robots/pr2Gripper.h
    ${GSRC}/include/robots/robotiq.h
    ${GSRC}/include/robots/mcGrip.h
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
    ${GSRC}/src/quality.cpp 
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
        ${GSRC}/include/Collision/Graspit)
    set (COLLISION_GRASPIT_HEADERS
        ${GSRC}/include/Collision/Graspit/collisionModel.h
        ${GSRC}/include/Collision/Graspit/collisionAlgorithms.h
        ${GSRC}/include/Collision/Graspit/collisionAlgorithms_inl.h
        ${GSRC}/include/Collision/Graspit/graspitCollision.h)
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
        ${GSRC}/include/dynamics/graspitDynamics.h 
        )
    set (GRASPIT_DYNAMICS_SOURCES
        ${GSRC}/src/dynamics/graspitDynamics.cpp
        )
endif(DYNAMICS_ENGINE STREQUAL GRASPIT_DYNAMICS)

if(DYNAMICS_ENGINE STREQUAL BULLET_DYNAMICS)
	set (GRASPIT_DEFINES
        ${GRASPIT_DEFINES}
        -DBULLET_DYNAMICS)
    set (GRASPIT_DYNAMICS_INCLUDES
        ${GSRC}/include/dynamics/bulletDynamics.h  
        )
    set (GRASPIT_DYNAMICS_SOURCES
        ${GSRC}/src/dynamics/bulletDynamics.cpp
        )

endif(DYNAMICS_ENGINE STREQUAL BULLET_DYNAMICS)


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
    ${GRASPIT_FORM_SOURCES}
    ${GRASPIT_IMAGE_SOURCES}
    ${UI_OUTFILES}
    ${MOC_OUTFILES}
)

add_library(graspit SHARED ${GRASPIT_LIBRARY_SOURCES})

## ---
## comment for future reference: This is required to build a static library
# add_library(graspit_static STATIC ${GRASPIT_LIBRARY_SOURCES})
# if ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))
#     target_compile_options(graspit_static PRIVATE -fPIC)  # does not work for bloom-release on saucy. Could 
# else ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))
#     set_target_properties(graspit_static PROPERTIES COMPILE_FLAGS -fPIC)
# endif ((CMAKE_VERSION VERSION_GREATER 2.8.12) OR (CMAKE_VERSION VERSION_EQUAL 2.8.12))
## ---

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
)

## ---
## comment for future reference: This is required to build a static library
# The graspit_simulator executable can still not be linked
# with a static library, the problem being the QPixmap resources
# (see also: https://wiki.qt.io/QtResources)
# With a static library they can't be found, and Q_INIT_RESOURCE can only be done
# with a .qrt file (which I tried to use but QT4_ADD_RESOURCES doesn't work).
# So for now, compile all sources into the graspit_simulator directly.
# set(SIMULATOR_SRC ${GRASPIT_LIBRARY_SOURCES} ${SIMULATOR_SRC})
# target_link_libraries(graspit_static ${GRASPIT_LINK_LIBRARIES})
# target_link_libraries(graspit_simulator ${GRASPIT_LINK_LIBRARIES})
## ---

## Link the shared library and the executable
target_link_libraries(graspit ${GRASPIT_LINK_LIBRARIES})
target_link_libraries(graspit_simulator graspit ${GRASPIT_LINK_LIBRARIES})

## Set install destination paths
set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
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

install(FILES
    ${GRASPIT_INCLUDEDIR_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}
)

install(FILES
    ${GRASPIT_CMDLINE_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/cmdline
)

install(FILES
     ${GRASPIT_DYNAMICS_INCLUDES}
    DESTINATION ${INCLUDE_DESTINATION}/dynamics
)

install(FILES
    ${GRASPIT_PLANNER_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/Planner
)

install(FILES
    ${GRASPIT_EGPLANNER_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/EGPlanner
)

install(FILES
    ${GRASPIT_EGPLANNER_ENERGY_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/EGPlanner/energy
)



if (PQP_COLLISION)
    install(FILES
        ${GRASPIT_INCLUDEDIR_HEADERS}
        ${COLLISION_PQP_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/Collision/PQP
    )
endif (PQP_COLLISION)


install(FILES
    ${GRASPIT_SRC_COLLISION_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/Collision
)

install(FILES
    ${GRASPIT_INCLUDE_MATH_HEADERS}
    DESTINATION ${INCLUDE_DESTINATION}/math
)

install(FILES
    ${UI_OUTFILES}
    DESTINATION ${INCLUDE_DESTINATION}/${GRASPIT_UI_INCLUDE_DIRNAME}
)

if (CGDB)
    install(FILES
        ${CGDB_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/DBase/
    )
    install(FILES
        ${CGDB_PLANNER_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/DBase/DBPlanner/
    )
endif (CGDB)

if (PLY)
    install(FILES
        ${PLY_HEADERS}
        DESTINATION ${INCLUDE_DESTINATION}/ply/
    )
endif (PLY)

#############
## Install ##
#############

install(TARGETS
    graspit 
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
    EXPORT graspit-targets
    DESTINATION ${LIB_DESTINATION})

install(EXPORT graspit-targets
    DESTINATION ${LIB_DESTINATION})

# For future reference: install graspitConfig.cmake as well as soon
# as it is added to the repository.
# set (GRASPIT_CONFIG_CMAKE graspitConfig.cmake)
install(FILES
    # cmake/${GRASPIT_CONFIG_CMAKE}
    CMakeMacros/FindBULLET.cmake
    CMakeMacros/FindSoQt4.cmake
    CMakeMacros/FindQhull.cmake
    DESTINATION ${LIB_DESTINATION}/${PROJECT_NAME}
)

## ---
## comment for future reference: This is required to build a static library
# install(TARGETS
#     graspit_static 
#     EXPORT graspit-static-targets
#     DESTINATION ${LIB_DESTINATION})
# install(EXPORT graspit-static-targets
#     DESTINATION ${LIB_DESTINATION})
## ---


#############
## Testing ##
#############

if(BUILD_TESTS)

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Make test executable
add_executable(graspit_simple_test
    ${GSRC}/test/simple_test.cpp
)

add_dependencies(graspit_simple_test googletest)

target_link_libraries(graspit_simple_test
   graspit
   ${GTEST_BOTH_LIBRARIES}
)

endif(BUILD_TESTS)
