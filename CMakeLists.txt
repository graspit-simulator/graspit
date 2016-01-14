cmake_minimum_required(VERSION 2.8.3)
project(graspit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  std_msgs
#)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")

find_package (Qhull REQUIRED)
find_package(SoQt4 REQUIRED)
find_package(LAPACK REQUIRED)

SET( QT_USE_QT3SUPPORT TRUE )
find_package(Qt4 COMPONENTS QtCore REQUIRED)

# include QT_USE_FILE is needed.
include (${QT_USE_FILE})

set (GSRC ${PROJECT_SOURCE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


####################################
## Define graspit variables ####### 
####################################

# if defined to false, the in-build collision in graspit is used instead.
set (PQP_COLLISION false)

set (GRASPIT_DEFINES 
    -DGRASPIT_RELEASE 
    #-DGRASPIT_DEBUG 
    -DGRASPIT_EXPORTS)

#--------------------------------------- Core sources ---------------------------------

set (GRASPIT_INCLUDES
  ${GSRC}
  ${GSRC}/include
  ${GSRC}/include/math
  ${GSRC}/include/EGPlanner
  ${GSRC}/include/Planner
  ${GSRC}/src/Collision
  ${GSRC}/cmdline
  # following only has to be included because of Pr2Gripper!
  ${GSRC}/ui
  # following needs to be included because basic files wouldn't compile without main window
  ${GSRC}/ui/Planner
  ${GSRC}/ui/EGPlanner
  ${GSRC}/src/optimizer
)


set (GRASPIT_HEADERS
    ${GSRC}/cmdline/cmdline.h 
    ${GSRC}/include/graspitParser.h 
    ${GSRC}/include/barrett.h 
	${GSRC}/include/body.h 
	${GSRC}/include/bBox.h 
	${GSRC}/include/bbox_inl.h 
	${GSRC}/include/contact.h 
	${GSRC}/include/contactSetting.h 
	${GSRC}/include/debug.h 
	${GSRC}/include/dof.h 
	${GSRC}/include/dynamics.h 
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
	${GSRC}/include/mcGrip.h 
	${GSRC}/include/mytools.h 
	${GSRC}/include/profiling.h 
	${GSRC}/include/puma560.h 
	${GSRC}/include/qhull_mutex.h 
	${GSRC}/include/quality.h 
    ${GSRC}/include/plugin.h 
	${GSRC}/include/pr2Gripper.h 
	${GSRC}/include/m7.h 
	${GSRC}/include/m7tool.h 
	${GSRC}/include/robonaut.h 
	${GSRC}/include/robot.h 
	${GSRC}/include/robotiq.h 
	${GSRC}/include/humanHand.h 
	${GSRC}/include/SoArrow.h 
	${GSRC}/include/SoComplexShape.h 
	${GSRC}/include/SoTorquePointer.h 
	${GSRC}/include/scanSimulator.h 
	${GSRC}/include/timer_calls.h 
	${GSRC}/include/triangle_inl.h 
	${GSRC}/include/triangle.h 
	${GSRC}/include/worldElement.h 
	${GSRC}/include/worldElementFactory.h 
	${GSRC}/include/world.h 
	${GSRC}/include/graspitGUI.h 
	${GSRC}/include/graspitServer.h 
	${GSRC}/include/graspitApp.h 
	${GSRC}/include/dynJoint.h 
	${GSRC}/include/arch.h 
	${GSRC}/include/math/matrix.h 
	${GSRC}/src/Collision/collisionInterface.h 
	${GSRC}/src/Collision/collisionStructures.h 
	${GSRC}/include/Planner/grasp_visualization.h 
	${GSRC}/include/Planner/grasp_tester.h 
	${GSRC}/include/Planner/grasp_preshape.h 
	${GSRC}/include/Planner/grasp_presenter.h 
	${GSRC}/include/Planner/grasp_planner.h 
	${GSRC}/include/Planner/grasp_manager.h 
	${GSRC}/include/Planner/grasp_grasps.h 
	${GSRC}/include/Planner/grasp_directions.h 
	${GSRC}/include/Planner/grasp_coordinates.h 
	${GSRC}/include/EGPlanner/search.h 
	${GSRC}/include/EGPlanner/simAnn.h 
	${GSRC}/include/EGPlanner/searchState.h 
	${GSRC}/include/EGPlanner/searchStateImpl.h 
	${GSRC}/include/EGPlanner/searchEnergy.h 
	${GSRC}/include/EGPlanner/onLinePlanner.h 
	${GSRC}/include/EGPlanner/egPlanner.h 
	${GSRC}/include/EGPlanner/simAnnPlanner.h 
	${GSRC}/include/EGPlanner/guidedPlanner.h 
	${GSRC}/include/EGPlanner/loopPlanner.h 
	${GSRC}/include/EGPlanner/timeTest.h 
	${GSRC}/include/EGPlanner/graspTesterThread.h 
	${GSRC}/include/EGPlanner/onLineGraspInterface.h 
	${GSRC}/include/EGPlanner/listPlanner.h 
	${GSRC}/include/FitParabola.h 
	${GSRC}/include/shadow.h
)


set (MOC_HEADERS
    ${MOC_HEADERS}
	${GSRC}/include/body.h
	${GSRC}/include/robot.h
	${GSRC}/include/world.h
	${GSRC}/include/grasp.h
	${GSRC}/include/worldElement.h
	${GSRC}/include/graspitServer.h
	${GSRC}/include/EGPlanner/egPlanner.h
	${GSRC}/include/EGPlanner/simAnn.h
	${GSRC}/include/EGPlanner/loopPlanner.h
	${GSRC}/include/EGPlanner/searchEnergy.h
	${GSRC}/include/EGPlanner/guidedPlanner.h
	${GSRC}/include/Planner/grasp_tester.h
	${GSRC}/include/humanHand.h
	${GSRC}/include/m7tool.h
	${GSRC}/include/m7.h
	${GSRC}/include/barrett.h
	${GSRC}/include/robonaut.h
	${GSRC}/include/pr2Gripper.h
	${GSRC}/include/robotiq.h
	${GSRC}/include/mcGrip.h
	${GSRC}/include/ivmgr.h
) 


set (GRASPIT_CORE_SOURCES
    ${GSRC}/src/graspitParser.cpp 
    ${GSRC}/src/arch.cpp 
	${GSRC}/src/barrett.cpp 
	${GSRC}/src/bBox.cpp 
	${GSRC}/src/body.cpp 
	${GSRC}/src/contact.cpp 
	${GSRC}/src/contactSetting.cpp 
	${GSRC}/src/dof.cpp 
	${GSRC}/src/dynamics.cpp 
	${GSRC}/src/dynJoint.cpp 
	${GSRC}/src/eigenGrasp.cpp 
	${GSRC}/src/gloveInterface.cpp 
	${GSRC}/src/grasp.cpp 
	${GSRC}/src/graspitGUI.cpp 
	${GSRC}/src/graspitServer.cpp 
	${GSRC}/src/graspitApp.cpp 
	${GSRC}/src/graspRecord.cpp 
	${GSRC}/src/gws.cpp 
	${GSRC}/src/gwsprojection.cpp 
	${GSRC}/src/humanHand.cpp 
	${GSRC}/src/ivmgr.cpp 
	${GSRC}/src/jacobian.cpp 
	${GSRC}/src/joint.cpp 
	${GSRC}/src/kinematicChain.cpp 
	${GSRC}/src/lmiOptimizer.cpp 
	${GSRC}/src/material.cpp 
	${GSRC}/src/matvec.cpp 
	${GSRC}/src/matvecIO.cpp 
	${GSRC}/src/maxdet_src.cpp 
	${GSRC}/src/mcGrip.cpp 
	${GSRC}/src/mytools.cpp 
	${GSRC}/src/profiling.cpp 
	${GSRC}/src/pr2Gripper.cpp 
	${GSRC}/src/m7.cpp 
	${GSRC}/src/m7tool.cpp 
    ${GSRC}/src/plugin.cpp 
	${GSRC}/src/puma560.cpp 
	${GSRC}/src/quality.cpp 
	${GSRC}/src/robonaut.cpp 
	${GSRC}/src/robot.cpp 
	${GSRC}/src/robotiq.cpp 
	${GSRC}/src/scanSimulator.cpp 
	${GSRC}/src/shadow.cpp 
	${GSRC}/src/SoArrow.cpp 
	${GSRC}/src/SoComplexShape.cpp 
	${GSRC}/src/SoTorquePointer.cpp 	
	${GSRC}/src/triangle.cpp 
	${GSRC}/src/world.cpp 
	${GSRC}/src/worldElement.cpp 
	${GSRC}/src/worldElementFactory.cpp 
	${GSRC}/src/math/matrix.cpp 
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
	${GSRC}/src/EGPlanner/searchEnergy.cpp 
	${GSRC}/src/EGPlanner/onLinePlanner.cpp 
	${GSRC}/src/EGPlanner/egPlanner.cpp 
	${GSRC}/src/EGPlanner/simAnnPlanner.cpp 
	${GSRC}/src/EGPlanner/guidedPlanner.cpp 
	${GSRC}/src/EGPlanner/loopPlanner.cpp 
	${GSRC}/src/EGPlanner/timeTest.cpp 
	${GSRC}/src/EGPlanner/graspTesterThread.cpp 
	${GSRC}/src/EGPlanner/onLineGraspInterface.cpp 
	${GSRC}/src/EGPlanner/listPlanner.cpp 
)

#--------------------------------------- Collision library ---------------------------------


if (PQP_COLLISION) 
	set(GRASPIT_DEFINES 
        ${GRASPIT_DEFINES}
         -DPQP_COLLISION)
    #set (GRASPIT_INCLUDES 
    #    ${GRASPIT_INCLUDES} 
    #    ${GSRC}/src/Collision/PQP PQP-VCOLLIDE/src 
    #    ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src 
    #    ${GSRC}/PQP-VCOLLIDE/ivcollide) 
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
    	${GSRC}/PQP-VCOLLIDE/include 
        ${GSRC}/PQP-VCOLLIDE/PQP_v1.1/src 
        ${GSRC}/PQP-VCOLLIDE/ivcollide)
	set (GRASPIT_HEADERS 
        ${GRASPIT_HEADERS} 
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

else (PQP_COLLISION)
 
	set(GRASPIT_DEFINES 
        ${GRASPIT_DEFINES}
	    -DGRASPIT_COLLISION)
    set (GRASPIT_INCLUDES
        ${GRASPIT_INCLUDES}
	    ${GSRC}/src/Collision/Graspit)
	set (GRASPIT_HEADERS 
        ${GRASPIT_HEADERS} 
        ${GSRC}/src/Collision/Graspit/collisionModel.h 
        ${GSRC}/src/Collision/Graspit/collisionAlgorithms.h 
        ${GSRC}/src/Collision/Graspit/collisionAlgorithms_inl.h 
        ${GSRC}/src/Collision/Graspit/graspitCollision.h)
    set (GRASPIT_COLLISION_SOURCES
        ${GSRC}/src/Collision/Graspit/collisionModel.cpp 
        ${GSRC}/src/Collision/Graspit/collisionAlgorithms.cpp 
        ${GSRC}/src/Collision/Graspit/graspitCollision.cpp)

endif (PQP_COLLISION)



#--------------------------------------- User interface: main window and dialogs ---------------------------------

set (GRASPIT_INCLUDES
    ${GRASPIT_INCLUDES}
    ${GSRC}/ui 
    ${GSRC}/ui/Planner 
    ${GSRC}/ui/EGPlanner
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

#message ("*****************************************")
#message (${GRASPIT_IMAGES})

#-------------------------------------- The TinyXML XML parser ---------------------------------------------------

set (GRASPIT_INCLUDES
    ${GRASPIT_INCLUDES}
    ${GSRC}/tinyxml
)
set (GRASPIT_HEADERS 
    ${GRASPIT_HEADERS} 
    ${GSRC}/tinyxml/tinyxml.h 
    ${GSRC}/tinyxml/tinystr.h
)

set (GRASPIT_CORE_SOURCES
    ${GRASPIT_CORE_SOURCES}
    ${GSRC}/tinyxml/tinyxmlparser.cpp
)


#-------------------------------------- Optimizer ---------------------

set (GRASPIT_INCLUDES
    ${GRASPIT_INCLUDES}
    ${GSRC}/src/optimizer
)
set (GRASPIT_HEADERS 
    ${GRASPIT_HEADERS} 
    ${GSRC}/src/optimizer/optimizerDlg.h
)

set (GRASPIT_OPTIMIZER_SOURCES
    ${GSRC}/src/optimizer/optimizerDlg.cpp
)

set (GRASPIT_FORMS
    ${GRASPIT_FORMS}
    ${GSRC}/src/optimizer/optimizerDlg.ui)

if (CGDB)  # for colombia grasp data base, but this is not elsewhere included in this cmakefile yet.
    set (GRASPIT_OPTIMIZER_SOURCES
        ${GRASPIT_OPTIMIZER_SOURCES}
        ${GSRC}/src/optimizer/eigenTorques.cpp)

    set (GRASPIT_HEADERS 
        ${GRASPIT_HEADERS} 
        ${GSRC}/src/optimizer/eigenTorques.h
)
endif (CGDB)    

set (MOC_HEADERS
    ${MOC_HEADERS}
	${GSRC}/src/optimizer/optimizerDlg.h
)



########################
## Generate Qt files  ## 
########################

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

SET(GRASPIT_FORM_SOURCES 
    ${GRASPIT_FORM_SOURCES}     
    ${CMAKE_CURRENT_BINARY_DIR}/qt_image_collection.cpp)
     

#set (GRASPIT_DEVEL_PATH ${CMAKE_CURRENT_BINARY_DIR})
set (GRASPIT_DEVEL_UI_DIRNAME ui)
set (GRASPIT_DEVEL_UI_PATH ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/${GRASPIT_DEVEL_UI_DIRNAME})
file(MAKE_DIRECTORY ${GRASPIT_DEVEL_UI_PATH})

## Make Qt Moc files of GraspIT
set (OLD_CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_CURRENT_BINARY_DIR ${GRASPIT_DEVEL_UI_PATH}) #artificially changing output location for UI files
QT4_WRAP_UI(UI_OUTFILES ${GRASPIT_FORMS})
set (CMAKE_CURRENT_BINARY_DIR ${OLD_CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_CPP(MOC_OUTFILES ${MOC_HEADERS})

set (GRASPIT_INCLUDES 
     ${GRASPIT_INCLUDES}
     ${GRASPIT_DEVEL_UI_PATH}
     ${SOQT_INCLUDE_DIRS}
     ${QT_INCLUDES}
     ${QT_INCLUDE_DIR}
     ${QT_QT3SUPPORT_INCLUDE_DIR}
     ${QHULL_INCLUDE_DIRS}
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${GRASPIT_INCLUDES}
    ${catkin_INCLUDES}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${GSRC}/include ${GRASPIT_INCLUDES}  
  LIBRARIES graspit
#  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS ${QT_LIBRARIES} ${QT_QT3SUPPORT_LIBRARY} ${QHULL_LIBRARIES} ${SOQT_LIBRARY}
)


############
## Build  ## 
############

add_definitions(${GRASPIT_DEFINES} -g)

## Declare a C++ library
add_library(graspit 
	${GRASPIT_CORE_SOURCES}    
	${GRASPIT_COLLISION_SOURCES}    
	${GRASPIT_OPTIMIZER_SOURCES}    
	${GRASPIT_FORM_SOURCES}    
    ${UI_OUTFILES}
    ${MOC_OUTFILES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(graspit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(graspit_simulator 
    ${GSRC}/src/main.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(graspit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(graspit
   ${catkin_LIBRARIES}
   ${QT_LIBRARIES}
   ${QT_QT3SUPPORT_LIBRARY}
   ${QHULL_LIBRARIES}
   ${SOQT_LIBRARY}
   ${LAPACK_LIBRARIES}
)
target_link_libraries(graspit_simulator
   ${catkin_LIBRARIES}
   graspit 
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS graspit graspit_simulator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

#install(FILES
#    ${GRASPIT_HEADERS}
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

# XXX still check this here, not tested yet
#install(FILES
#    ${UI_OUTFILES}
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/${GRASPIT_DEVEL_UI_DIRNAME}
#)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_graspit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
